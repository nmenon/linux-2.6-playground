/*
 *	fs/proc/vmcore.c Interface for accessing the crash
 * 				 dump from the system's previous life.
 * 	Heavily borrowed from fs/proc/kcore.c
 *	Created by: Hariprasad Nellitheertha (hari@in.ibm.com)
 *	Copyright (C) IBM Corporation, 2004. All rights reserved
 *
 */

#include <linux/mm.h>
#include <linux/kcore.h>
#include <linux/user.h>
#include <linux/elf.h>
#include <linux/elfcore.h>
#include <linux/export.h>
#include <linux/slab.h>
#include <linux/highmem.h>
#include <linux/printk.h>
#include <linux/bootmem.h>
#include <linux/init.h>
#include <linux/crash_dump.h>
#include <linux/list.h>
#include <asm/uaccess.h>
#include <asm/io.h>
#include "internal.h"

/* List representing chunks of contiguous memory areas and their offsets in
 * vmcore file.
 */
static LIST_HEAD(vmcore_list);

/* Stores the pointer to the buffer containing kernel elf core headers. */
static char *elfcorebuf;
static size_t elfcorebuf_sz;
static size_t elfcorebuf_sz_orig;

/* Total size of vmcore file. */
static u64 vmcore_size;

static struct proc_dir_entry *proc_vmcore = NULL;

/*
 * Returns > 0 for RAM pages, 0 for non-RAM pages, < 0 on error
 * The called function has to take care of module refcounting.
 */
static int (*oldmem_pfn_is_ram)(unsigned long pfn);

int register_oldmem_pfn_is_ram(int (*fn)(unsigned long pfn))
{
	if (oldmem_pfn_is_ram)
		return -EBUSY;
	oldmem_pfn_is_ram = fn;
	return 0;
}
EXPORT_SYMBOL_GPL(register_oldmem_pfn_is_ram);

void unregister_oldmem_pfn_is_ram(void)
{
	oldmem_pfn_is_ram = NULL;
	wmb();
}
EXPORT_SYMBOL_GPL(unregister_oldmem_pfn_is_ram);

static int pfn_is_ram(unsigned long pfn)
{
	int (*fn)(unsigned long pfn);
	/* pfn is ram unless fn() checks pagetype */
	int ret = 1;

	/*
	 * Ask hypervisor if the pfn is really ram.
	 * A ballooned page contains no data and reading from such a page
	 * will cause high load in the hypervisor.
	 */
	fn = oldmem_pfn_is_ram;
	if (fn)
		ret = fn(pfn);

	return ret;
}

/* Reads a page from the oldmem device from given offset. */
static ssize_t read_from_oldmem(char *buf, size_t count,
				u64 *ppos, int userbuf)
{
	unsigned long pfn, offset;
	size_t nr_bytes;
	ssize_t read = 0, tmp;

	if (!count)
		return 0;

	offset = (unsigned long)(*ppos % PAGE_SIZE);
	pfn = (unsigned long)(*ppos / PAGE_SIZE);

	do {
		if (count > (PAGE_SIZE - offset))
			nr_bytes = PAGE_SIZE - offset;
		else
			nr_bytes = count;

		/* If pfn is not ram, return zeros for sparse dump files */
		if (pfn_is_ram(pfn) == 0)
			memset(buf, 0, nr_bytes);
		else {
			tmp = copy_oldmem_page(pfn, buf, nr_bytes,
						offset, userbuf);
			if (tmp < 0)
				return tmp;
		}
		*ppos += nr_bytes;
		count -= nr_bytes;
		buf += nr_bytes;
		read += nr_bytes;
		++pfn;
		offset = 0;
	} while (count);

	return read;
}

/* Read from the ELF header and then the crash dump. On error, negative value is
 * returned otherwise number of bytes read are returned.
 */
static ssize_t read_vmcore(struct file *file, char __user *buffer,
				size_t buflen, loff_t *fpos)
{
	ssize_t acc = 0, tmp;
	size_t tsz;
	u64 start;
	struct vmcore *m = NULL;

	if (buflen == 0 || *fpos >= vmcore_size)
		return 0;

	/* trim buflen to not go beyond EOF */
	if (buflen > vmcore_size - *fpos)
		buflen = vmcore_size - *fpos;

	/* Read ELF core header */
	if (*fpos < elfcorebuf_sz) {
		tsz = elfcorebuf_sz - *fpos;
		if (buflen < tsz)
			tsz = buflen;
		if (copy_to_user(buffer, elfcorebuf + *fpos, tsz))
			return -EFAULT;
		buflen -= tsz;
		*fpos += tsz;
		buffer += tsz;
		acc += tsz;

		/* leave now if filled buffer already */
		if (buflen == 0)
			return acc;
	}

	list_for_each_entry(m, &vmcore_list, list) {
		if (*fpos < m->offset + m->size) {
			tsz = m->offset + m->size - *fpos;
			if (buflen < tsz)
				tsz = buflen;
			start = m->paddr + *fpos - m->offset;
			tmp = read_from_oldmem(buffer, tsz, &start, 1);
			if (tmp < 0)
				return tmp;
			buflen -= tsz;
			*fpos += tsz;
			buffer += tsz;
			acc += tsz;

			/* leave now if filled buffer already */
			if (buflen == 0)
				return acc;
		}
	}

	return acc;
}

static const struct file_operations proc_vmcore_operations = {
	.read		= read_vmcore,
	.llseek		= default_llseek,
};

static struct vmcore* __init get_new_element(void)
{
	return kzalloc(sizeof(struct vmcore), GFP_KERNEL);
}

static u64 __init get_vmcore_size_elf64(char *elfptr, size_t elfsz)
{
	int i;
	u64 size;
	Elf64_Ehdr *ehdr_ptr;
	Elf64_Phdr *phdr_ptr;

	ehdr_ptr = (Elf64_Ehdr *)elfptr;
	phdr_ptr = (Elf64_Phdr*)(elfptr + sizeof(Elf64_Ehdr));
	size = elfsz;
	for (i = 0; i < ehdr_ptr->e_phnum; i++) {
		size += phdr_ptr->p_memsz;
		phdr_ptr++;
	}
	return size;
}

static u64 __init get_vmcore_size_elf32(char *elfptr, size_t elfsz)
{
	int i;
	u64 size;
	Elf32_Ehdr *ehdr_ptr;
	Elf32_Phdr *phdr_ptr;

	ehdr_ptr = (Elf32_Ehdr *)elfptr;
	phdr_ptr = (Elf32_Phdr*)(elfptr + sizeof(Elf32_Ehdr));
	size = elfsz;
	for (i = 0; i < ehdr_ptr->e_phnum; i++) {
		size += phdr_ptr->p_memsz;
		phdr_ptr++;
	}
	return size;
}

/* Merges all the PT_NOTE headers into one. */
static int __init merge_note_headers_elf64(char *elfptr, size_t *elfsz,
						struct list_head *vc_list)
{
	int i, nr_ptnote=0, rc=0;
	char *tmp;
	Elf64_Ehdr *ehdr_ptr;
	Elf64_Phdr phdr, *phdr_ptr;
	Elf64_Nhdr *nhdr_ptr;
	u64 phdr_sz = 0, note_off;

	ehdr_ptr = (Elf64_Ehdr *)elfptr;
	phdr_ptr = (Elf64_Phdr*)(elfptr + sizeof(Elf64_Ehdr));
	for (i = 0; i < ehdr_ptr->e_phnum; i++, phdr_ptr++) {
		int j;
		void *notes_section;
		struct vmcore *new;
		u64 offset, max_sz, sz, real_sz = 0;
		if (phdr_ptr->p_type != PT_NOTE)
			continue;
		nr_ptnote++;
		max_sz = phdr_ptr->p_memsz;
		offset = phdr_ptr->p_offset;
		notes_section = kmalloc(max_sz, GFP_KERNEL);
		if (!notes_section)
			return -ENOMEM;
		rc = read_from_oldmem(notes_section, max_sz, &offset, 0);
		if (rc < 0) {
			kfree(notes_section);
			return rc;
		}
		nhdr_ptr = notes_section;
		for (j = 0; j < max_sz; j += sz) {
			if (nhdr_ptr->n_namesz == 0)
				break;
			sz = sizeof(Elf64_Nhdr) +
				((nhdr_ptr->n_namesz + 3) & ~3) +
				((nhdr_ptr->n_descsz + 3) & ~3);
			real_sz += sz;
			nhdr_ptr = (Elf64_Nhdr*)((char*)nhdr_ptr + sz);
		}

		/* Add this contiguous chunk of notes section to vmcore list.*/
		new = get_new_element();
		if (!new) {
			kfree(notes_section);
			return -ENOMEM;
		}
		new->paddr = phdr_ptr->p_offset;
		new->size = real_sz;
		list_add_tail(&new->list, vc_list);
		phdr_sz += real_sz;
		kfree(notes_section);
	}

	/* Prepare merged PT_NOTE program header. */
	phdr.p_type    = PT_NOTE;
	phdr.p_flags   = 0;
	note_off = sizeof(Elf64_Ehdr) +
			(ehdr_ptr->e_phnum - nr_ptnote +1) * sizeof(Elf64_Phdr);
	phdr.p_offset  = note_off;
	phdr.p_vaddr   = phdr.p_paddr = 0;
	phdr.p_filesz  = phdr.p_memsz = phdr_sz;
	phdr.p_align   = 0;

	/* Add merged PT_NOTE program header*/
	tmp = elfptr + sizeof(Elf64_Ehdr);
	memcpy(tmp, &phdr, sizeof(phdr));
	tmp += sizeof(phdr);

	/* Remove unwanted PT_NOTE program headers. */
	i = (nr_ptnote - 1) * sizeof(Elf64_Phdr);
	*elfsz = *elfsz - i;
	memmove(tmp, tmp+i, ((*elfsz)-sizeof(Elf64_Ehdr)-sizeof(Elf64_Phdr)));
	memset(elfptr + *elfsz, 0, i);
	*elfsz = roundup(*elfsz, PAGE_SIZE);

	/* Modify e_phnum to reflect merged headers. */
	ehdr_ptr->e_phnum = ehdr_ptr->e_phnum - nr_ptnote + 1;

	return 0;
}

/* Merges all the PT_NOTE headers into one. */
static int __init merge_note_headers_elf32(char *elfptr, size_t *elfsz,
						struct list_head *vc_list)
{
	int i, nr_ptnote=0, rc=0;
	char *tmp;
	Elf32_Ehdr *ehdr_ptr;
	Elf32_Phdr phdr, *phdr_ptr;
	Elf32_Nhdr *nhdr_ptr;
	u64 phdr_sz = 0, note_off;

	ehdr_ptr = (Elf32_Ehdr *)elfptr;
	phdr_ptr = (Elf32_Phdr*)(elfptr + sizeof(Elf32_Ehdr));
	for (i = 0; i < ehdr_ptr->e_phnum; i++, phdr_ptr++) {
		int j;
		void *notes_section;
		struct vmcore *new;
		u64 offset, max_sz, sz, real_sz = 0;
		if (phdr_ptr->p_type != PT_NOTE)
			continue;
		nr_ptnote++;
		max_sz = phdr_ptr->p_memsz;
		offset = phdr_ptr->p_offset;
		notes_section = kmalloc(max_sz, GFP_KERNEL);
		if (!notes_section)
			return -ENOMEM;
		rc = read_from_oldmem(notes_section, max_sz, &offset, 0);
		if (rc < 0) {
			kfree(notes_section);
			return rc;
		}
		nhdr_ptr = notes_section;
		for (j = 0; j < max_sz; j += sz) {
			if (nhdr_ptr->n_namesz == 0)
				break;
			sz = sizeof(Elf32_Nhdr) +
				((nhdr_ptr->n_namesz + 3) & ~3) +
				((nhdr_ptr->n_descsz + 3) & ~3);
			real_sz += sz;
			nhdr_ptr = (Elf32_Nhdr*)((char*)nhdr_ptr + sz);
		}

		/* Add this contiguous chunk of notes section to vmcore list.*/
		new = get_new_element();
		if (!new) {
			kfree(notes_section);
			return -ENOMEM;
		}
		new->paddr = phdr_ptr->p_offset;
		new->size = real_sz;
		list_add_tail(&new->list, vc_list);
		phdr_sz += real_sz;
		kfree(notes_section);
	}

	/* Prepare merged PT_NOTE program header. */
	phdr.p_type    = PT_NOTE;
	phdr.p_flags   = 0;
	note_off = sizeof(Elf32_Ehdr) +
			(ehdr_ptr->e_phnum - nr_ptnote +1) * sizeof(Elf32_Phdr);
	phdr.p_offset  = note_off;
	phdr.p_vaddr   = phdr.p_paddr = 0;
	phdr.p_filesz  = phdr.p_memsz = phdr_sz;
	phdr.p_align   = 0;

	/* Add merged PT_NOTE program header*/
	tmp = elfptr + sizeof(Elf32_Ehdr);
	memcpy(tmp, &phdr, sizeof(phdr));
	tmp += sizeof(phdr);

	/* Remove unwanted PT_NOTE program headers. */
	i = (nr_ptnote - 1) * sizeof(Elf32_Phdr);
	*elfsz = *elfsz - i;
	memmove(tmp, tmp+i, ((*elfsz)-sizeof(Elf32_Ehdr)-sizeof(Elf32_Phdr)));
	memset(elfptr + *elfsz, 0, i);
	*elfsz = roundup(*elfsz, PAGE_SIZE);

	/* Modify e_phnum to reflect merged headers. */
	ehdr_ptr->e_phnum = ehdr_ptr->e_phnum - nr_ptnote + 1;

	return 0;
}

/* Add memory chunks represented by program headers to vmcore list. Also update
 * the new offset fields of exported program headers. */
static int __init process_ptload_program_headers_elf64(char *elfptr,
						size_t elfsz,
						struct list_head *vc_list)
{
	int i;
	Elf64_Ehdr *ehdr_ptr;
	Elf64_Phdr *phdr_ptr;
	loff_t vmcore_off;
	struct vmcore *new;

	ehdr_ptr = (Elf64_Ehdr *)elfptr;
	phdr_ptr = (Elf64_Phdr*)(elfptr + sizeof(Elf64_Ehdr)); /* PT_NOTE hdr */

	/* First program header is PT_NOTE header. */
	vmcore_off = elfsz +
			phdr_ptr->p_memsz; /* Note sections */

	for (i = 0; i < ehdr_ptr->e_phnum; i++, phdr_ptr++) {
		u64 paddr, start, end, size;

		if (phdr_ptr->p_type != PT_LOAD)
			continue;

		paddr = phdr_ptr->p_offset;
		start = rounddown(paddr, PAGE_SIZE);
		end = roundup(paddr + phdr_ptr->p_memsz, PAGE_SIZE);
		size = end - start;

		/* Add this contiguous chunk of memory to vmcore list.*/
		new = get_new_element();
		if (!new)
			return -ENOMEM;
		new->paddr = start;
		new->size = size;
		list_add_tail(&new->list, vc_list);

		/* Update the program header offset. */
		phdr_ptr->p_offset = vmcore_off + (paddr - start);
		vmcore_off = vmcore_off + size;
	}
	return 0;
}

static int __init process_ptload_program_headers_elf32(char *elfptr,
						size_t elfsz,
						struct list_head *vc_list)
{
	int i;
	Elf32_Ehdr *ehdr_ptr;
	Elf32_Phdr *phdr_ptr;
	loff_t vmcore_off;
	struct vmcore *new;

	ehdr_ptr = (Elf32_Ehdr *)elfptr;
	phdr_ptr = (Elf32_Phdr*)(elfptr + sizeof(Elf32_Ehdr)); /* PT_NOTE hdr */

	/* First program header is PT_NOTE header. */
	vmcore_off = elfsz +
			phdr_ptr->p_memsz; /* Note sections */

	for (i = 0; i < ehdr_ptr->e_phnum; i++, phdr_ptr++) {
		u64 paddr, start, end, size;

		if (phdr_ptr->p_type != PT_LOAD)
			continue;

		paddr = phdr_ptr->p_offset;
		start = rounddown(paddr, PAGE_SIZE);
		end = roundup(paddr + phdr_ptr->p_memsz, PAGE_SIZE);
		size = end - start;

		/* Add this contiguous chunk of memory to vmcore list.*/
		new = get_new_element();
		if (!new)
			return -ENOMEM;
		new->paddr = start;
		new->size = size;
		list_add_tail(&new->list, vc_list);

		/* Update the program header offset */
		phdr_ptr->p_offset = vmcore_off + (paddr - start);
		vmcore_off = vmcore_off + size;
	}
	return 0;
}

/* Sets offset fields of vmcore elements. */
static void __init set_vmcore_list_offsets(size_t elfsz,
					   struct list_head *vc_list)
{
	loff_t vmcore_off;
	struct vmcore *m;

	/* Skip Elf header and program headers. */
	vmcore_off = elfsz;

	list_for_each_entry(m, vc_list, list) {
		m->offset = vmcore_off;
		vmcore_off += m->size;
	}
}

static void free_elfcorebuf(void)
{
	free_pages((unsigned long)elfcorebuf, get_order(elfcorebuf_sz_orig));
	elfcorebuf = NULL;
}

static int __init parse_crash_elf64_headers(void)
{
	int rc=0;
	Elf64_Ehdr ehdr;
	u64 addr;

	addr = elfcorehdr_addr;

	/* Read Elf header */
	rc = read_from_oldmem((char*)&ehdr, sizeof(Elf64_Ehdr), &addr, 0);
	if (rc < 0)
		return rc;

	/* Do some basic Verification. */
	if (memcmp(ehdr.e_ident, ELFMAG, SELFMAG) != 0 ||
		(ehdr.e_type != ET_CORE) ||
		!vmcore_elf64_check_arch(&ehdr) ||
		ehdr.e_ident[EI_CLASS] != ELFCLASS64 ||
		ehdr.e_ident[EI_VERSION] != EV_CURRENT ||
		ehdr.e_version != EV_CURRENT ||
		ehdr.e_ehsize != sizeof(Elf64_Ehdr) ||
		ehdr.e_phentsize != sizeof(Elf64_Phdr) ||
		ehdr.e_phnum == 0) {
		pr_warn("Warning: Core image elf header is not sane\n");
		return -EINVAL;
	}

	/* Read in all elf headers. */
	elfcorebuf_sz_orig = sizeof(Elf64_Ehdr) +
				ehdr.e_phnum * sizeof(Elf64_Phdr);
	elfcorebuf_sz = elfcorebuf_sz_orig;
	elfcorebuf = (void *)__get_free_pages(GFP_KERNEL | __GFP_ZERO,
					      get_order(elfcorebuf_sz_orig));
	if (!elfcorebuf)
		return -ENOMEM;
	addr = elfcorehdr_addr;
	rc = read_from_oldmem(elfcorebuf, elfcorebuf_sz_orig, &addr, 0);
	if (rc < 0)
		goto fail;

	/* Merge all PT_NOTE headers into one. */
	rc = merge_note_headers_elf64(elfcorebuf, &elfcorebuf_sz, &vmcore_list);
	if (rc)
		goto fail;
	rc = process_ptload_program_headers_elf64(elfcorebuf, elfcorebuf_sz,
							&vmcore_list);
	if (rc)
		goto fail;
	set_vmcore_list_offsets(elfcorebuf_sz, &vmcore_list);
	return 0;
fail:
	free_elfcorebuf();
	return rc;
}

static int __init parse_crash_elf32_headers(void)
{
	int rc=0;
	Elf32_Ehdr ehdr;
	u64 addr;

	addr = elfcorehdr_addr;

	/* Read Elf header */
	rc = read_from_oldmem((char*)&ehdr, sizeof(Elf32_Ehdr), &addr, 0);
	if (rc < 0)
		return rc;

	/* Do some basic Verification. */
	if (memcmp(ehdr.e_ident, ELFMAG, SELFMAG) != 0 ||
		(ehdr.e_type != ET_CORE) ||
		!elf_check_arch(&ehdr) ||
		ehdr.e_ident[EI_CLASS] != ELFCLASS32||
		ehdr.e_ident[EI_VERSION] != EV_CURRENT ||
		ehdr.e_version != EV_CURRENT ||
		ehdr.e_ehsize != sizeof(Elf32_Ehdr) ||
		ehdr.e_phentsize != sizeof(Elf32_Phdr) ||
		ehdr.e_phnum == 0) {
		pr_warn("Warning: Core image elf header is not sane\n");
		return -EINVAL;
	}

	/* Read in all elf headers. */
	elfcorebuf_sz_orig = sizeof(Elf32_Ehdr) + ehdr.e_phnum * sizeof(Elf32_Phdr);
	elfcorebuf_sz = elfcorebuf_sz_orig;
	elfcorebuf = (void *)__get_free_pages(GFP_KERNEL | __GFP_ZERO,
					      get_order(elfcorebuf_sz_orig));
	if (!elfcorebuf)
		return -ENOMEM;
	addr = elfcorehdr_addr;
	rc = read_from_oldmem(elfcorebuf, elfcorebuf_sz_orig, &addr, 0);
	if (rc < 0)
		goto fail;

	/* Merge all PT_NOTE headers into one. */
	rc = merge_note_headers_elf32(elfcorebuf, &elfcorebuf_sz, &vmcore_list);
	if (rc)
		goto fail;
	rc = process_ptload_program_headers_elf32(elfcorebuf, elfcorebuf_sz,
								&vmcore_list);
	if (rc)
		goto fail;
	set_vmcore_list_offsets(elfcorebuf_sz, &vmcore_list);
	return 0;
fail:
	free_elfcorebuf();
	return rc;
}

static int __init parse_crash_elf_headers(void)
{
	unsigned char e_ident[EI_NIDENT];
	u64 addr;
	int rc=0;

	addr = elfcorehdr_addr;
	rc = read_from_oldmem(e_ident, EI_NIDENT, &addr, 0);
	if (rc < 0)
		return rc;
	if (memcmp(e_ident, ELFMAG, SELFMAG) != 0) {
		pr_warn("Warning: Core image elf header not found\n");
		return -EINVAL;
	}

	if (e_ident[EI_CLASS] == ELFCLASS64) {
		rc = parse_crash_elf64_headers();
		if (rc)
			return rc;

		/* Determine vmcore size. */
		vmcore_size = get_vmcore_size_elf64(elfcorebuf, elfcorebuf_sz);
	} else if (e_ident[EI_CLASS] == ELFCLASS32) {
		rc = parse_crash_elf32_headers();
		if (rc)
			return rc;

		/* Determine vmcore size. */
		vmcore_size = get_vmcore_size_elf32(elfcorebuf, elfcorebuf_sz);
	} else {
		pr_warn("Warning: Core image elf header is not sane\n");
		return -EINVAL;
	}
	return 0;
}

/* Init function for vmcore module. */
static int __init vmcore_init(void)
{
	int rc = 0;

	/* If elfcorehdr= has been passed in cmdline, then capture the dump.*/
	if (!(is_vmcore_usable()))
		return rc;
	rc = parse_crash_elf_headers();
	if (rc) {
		pr_warn("Kdump: vmcore not initialized\n");
		return rc;
	}

	proc_vmcore = proc_create("vmcore", S_IRUSR, NULL, &proc_vmcore_operations);
	if (proc_vmcore)
		proc_vmcore->size = vmcore_size;
	return 0;
}
module_init(vmcore_init)

/* Cleanup function for vmcore module. */
void vmcore_cleanup(void)
{
	struct list_head *pos, *next;

	if (proc_vmcore) {
		proc_remove(proc_vmcore);
		proc_vmcore = NULL;
	}

	/* clear the vmcore list. */
	list_for_each_safe(pos, next, &vmcore_list) {
		struct vmcore *m;

		m = list_entry(pos, struct vmcore, list);
		list_del(&m->list);
		kfree(m);
	}
	free_elfcorebuf();
}
EXPORT_SYMBOL_GPL(vmcore_cleanup);
