#ifndef _LINUX_COREDUMP_H
#define _LINUX_COREDUMP_H

#include <linux/types.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <asm/siginfo.h>

/*
 * These are the only things you should do on a core-file: use only these
 * functions to write out all the necessary info.
 */
extern bool dump_emit(struct coredump_params *cprm, const void *addr, size_t size);
extern bool dump_skip(struct coredump_params *cprm, size_t size);
extern bool dump_align(struct coredump_params *cprm, int align);
#ifdef CONFIG_COREDUMP
extern void do_coredump(siginfo_t *siginfo);
#else
static inline void do_coredump(siginfo_t *siginfo) {}
#endif

#endif /* _LINUX_COREDUMP_H */
