/*
 * fspin.c - user utility to burn CPU cycles, thrash the cache and memory
 *
 * Copyright (c) 2013, Intel Corporation.
 * Len Brown <len.brown@intel.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 */

/*
 * Creates one thread per logical processor (override with -t).
 * Threads run on any processor (override with -b).
 * Each thread allocates and initializes its own data.
 * Then it processes the data using an infinite DAXPY loop:
 * Double precision Y[i] = A*X[i] + Y[i]
 *
 * The parent thread wakes up every reporting interval,
 * (override 5 sec default with -i),
 * sums up and prints aggregate performance.
 *
 * The actual computation is somewhat arbitrary, if not random.
 * The performance number is intended only to be compared to itself
 * on the same machine, to illustrate how various power limiting
 * techniques impact performance.
 */
#define _GNU_SOURCE
#include <pthread.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <ctype.h>
#include <time.h>
#include <sched.h>
#include <errno.h>
#include <sys/time.h>

#define BANNER "fspin v1.1, April 7, 2013 - Len Brown <len.brown@intel.com>"

#define handle_error_en(en, msg) \
	do { errno = en; perror(msg); exit(EXIT_FAILURE); } while (0)

#define handle_error(msg) \
	do { perror(msg); exit(EXIT_FAILURE); } while (0)

struct thread_info {		/* Used as argument to spin_loop() */
	pthread_t thread_id;	/* ID returned by pthread_create() */
	int thread_num;		/* Application-defined thread # */
};

struct padded {
	double counter;	/* 8 bytes */
	double pad[(32 - 1)];	/* round up to 256 byte line */
} *thread_data;

int num_threads;
int thread_num_override;
int data_bytes = 512;
int nrcpus = 64;
int sec_per_interval = 5;	/* seconds */
int iterations;
int verbose;
int do_binding;

cpu_set_t *cpu_affinity_set;
size_t cpu_affinity_setsize;

void
allocate_cpusets()
{
	/*
	 * Allocate and initialize cpu_affinity_set
	 */
	cpu_affinity_set = CPU_ALLOC(nrcpus);
	if (cpu_affinity_set == NULL) {
		perror("CPU_ALLOC");
		exit(3);
	}
	cpu_affinity_setsize = CPU_ALLOC_SIZE(nrcpus);
	CPU_ZERO_S(cpu_affinity_setsize, cpu_affinity_set);
}

void
bind_to_cpus()
{
	if (!do_binding)
		return;

	if (sched_setaffinity(0, cpu_affinity_setsize, cpu_affinity_set) == -1) {
		fprintf(stderr, "bind_to_cpus() failed\n");
		perror("sched_setaffinity");
		exit(-1);
	}
}

int get_num_cpus()
{
	cpu_set_t *mask;
	size_t size;
	int num_cpus;

realloc:
	mask = CPU_ALLOC(nrcpus);
	size = CPU_ALLOC_SIZE(nrcpus);
	CPU_ZERO_S(size, mask);
	if (sched_getaffinity(0, size, mask) == -1) {
		CPU_FREE(mask);
		if (errno == EINVAL &&
			nrcpus < (1024 << 8)) {
			nrcpus = nrcpus << 2;
			goto realloc;
		}
		perror("sched_getaffinity");
		return -1;
	}

	num_cpus = CPU_COUNT_S(size, mask);

	CPU_FREE(mask);

	return num_cpus;
}

static void *spin_loop(void *arg)
{
	struct thread_info *tinfo = (struct thread_info *)arg;
	double *x, *y;
	int i = 0;
	int data_entries = data_bytes / sizeof(double);
	unsigned long long bitmask = random();

		
	x = malloc(data_bytes);
	y = malloc(data_bytes);

	if (x == NULL || y == NULL) {
		perror("malloc");
		exit(-1);
	}

	/*
	 * seed data array with random bits
	 */
	for (i = 0; i < data_entries; ++i) {
		x[i] = 1.0 + i * bitmask;
		y[i] = 1.0 + i * bitmask;
	}

	for (i = 0; ; i++) {

		double a = 3.1415926535 * i;

		y[i] = a * x[i] + y[i];		/* DAXPY */

		thread_data[tinfo->thread_num].counter++;

		if (i >= data_entries)
			i = 0;
	}
	/* not reached */
}

void usage()
{
	fprintf(stderr,
		"Usage: fspin [-v][-s sec_per_iteration][-i iterations][-t num_threads][-b cpu_list][-m memory(b|k|m)]\n");
	fprintf(stderr, "\twhere 'cpu_list' is comma and dash separated numbers with no spaces\n");
	exit(EXIT_FAILURE);
}

void parse_error(char *string, char c)
{
	fprintf(stderr, "parse error on '%s' at '%c'\n", string, c);
	usage();
}

int add_cpu_to_bind_mask(int cpu) {
	static int num_added;

	/* check if cpu is valid */
	if (cpu < 0 || cpu > nrcpus) {
		fprintf(stderr, "invalid cpu %d\n", cpu);
		exit(1);
	}

	if (CPU_ISSET_S(cpu, cpu_affinity_setsize, cpu_affinity_set)) {
		fprintf(stderr, "can't bind to cpu %d more than once\n", cpu);
		exit(1);
	}

	/* add cpu to set */
	CPU_SET_S(cpu, cpu_affinity_setsize, cpu_affinity_set);

	if (verbose)
		printf("%d, ", cpu);

	num_added += 1;

	return num_added;
}


int
parse_bind_cpu_list(char *cpu_list)
{
	char *p;
	int range_next = -1;
	int total_cpus_added = 0;

	allocate_cpusets();

	for(p = cpu_list; *p != '\0'; ) {
		int num, retval;

		/* remaining list must start w/ valid cpu number */

		if (!isdigit(*p))
			parse_error(p, *p);

		retval = sscanf(p, "%u", &num);
		if (retval == EOF)
			usage();
		else if (retval == 0)
			parse_error(p, *p);

		if (range_next >= 0) {
			if (num <= range_next)	/* range must be low to high */
				parse_error(p, *p);

			for ( ; range_next < num; range_next++)
				total_cpus_added = add_cpu_to_bind_mask(range_next);

			range_next = -1;
		}

		total_cpus_added = add_cpu_to_bind_mask(num);

		while (isdigit(*p))
			p++;

		switch (*p) {
		case ',':
			p++;
			continue;
		case '-':
			range_next = num + 1;
			p++;
			continue;
		}

	}
	return total_cpus_added;
}

int parse_memory_param(char *p)
{
	int bytes;
	char units;

	if (2 != sscanf(p, "%d%c", &bytes, &units)) {
		fprintf(stderr, "failed to parse -m\n");
		usage();
	}
	switch (units) {
	case 'b':
	case 'B':
		break;
	case 'k':
	case 'K':
		bytes *= 1024;
		break;
	case 'm':
	case 'M':
		bytes *= 1024*1024;
		break;
	case 'g':
	case 'G':
		bytes *= 1024*1024*1024;
		break;
	default:
		fprintf(stderr, "-m: bad memory units, use b, k, m, g\n");
	
	}
	return bytes;
	
}

void parse_args(int argc, char *argv[])
{
	int opt;

	nrcpus = get_num_cpus();

	while ((opt = getopt(argc, argv, "s:i:t:b:m:v")) != -1) {
		switch (opt) {
		case 's':
			sec_per_interval = atoi(optarg);
			if (verbose)
				printf("sec_per_interval %d\n", sec_per_interval);
			break;
		case 'i':
			iterations = atoi(optarg);
			if (verbose)
				printf("iterations %d\n", iterations);
			break;
		case 't':
			thread_num_override = atoi(optarg);
			if (verbose)
				printf("Thread Count Override: %d\n", thread_num_override);
			break;
		case 'b':
			do_binding = parse_bind_cpu_list(optarg);
			if (verbose)
				printf("Binding to %d CPUs.\n", do_binding);
			break;
		case 'm':
			data_bytes = parse_memory_param(optarg);
			if (verbose)
				printf("Memory Override: %d\n", data_bytes);
			break;
		case 'v':
			verbose++;
			break;
		default:	/* '?' */
			usage();	/* does not return */
		}
	}
}

unsigned long long lsum_old;


struct thread_info *tinfo;
pthread_attr_t attr;

void create_threads()
{
	int s, tnum;

	if (thread_num_override)
		num_threads = thread_num_override;
	else if (do_binding)
		num_threads = do_binding;
	else
		num_threads = nrcpus;

	thread_data = calloc(num_threads, sizeof(struct padded));
	if (thread_data == NULL)
		handle_error("calloc");

	/* Initialize thread creation attributes */

	s = pthread_attr_init(&attr);
	if (s != 0)
		handle_error_en(s, "pthread_attr_init");

	/* Allocate memory for pthread_create() arguments */

	tinfo = calloc(num_threads, sizeof(struct thread_info));
	if (tinfo == NULL)
		handle_error("calloc");

	for (tnum = 0; tnum < num_threads; tnum++) {
		tinfo[tnum].thread_num = tnum;

		/* The pthread_create() call stores the thread ID into
		 * corresponding element of tinfo[]
		 */

		s = pthread_create(&tinfo[tnum].thread_id, &attr,
				   &spin_loop, &tinfo[tnum]);
		if (s != 0)
			handle_error_en(s, "pthread_create");
	}
	printf("%d threads created\n", num_threads);
	return;
}


void monitor_threads()
{
	struct timespec ts;
	struct timeval tv_old, tv_new, tv_delta;
	int i, j;
	double interval_float;
	unsigned long long lsum;

	ts.tv_sec = sec_per_interval;
	ts.tv_nsec = 0;
	gettimeofday(&tv_old, (struct timezone *)NULL);

	for (i = 0; iterations ? i < iterations : 1 ; i++) {

		if (nanosleep(&ts, NULL) != 0) {
			perror("nanosleep");
			exit(-1);
		}

		for (j = 0, lsum = 0; j < num_threads; ++j)
			lsum += thread_data[j].counter;

		gettimeofday(&tv_new, NULL);
		timersub(&tv_new, &tv_old, &tv_delta);

		interval_float = tv_delta.tv_sec + tv_delta.tv_usec/1000000.0;
		printf("%.2f\n", (lsum - lsum_old)/interval_float/1000000);

		tv_old = tv_new;
		lsum_old = lsum;
	}
	/* summary */
	for (j = 0, lsum = 0; j < num_threads; ++j) {
		printf("%d %.2f\n", j, thread_data[j].counter/1000000.0);
		lsum += thread_data[j].counter;
	}
	printf("Total %.2f\n", lsum/1000000.0);
	
}


void print_banner()
{
	puts(BANNER);
}

int main(int argc, char *argv[])
{
	parse_args(argc, argv);


	print_banner();

	bind_to_cpus();

	create_threads();

	monitor_threads();	/* never returns */

	return 0;
}
