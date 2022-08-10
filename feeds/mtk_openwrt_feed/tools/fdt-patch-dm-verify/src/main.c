// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2022 MediaTek Inc. All Rights Reserved.
 *
 * Author: Weijie Gao <weijie.gao@mediatek.com>
 *
 * Tool for modifying bootargs in dtb for dm-verity
 */

#ifdef _MSC_VER
#define _CRT_SECURE_NO_WARNINGS
#define _CRT_NONSTDC_NO_WARNINGS
#endif /* _MSC_VER */

#define _GNU_SOURCE

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdarg.h>
#include <malloc.h>
#include <ctype.h>
#include <errno.h>
#include <libfdt.h>

#define ARRAY_SIZE(a)		(sizeof(a) / sizeof(a[0]))

struct kvpair {
	char *name;
	char *value;
	bool quoted_value;
	bool deleted;
};

static void *fdt;
static char *veritysummary;
static uint32_t fdt_len, summary_len;

static struct kvpair *summary_lines;
static uint32_t summary_line_count;

static struct kvpair *bootargs_items;
static uint32_t bootargs_item_count;

#ifdef _MSC_VER
int vasprintf(char **strp, const char *format, va_list ap)
{
	int len = _vscprintf(format, ap);
	if (len == -1)
		return -1;
	char *str = (char *)malloc((size_t)len + 1);
	if (!str)
		return -1;
	int retval = vsnprintf(str, len + 1, format, ap);
	if (retval == -1) {
		free(str);
		return -1;
	}
	*strp = str;
	return retval;
}

int asprintf(char **strp, const char *format, ...)
{
	va_list ap;
	va_start(ap, format);
	int retval = vasprintf(strp, format, ap);
	va_end(ap);
	return retval;
}

static char *strndup(const char *str, size_t n)
{
	size_t len = strlen(str);
	char *s;

	if (len < n)
		return strdup(str);

	s = malloc(n + 1);
	if (!s)
		return NULL;

	memcpy(s, str, n);
	s[n] = 0;

	return s;
}
#endif /* _MSC_VER */

static int read_file(const char *file, void **buffer, uint32_t *filelen)
{
	size_t rdlen;
	int ret = 0;
	uint8_t *ptr;
	FILE *f;
	long len;

	f = fopen(file, "rb");
	if (!f) {
		fprintf(stderr, "Failed to open file '%s', error %d\n", file, errno);
		return -errno;
	}

	ret = fseek(f, 0, SEEK_END);
	if (ret < 0) {
		ret = ferror(f);
		fprintf(stderr, "fseek() failed, error %d\n", ret);
		goto cleanup;
	}

	len = ftell(f);
	if (len < 0) {
		ret = ferror(f);
		fprintf(stderr, "ftell() failed, error %d\n", ret);
		goto cleanup;
	}

	ret = fseek(f, 0, SEEK_SET);
	if (ret < 0) {
		ret = ferror(f);
		fprintf(stderr, "fseek() failed, error %d\n", ret);
		goto cleanup;
	}

	ptr = malloc(len + 1);
	if (!ptr) {
		ret = ferror(f);
		fprintf(stderr, "Failed to allocate memory\n");
		goto cleanup;
	}

	rdlen = fread(ptr, 1, len, f);
	if (rdlen != len) {
		ret = ferror(f);
		fprintf(stderr, "Failed to read file, error %d\n", ret);
		free(ptr);
		*buffer = NULL;
		goto cleanup;
	}

	ptr[len] = 0;

	*buffer = (void *)ptr;

	if (filelen)
		*filelen = len;

cleanup:
	fclose(f);
	return ret;
}

static int write_file(const char *file, const void *buffer, uint32_t len)
{
	size_t wrlen;
	FILE *f;
	int ret;

	f = fopen(file, "wb");
	if (!f) {
		fprintf(stderr, "Failed to open file '%s', error %d\n", file, errno);
		return -errno;
	}

	wrlen = fwrite(buffer, 1, len, f);
	ret = ferror(f);
	fclose(f);

	if (wrlen != len) {
		fprintf(stderr, "Failed to write file, error %d\n", ret);
		return ret;
	}

	return 0;
}

static struct kvpair *kvpair_find(struct kvpair *pairs, uint32_t count, const char *name)
{
	uint32_t i;

	for (i = 0; i < count; i++) {
		if (!strcmp(pairs[i].name, name))
			return &pairs[i];
	}

	return NULL;
}

/* find the first line-ending char in a string */
static inline char *strcrlf(char *str)
{
	while (*str) {
		if (*str == '\r' || *str == '\n')
			return str;
		str++;
	}

	return NULL;
}

/* find the first NULL char in a string */
static inline char *strnull(char *str, size_t len)
{
	while (len) {
		if (!*str)
			return str;
		str++;
		len--;
	}

	return NULL;
}

/* trim all whitespace chars at the beginning of a string by moving pointer */
static inline char *ltrim(const char *str)
{
	while (*str == 0x20 || *str == '\t')
		str++;

	return (char *)str;
}

/* trim all whitespace chars at the end of a string. chars will be changed */
static inline void rtrim_len(char *str, size_t len)
{
	char *eos = str + len - 1;

	while (eos >= str) {
		if (*eos != 0x20 && *eos != '\t')
			break;

		*eos = 0;
		eos--;
	}
}

/* derived from libkvcutil */
static int parse_verity_summary(void)
{
	char *ptr = veritysummary, *pcrlf, *psep, *peol, *pkey;
	uint32_t lines = 1;

	/*
	 * first step: count total number of lines
	 * this make sure we only allocate memory once
	 */
	do {
		pcrlf = strcrlf(ptr);
		if (!pcrlf)
			break;

		/*
		 * rules of line splitting:
		 *     - CR + LF:  treat as one line (Windows/DOS)
		 *     - CR + ^LF: Mac
		 *     - LF:       Unix/Linux
		 */
		if (pcrlf[0] == '\r' && pcrlf[1] == '\n')
			ptr = pcrlf + 2;
		else
			ptr = pcrlf + 1;

		lines++;
	} while (1);

	/* allocate memory to store parsed lines */
	summary_lines = calloc(lines, sizeof(*summary_lines));
	if (!summary_lines) {
		fprintf(stderr, "Failed to allocate memory for summary parsing\n");
		return -ENOMEM;
	}

	/* second step: split lines and parse keys and values */
	ptr = veritysummary;

	do {
		peol = strcrlf(ptr);
		pcrlf = peol;

		/* split a line and record its line-ending */
		if (pcrlf) {
			if (pcrlf[0] == '\r' && pcrlf[1] == '\n') {
				pcrlf[0] = 0;
				pcrlf += 2;
			} else {
				pcrlf[0] = 0;
				pcrlf += 1;
			}
		} else {
			/* CR/LF not found. should be the last line */
			peol = strnull(ptr, summary_len - (ptr - veritysummary) + 1);
			pcrlf = peol + 1;
		}

		/* trim leading spaces of key */
		pkey = ltrim(ptr);

		psep = strchr(pkey, ':');

		if (!psep) {
			/* line has no ':' */
			goto next_line;
		}

		psep[0] = 0;

		/* trim trailing spaces of key */
		rtrim_len(pkey, psep - pkey);

		/* trim white spaces of value */
		psep = ltrim(psep + 1);
		rtrim_len(psep, peol - psep);

		summary_lines[summary_line_count].name = pkey;
		summary_lines[summary_line_count].value = psep;
		summary_line_count++;

	next_line:
		ptr = pcrlf;
		if (ptr >= veritysummary + summary_len)
			break;
	} while (1);

	return 0;
}

static const char *verity_summary_find(const char *name)
{
	struct kvpair *p = kvpair_find(summary_lines, summary_line_count, name);

	if (p)
		return p->value;

	return NULL;
}

/**
 * skip_spaces - Removes leading whitespace from @str.
 * @str: The string to be stripped.
 *
 * Returns a pointer to the first non-whitespace character in @str.
 */
char *skip_spaces(const char *str)
{
	while (isspace(*str))
		++str;
	return (char *)str;
}

/* derived from linux kernel */
static const char *get_arg_next(const char *args, const char **param, size_t *keylen)
{
	unsigned int i, equals = 0;
	int in_quote = 0;

	args = skip_spaces(args);
	if (!*args)
		return NULL;

	if (*args == '"') {
		args++;
		in_quote = 1;
	}

	for (i = 0; args[i]; i++) {
		if (isspace(args[i]) && !in_quote)
			break;

		if (equals == 0) {
			if (args[i] == '=')
				equals = i;
		}

		if (args[i] == '"')
			in_quote = !in_quote;
	}

	*param = args;

	if (equals)
		*keylen = equals;
	else
		*keylen = i;

	return args + i;
}

static int parse_bootargs(const char *bootargs)
{
	const char *n = bootargs, *p;
	size_t len, keylen;
	uint32_t i = 0;

	while (1) {
		n = get_arg_next(n, &p, &keylen);
		if (!n)
			break;

		bootargs_item_count++;
	}

	if (!bootargs_item_count)
		return 0;

	bootargs_items = calloc(bootargs_item_count, sizeof(*bootargs_items));
	if (!bootargs_items) {
		fprintf(stderr, "Failed to allocate memory for summary parsing\n");
		return -ENOMEM;
	}

	n = bootargs;

	while (1) {
		n = get_arg_next(n, &p, &keylen);
		if (!n)
			break;

		len = n - p;

		bootargs_items[i].name = strndup(p, keylen);
		if (!bootargs_items[i].name)
			return -ENOMEM;

		if (keylen < len) {
			if (p[keylen + 1] == '\"') {
				bootargs_items[i].value = strndup(p + keylen + 2, len - keylen - 3);
				bootargs_items[i].quoted_value = true;
			} else {
				bootargs_items[i].value = strndup(p + keylen + 1, len - keylen - 1);
			}

			if (!bootargs_items[i].value)
				return -ENOMEM;
		}

		i++;
	}

	return 0;
}

static const char *bootargs_find(const char *name)
{
	struct kvpair *p = kvpair_find(bootargs_items, bootargs_item_count, name);

	if (p)
		return p->value;

	return NULL;
}

static char *strconcat(char *dst, const char *src)
{
	while (*src)
		*dst++ = *src++;

	return dst;
}

static char *merge_bootargs(struct kvpair *new_pairs, uint32_t num)
{
	struct kvpair *kvp;
	uint32_t i, len = 0;
	char *val;
	char *str, *p;
	bool quoted;

	/* Estimate new booargs size */
	for (i = 0; i < bootargs_item_count; i++) {
		len += strlen(bootargs_items[i].name);
		if (bootargs_items[i].value)
			len += strlen(bootargs_items[i].value);
		len += 4; /* =, "", space */
	}

	for (i = 0; i < num; i++) {
		len += strlen(new_pairs[i].name);
		if (new_pairs[i].value)
			len += strlen(new_pairs[i].value);
		len += 4; /* =, "", space */
	}

	str = calloc(len + 1, 1);
	if (!str)
		return NULL;

	/* Merge */
	p = str;

	/* Existed or overwritten */
	for (i = 0; i < bootargs_item_count; i++) {
		kvp = kvpair_find(new_pairs, num, bootargs_items[i].name);
		if (kvp) {
			val = kvp->value;
			quoted = kvp->quoted_value;
			kvp->deleted = true;
		} else {
			val = bootargs_items[i].value;
			quoted = bootargs_items[i].quoted_value;
		}

		p = strconcat(p, bootargs_items[i].name);
		if (val) {
			*p++ = '=';

			if (quoted)
				*p++ = '\"';

			p = strconcat(p, val);

			if (quoted)
				*p++ = '\"';
		}

		*p++ = ' ';
	}

	/* New */
	for (i = 0; i < num; i++) {
		if (new_pairs[i].deleted)
			continue;

		p = strconcat(p, new_pairs[i].name);
		if (new_pairs[i].value) {
			*p++ = '=';

			if (new_pairs[i].quoted_value)
				*p++ = '\"';

			p = strconcat(p, new_pairs[i].value);

			if (new_pairs[i].quoted_value)
				*p++ = '\"';
		}

		*p++ = ' ';
	}

	p[-1] = 0;

	return str;
}

int main(int argc, char *argv[])
{
	const char *datablocks, *datablock_size, *hashblock_size, *hash_algo, *salt, *root_hash;
	const char *bootargs, *rootdev;
	int ret, nodeoffset, len;
	struct kvpair dmpairs[2];
	uint32_t datablocks_num;
	char *dmstr, *nfdt;
	size_t nlen;

	if (argc < 4) {
		printf("Usage: <summary> <fdt-in> <fdt-out> [override-root]\n");
		return 0;
	}

	ret = read_file(argv[1], (void **)&veritysummary, &summary_len);
	if (ret)
		return 1;

	ret = read_file(argv[2], &fdt, &fdt_len);
	if (ret)
		return 2;

	if (parse_verity_summary())
		return 3;

	/* find "/chosen" node. */
	nodeoffset = fdt_subnode_offset(fdt, 0, "chosen");
	if (nodeoffset < 0) {
		fprintf(stderr, "Node `chosen' not found\n");
		return 4;
	}

	bootargs = fdt_getprop(fdt, nodeoffset, "bootargs", &len);
	if (!bootargs) {
		fprintf(stderr, "Property `bootargs' not found\n");
		return 5;
	}

	parse_bootargs(bootargs);

	/* find rootdev */
	rootdev = bootargs_find("root");
	if (!rootdev) {
		if (argc == 4 || !strlen(argv[4])) {
			fprintf(stderr, "`root' not found in `bootargs`\n");
			return 6;
		}

		if (strchr(argv[4], ' ') || strchr(argv[4], '\t') || strcrlf(argv[4])) {
			fprintf(stderr, "Overrided `root' must not contain whitespace\n");
			return 6;
		}

		rootdev = argv[4];
	}

	/* No dm-mod.create is expected */
	if (bootargs_find("dm-mod.create")) {
		fprintf(stderr, "Found unexpected `dm-mod.create' in `bootargs'\n");
		return 7;
	}

	/* Assemble `dm-mod.create' */
	datablocks = verity_summary_find("Data blocks");
	datablock_size = verity_summary_find("Data block size");
	hashblock_size = verity_summary_find("Hash block size");
	hash_algo = verity_summary_find("Hash algorithm");
	salt = verity_summary_find("Salt");
	root_hash = verity_summary_find("Root hash");

	if (!datablocks || !datablock_size || !hashblock_size || !hash_algo || !salt || !root_hash) {
		fprintf(stderr, "Incomplete summary of veritysetup\n");
		return 8;
	}

	datablocks_num = strtoul(datablocks, NULL, 0);
	if (datablocks_num == ULONG_MAX) {
		fprintf(stderr, "Data blocks is invalid\n");
		return 9;
	}

	ret = asprintf(&dmstr, "dm-verity,,,ro,0 %u verity 1 %s %s %s %s %u %u %s %s %s",
		       datablocks_num * 8, rootdev, rootdev, datablock_size, hashblock_size,
		       datablocks_num, datablocks_num + 1, hash_algo, root_hash, salt);
	if (ret < 0) {
		fprintf(stderr, "Failed to format `dm-mod.create'\n");
		return 9;
	}

	/* Assemble new bootargs */
	memset(dmpairs, 0, sizeof(dmpairs));

	dmpairs[0].name = "root";
	dmpairs[0].value = "/dev/dm-0";

	dmpairs[1].name = "dm-mod.create";
	dmpairs[1].value = dmstr;
	dmpairs[1].quoted_value = true;

	bootargs = merge_bootargs(dmpairs, ARRAY_SIZE(dmpairs));
	if (!bootargs) {
		fprintf(stderr, "Failed to merge new bootargs\n");
		return 10;
	}

	/* Resize dtb buffer */
	nlen = strlen(bootargs) + 1;
	nfdt = realloc(fdt, fdt_len + nlen);
	if (!nfdt) {
		fprintf(stderr, "Failed to extend fdt buffer\n");
		return 11;
	}

	memset((uint8_t *)nfdt + fdt_len, 0, nlen);

	fdt = nfdt;

	/* Modify bootagrs in dtb */
	ret = fdt_open_into(fdt, fdt, fdt_totalsize(fdt) + nlen);
	if (ret) {
		fprintf(stderr, "Failed to extend fdt size\n");
		return 11;
	}

	ret = fdt_setprop(fdt, nodeoffset, "bootargs", bootargs, nlen);
	if (ret < 0) {
		fprintf(stderr, "Failed to set new `bootargs'\n");
		return 12;
	}

	/* Change the fdt header to reflect the correct size */
	fdt_set_totalsize(fdt, fdt_off_dt_strings(fdt) + fdt_size_dt_strings(fdt));

	/* Save fdt */
	ret = write_file(argv[3], fdt, fdt_totalsize(fdt));
	if (ret)
		return 13;

	return 0;
}
