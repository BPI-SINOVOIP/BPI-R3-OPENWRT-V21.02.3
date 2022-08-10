#
# Copyright (C) 2022 MediaTek Inc. All rights reserved.
#
# This is free software, licensed under the GNU General Public License v2.
# See /LICENSE for more information.
#

include libfdt/Makefile.libfdt

OBJS := main.o $(LIBFDT_SRCS:%.c=libfdt/%.o)
DEPS := $(OBJS:%.o=%.d)

CC ?= gcc
CFLAGS ?= -O2 -ffunction-sections
LDFLAGS ?= -Wl,--gc-sections
OPTFLAGS ?= -ggdb

all: fdt-patch-dm-verify

fdt-patch-dm-verify: $(OBJS)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $(OBJS) $(LIBS)

$(OBJS): %.o: %.c
	$(CC) $(CFLAGS) $(OPTFLAGS) -Ilibfdt -MD -c -o $@ $<

clean: libfdt_clean
	rm -f fdt-patch-dm-verify $(OBJS) $(DEPS)

.PHONY: clean

-include $(DEPS)
