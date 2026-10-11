# SPDX-License-Identifier: GPL-2.0-or-later
# sdr-slc-vocoderd — example / reference code. Copyright (C) 2026 Ivo van Ling (PA2IX).
# GPL-2.0-or-later; see LICENSE.
#
# Needs: json-c, mbelib-neo (>= 2.4, via pkg-config), and optionally
# avahi-client for mDNS. If mbelib-neo is installed under a custom prefix:
#   make PKG_CONFIG_PATH=/opt/mbe-neo/lib/pkgconfig
# To build against a vendored mbelib-neo checkout instead, see `make mbelib`.

CC      ?= gcc
PREFIX  ?= /usr/local
PKG_CONFIG ?= pkg-config
CFLAGS  ?= -O2 -g -Wall -Wextra -Wpedantic -std=c11
CPPFLAGS += -Iinclude -Isrc

MBE_VENDOR := third_party/mbelib-neo
MBE_LOCAL  := $(CURDIR)/build/mbelib-neo
# A vendored build (`make mbelib`) takes precedence. The search path is put on
# the pkg-config command line itself: GNU make before 4.4 does not pass
# exported variables into $(shell ...).
MBE_LOCAL_PC := $(firstword $(wildcard $(MBE_LOCAL)/lib*/pkgconfig))
ifneq ($(MBE_LOCAL_PC),)
PKG_CONFIG := PKG_CONFIG_PATH=$(MBE_LOCAL_PC):$$PKG_CONFIG_PATH $(PKG_CONFIG)
endif

MBE_OK := $(shell $(PKG_CONFIG) --exists 'libmbe-neo >= 2.4' && echo yes)
ifneq ($(MBE_OK),yes)
ifeq ($(filter clean mbelib,$(MAKECMDGOALS)),)
$(warning mbelib-neo >= 2.4 not found by pkg-config. Install it (see README), or run `make mbelib` to build the vendored copy.)
endif
endif
MBE_CFLAGS := $(shell $(PKG_CONFIG) --cflags libmbe-neo 2>/dev/null)
MBE_LIBDIR := $(shell $(PKG_CONFIG) --variable=libdir libmbe-neo 2>/dev/null)
# Link mbelib-neo statically when the static library is there, so the daemon
# does not depend on a library path at run time.
ifneq ($(wildcard $(MBE_LIBDIR)/libmbe-neo.a),)
MBE_LIBS := $(MBE_LIBDIR)/libmbe-neo.a -lm
else
MBE_LIBS := $(shell $(PKG_CONFIG) --libs libmbe-neo 2>/dev/null) -Wl,-rpath,$(MBE_LIBDIR) -lm
endif

CPPFLAGS += $(MBE_CFLAGS) $(shell $(PKG_CONFIG) --cflags json-c)
LDLIBS   += $(MBE_LIBS) $(shell $(PKG_CONFIG) --libs json-c) -lpthread

AVAHI_OK := $(shell $(PKG_CONFIG) --exists avahi-client && echo yes)
ifeq ($(AVAHI_OK),yes)
CPPFLAGS += -DHAVE_AVAHI $(shell $(PKG_CONFIG) --cflags avahi-client)
LDLIBS   += $(shell $(PKG_CONFIG) --libs avahi-client)
else
$(warning libavahi-client-dev not found: daemon will build without mDNS advertisement)
endif

CORE = src/vocoder.c src/swvoc.c src/frames.c src/ambe3k.c
SRC  = src/main.c src/rtp.c src/mdns.c $(CORE)
OBJ  = $(SRC:.c=.o)
TESTS = tests/test_rtp tests/test_frames tests/test_swvoc tests/test_arbiter

all: sdr-slc-vocoderd

sdr-slc-vocoderd: $(OBJ)
	$(CC) -o $@ $(OBJ) $(LDLIBS)

%.o: %.c
	$(CC) $(CPPFLAGS) $(CFLAGS) -c $< -o $@

$(OBJ): $(wildcard src/*.h include/*.h)

# ---- tests --------------------------------------------------------------
tests/test_rtp: tests/test_rtp.c src/rtp.c
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ $^
tests/test_frames: tests/test_frames.c src/frames.c tests/ref/dmr_prio.c tests/ref/codecd_dmr_air.c
	$(CC) $(CPPFLAGS) -Itests $(CFLAGS) -o $@ $^ $(MBE_LIBS)
tests/test_swvoc: tests/test_swvoc.c src/swvoc.c src/frames.c
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ $^ $(MBE_LIBS)
tests/test_arbiter: tests/test_arbiter.c $(CORE)
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ $^ $(MBE_LIBS) -lpthread

test check: $(TESTS)
	@for t in $(TESTS); do echo "== $$t"; ./$$t || exit 1; done
	@echo "all tests passed"

# The arbiter under ThreadSanitizer (needs a TSan-capable compiler).
tsan:
	$(CC) $(CPPFLAGS) -O1 -g -fsanitize=thread -std=c11 -o tests/test_arbiter_tsan tests/test_arbiter.c $(CORE) $(MBE_LIBS) -lpthread
	./tests/test_arbiter_tsan

# ---- bench tool ---------------------------------------------------------
ambe3k_loopback: tools/ambe3k_loopback.c src/ambe3k.c src/frames.c
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ $^ $(MBE_LIBS)

# Simulated AMBE-3000 on a pty (test the daemon without a DVstick).
ambe3k_sim: tools/ambe3k_sim.c src/frames.c src/swvoc.c
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ $^ $(MBE_LIBS)

# End-to-end: the real daemon + driver against ambe3k_sim, over SLC-CP/RTP.
tests/gen_frames: tests/gen_frames.c src/frames.c src/swvoc.c
	$(CC) $(CPPFLAGS) $(CFLAGS) -o $@ $^ $(MBE_LIBS)
e2e: sdr-slc-vocoderd ambe3k_sim tests/gen_frames
	python3 tests/e2e.py

# ---- vendored mbelib-neo (git submodule) --------------------------------
# For a one-clone build (e.g. on a Raspberry Pi without mbelib-neo packaged):
#   git submodule update --init && make mbelib && make
mbelib:
	@test -f $(MBE_VENDOR)/CMakeLists.txt || { echo "run: git submodule update --init"; exit 1; }
	cmake -S $(MBE_VENDOR) -B build/mbelib-neo-build -DCMAKE_BUILD_TYPE=Release \
	      -DMBELIB_BUILD_TESTS=OFF -DMBELIB_BUILD_EXAMPLES=OFF -DMBELIB_WARNINGS_AS_ERRORS=OFF \
	      -DCMAKE_INSTALL_PREFIX=$(MBE_LOCAL)
	cmake --build build/mbelib-neo-build -j
	cmake --install build/mbelib-neo-build

install: sdr-slc-vocoderd
	install -D -m755 sdr-slc-vocoderd $(DESTDIR)$(PREFIX)/sbin/sdr-slc-vocoderd
	install -D -m644 packaging/sdr-slc-vocoderd.service $(DESTDIR)/etc/systemd/system/sdr-slc-vocoderd.service

clean:
	rm -f $(OBJ) sdr-slc-vocoderd ambe3k_loopback ambe3k_sim tests/gen_frames $(TESTS) tests/test_arbiter_tsan

.PHONY: all test check tsan e2e mbelib install clean
