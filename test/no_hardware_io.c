/*
 * Copyright (C) 2026 SpacemiT (Hangzhou) Technology Co. Ltd.
 * SPDX-License-Identifier: Apache-2.0
 */
// Test-only LD_PRELOAD tripwire. Abort BEFORE a physical serial open occurs.
// Pseudo-terminals (/dev/pts/*) are allowed for protocol integration tests.
#define _GNU_SOURCE
#include <dlfcn.h>
#include <fcntl.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

static void check(const char* path) {
    if (path &&
        (strstr(path, "/dev/serial/") || strstr(path, "/dev/tty") || strstr(path, "/dev/go2/") ||
            strstr(path, "/dev/rpmsg") || strstr(path, "/dev/spidev") || strstr(path, "/dev/bmi270"))) {
        const char* message = "NO_HARDWARE_IO: forbidden serial open\n";
        const ssize_t written = write(STDERR_FILENO, message, strlen(message));
        (void)written;
        _exit(86);
    }
}
static int needs_mode(int flags) { return (flags & O_CREAT) || (flags & O_TMPFILE) == O_TMPFILE; }
#define OPEN_WRAPPER(name)                                                 \
    int name(const char* path, int flags, ...) {                           \
        check(path);                                                       \
        mode_t mode = 0;                                                   \
        if (needs_mode(flags)) {                                           \
            va_list a;                                                     \
            va_start(a, flags);                                            \
            mode = va_arg(a, int);                                         \
            va_end(a);                                                     \
        }                                                                  \
        int (*real_open)(const char*, int, ...) = dlsym(RTLD_NEXT, #name); \
        return real_open(path, flags, mode);                               \
    }
#define OPENAT_WRAPPER(name)                                                    \
    int name(int dir, const char* path, int flags, ...) {                       \
        check(path);                                                            \
        if (dir != AT_FDCWD && path && path[0] != '/') {                        \
            char link[64], resolved[1024];                                      \
            snprintf(link, sizeof(link), "/proc/self/fd/%d", dir);              \
            ssize_t n = readlink(link, resolved, sizeof(resolved) - 1);         \
            if (n > 0) {                                                        \
                resolved[n] = 0;                                                \
                check(resolved);                                                \
            }                                                                   \
        }                                                                       \
        mode_t mode = 0;                                                        \
        if (needs_mode(flags)) {                                                \
            va_list a;                                                          \
            va_start(a, flags);                                                 \
            mode = va_arg(a, int);                                              \
            va_end(a);                                                          \
        }                                                                       \
        int (*real_open)(int, const char*, int, ...) = dlsym(RTLD_NEXT, #name); \
        return real_open(dir, path, flags, mode);                               \
    }
OPEN_WRAPPER(open)
OPEN_WRAPPER(open64)
OPENAT_WRAPPER(openat)
OPENAT_WRAPPER(openat64)
int __open_2(const char* p, int f) { return open(p, f); }
int __open64_2(const char* p, int f) { return open64(p, f); }
int __openat_2(int d, const char* p, int f) { return openat(d, p, f); }
int __openat64_2(int d, const char* p, int f) { return openat64(d, p, f); }
