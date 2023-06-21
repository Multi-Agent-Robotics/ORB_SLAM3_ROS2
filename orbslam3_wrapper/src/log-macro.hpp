#pragma once

#include <cstdarg>
#include <cstdio>
#include <unistd.h>

namespace escape_codes {
constexpr const char *reset = "\033[0m";
constexpr const char *red = "\033[31m";
constexpr const char *green = "\033[32m";
constexpr const char *yellow = "\033[33m";
constexpr const char *blue = "\033[34m";
constexpr const char *magenta = "\033[35m";
constexpr const char *cyan = "\033[36m";
} // namespace escape_codes

// #ifndef NDEBUG

// #ifdef DEBUG_LOG
// #warning "DEBUG_LOG is already defined."
// #endif

#define DEBUG_LOG(out, format, ...)                                            \
    /* The do {...} while(0) structure is a common idiom used in macros. */    \
    /* It allows the macro to be used in all contexts that a normal function   \
     * call could be used. */                                                  \
    /* It creates a compound statement in C/C++ that behaves as a single       \
     * statement. */                                                           \
    do {                                                                       \
        bool is_tty = isatty(fileno(out));                                     \
        fprintf(out, "%s%s%s:%s%s%s:%s%4d%s: ", escape_codes::cyan, __FILE__,  \
                escape_codes::reset, escape_codes::yellow, __func__,           \
                escape_codes::reset, escape_codes::green, __LINE__,            \
                escape_codes::reset);                                          \
        fprintf(out, format, ##__VA_ARGS__);                                   \
        fprintf(out, "\n");                                                    \
    } while (0)
// #else
// // do nothing
// #define DEBUG_LOG(out, format, ...)

// #endif
