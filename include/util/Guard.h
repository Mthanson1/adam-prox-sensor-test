/*
 * Early-return guard macros.
 *
 * Do not use inside an ISR
 */

#pragma once

#include <stdio.h>

#include "Console.h"

/*
 * Fault hook, defined by the application. Puts outputs in a safe state, reports
 * the call site and does not return.
 *
 * Not named Error_Handler: STM32duino defines Error_Handler() as a macro (a bare
 * spin loop, since its PlatformIO build always sets NDEBUG), so that name cannot
 * be declared or defined here.
 */
[[noreturn]] void faultHandler(const char* file, int line);

#ifdef DEBUG

/*
 * Formatted messages longer than this are truncated.
 */
#define GUARD_MSG_BUF_SIZE 128

/*
 * printf-style log line prefixed with the call site. The Arduino Print class has
 * no printf, so format into a stack buffer first.
 */
#define GUARD_LOG(...)                                     \
  do {                                                     \
    char guard_buf_[GUARD_MSG_BUF_SIZE];                   \
    snprintf(guard_buf_, sizeof(guard_buf_), __VA_ARGS__); \
    CONSOLE.print("[GUARD] ");                             \
    CONSOLE.print(__FILE__);                               \
    CONSOLE.print(":");                                    \
    CONSOLE.print(__LINE__);                               \
    CONSOLE.print(": ");                                   \
    CONSOLE.println(guard_buf_);                           \
  } while (0)

#else

#define GUARD_LOG(...) \
  do {                 \
  } while (0)

#endif

/**
 * @brief Return `returnVal` if `expr` is false; logs the failed expression in DEBUG.
 */
#define Guard(expr, returnVal)              \
  do {                                      \
    if (!(expr)) {                          \
      GUARD_LOG("check failed: %s", #expr); \
      return returnVal;                     \
    }                                       \
  } while (0)

/**
 * @brief Return from a void function if `expr` is false; logs the failed expression in DEBUG.
 */
#define voidGuard(expr)                     \
  do {                                      \
    if (!(expr)) {                          \
      GUARD_LOG("check failed: %s", #expr); \
      return;                               \
    }                                       \
  } while (0)

/**
 * @brief Return `returnVal` if `expr` is false; logs a printf-style message in DEBUG.
 *
 * Usage: GuardMsg(expr, returnVal, fmt, args...)
 */
#define GuardMsg(expr, returnVal, ...) \
  do {                                 \
    if (!(expr)) {                     \
      GUARD_LOG(__VA_ARGS__);          \
      return returnVal;                \
    }                                  \
  } while (0)

/**
 * @brief Call faultHandler() if `expr` is false; logs the failed expression in DEBUG.
 *
 * For failures the program cannot continue past, e.g. hardware init in setup().
 */
#define GuardFatal(expr)                \
  do {                                  \
    if (!(expr)) {                      \
      GUARD_LOG("fatal: %s", #expr);    \
      faultHandler(__FILE__, __LINE__); \
    }                                   \
  } while (0)
