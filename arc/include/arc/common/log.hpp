/**
 * @file log.hpp
 * @author Andrea Marcer (marcera.andrea@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-01-14
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

/*****************************************************************************\
|                                   INCLUDES                                  |
\*****************************************************************************/
#include <string.h>
#include <iostream>

#include "pico/stdlib.h"

#include "common/common.hpp"

/*****************************************************************************\
|                                    MACROS                                   |
\*****************************************************************************/
#define LOG_ERROR 1
#define LOG_CRIT 1
#define LOG_WARNING 2
#define LOG_INFO 3

#ifdef LOG_COLOR
#define LOG_NONE "\e[0m"
#define LOG_PURPLE "\e[1;35m"
#define LOG_RED "\e[1;31m"
#define LOG_YELLOW "\e[1;33m"
#define LOG_BOLD "\e[1m"
#else
#define LOG_NONE
#define LOG_PURPLE
#define LOG_RED
#define LOG_YELLOW
#define LOG_BOLD
#endif

#define FILENAME(x) strstr(x, "/arc") + 5

#if defined(LOG_DEBUG) && !defined(NDEBUG)
#define log_debug \
    if (true) std::cout << LOG_PURPLE "[DEBUG] " LOG_NONE
#define log_debug_s \
    if (true) std::cout
#else
#define log_debug \
    if (false) std::cout
#define log_debug_s \
    if (false) std::cout
#endif

#if LOG_LEVEL >= LOG_ERROR
#define log_error                                                   \
    if (true)                                                       \
    std::cerr << LOG_RED "[ERROR] {./" << FILENAME(__FILE__) << ":" \
              << __LINE__ << "} " << __func__ << "(): " LOG_NONE
#else
#define log_error \
    if (false) std::cerr
#endif

#if LOG_LEVEL >= LOG_CRIT
#define log_critical \
    if (true) std::cout << LOG_RED "[CRIT ] " LOG_NONE
#define log_critical_s \
    if (true) std::cout
#else
#define log_critical \
    if (false) std::cout
#define log_critical_s \
    if (false) std::cout
#endif

#if LOG_LEVEL >= LOG_WARNING
#define log_warning \
    if (true) std::cout << LOG_YELLOW "[WARN ] " LOG_NONE
#define log_warning_s \
    if (true) std::cout
#else
#define log_warning \
    if (false) std::cout
#define log_warning_s \
    if (false) std::cout
#endif

#if LOG_LEVEL >= LOG_INFO
#define log_info \
    if (true) std::cout << LOG_BOLD "[INFO ] " LOG_NONE
#define log_info_s \
    if (true) std::cout
#else
#define log_info \
    if (false) std::cout
#define log_info_s \
    if (false) std::cout
#endif