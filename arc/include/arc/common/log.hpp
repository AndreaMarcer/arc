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

#include "pico/stdlib.h"
#include <string.h>

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

#ifdef LOG_DEBUG
#define log_debug(M, ...)                                                 \
	MULTILINE_DEFINE_BEGINE                                           \
	fprintf(stdout, LOG_PURPLE "[DEBUG] " LOG_NONE M, ##__VA_ARGS__); \
	MULTILINE_DEFINE_END
#define log_debug_s(M, ...)                \
	MULTILINE_DEFINE_BEGINE            \
	fprintf(stdout, M, ##__VA_ARGS__); \
	MULTILINE_DEFINE_END
#else
#define log_debug(m, ...)
#endif

#if LOG_LEVEL >= LOG_ERROR
#define log_error(M, ...)                                               \
	MULTILINE_DEFINE_BEGINE                                         \
	fprintf(stdout, LOG_RED "[ERROR] {./%s:%d} %s(): " LOG_NONE M,  \
		FILENAME(__FILE__), __LINE__, __func__, ##__VA_ARGS__); \
	MULTILINE_DEFINE_END
#else
#define log_error(m, ...)
#endif

#if LOG_LEVEL >= LOG_CRIT
#define log_critical(M, ...)                                           \
	MULTILINE_DEFINE_BEGINE                                        \
	fprintf(stdout, LOG_RED "[CRIT ] " LOG_NONE M, ##__VA_ARGS__); \
	MULTILINE_DEFINE_END
#else
#define log_critical(m, ...)
#endif

#if LOG_LEVEL >= LOG_WARNING
#define log_warning(M, ...)                                               \
	MULTILINE_DEFINE_BEGINE                                           \
	fprintf(stdout, LOG_YELLOW "[WARN ] " LOG_NONE M, ##__VA_ARGS__); \
	MULTILINE_DEFINE_END
#else
#define log_warning(m, ...)
#endif

#if LOG_LEVEL >= LOG_INFO
#define log_info(M, ...)                                                \
	MULTILINE_DEFINE_BEGINE                                         \
	fprintf(stdout, LOG_BOLD "[INFO ] " LOG_NONE M, ##__VA_ARGS__); \
	MULTILINE_DEFINE_END
#else
#define log_info(m, ...)
#endif

namespace arc {
namespace common {
} // arc
} // common