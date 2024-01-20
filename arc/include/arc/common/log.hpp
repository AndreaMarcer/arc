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

#define LOG_ERROR 1
#define LOG_WARNING 2
#define LOG_INFO 3

#define NONE "\e[0m"
#define PURPLE "\e[0;35m"
#define RED "\e[0;31m"
#define YELLOW "\e[1;33m"

#define BOLD "\e[1m"

#ifdef LOG_DEBUG
#define log_debug(M, ...)                                              \
	MULTILINE_DEFINE_BEGINE                                        \
	fprintf(stdout, BOLD PURPLE "[DEBUG] " NONE M, ##__VA_ARGS__); \
	MULTILINE_DEFINE_END
#else
#define log_debug(m, ...)
#endif

#if LOG_LEVEL >= LOG_ERROR
#define log_error(M, ...)                                           \
	MULTILINE_DEFINE_BEGINE                                     \
	fprintf(stdout, BOLD RED "[ERROR] " NONE M, ##__VA_ARGS__); \
	MULTILINE_DEFINE_END
#else
#define log_error(m, ...)
#endif

#if LOG_LEVEL >= LOG_WARNING
#define log_warning(M, ...)                                            \
	MULTILINE_DEFINE_BEGINE                                        \
	fprintf(stdout, BOLD YELLOW "[WARN ] " NONE M, ##__VA_ARGS__); \
	MULTILINE_DEFINE_END
#else
#define log_warning(m, ...)
#endif

#if LOG_LEVEL >= LOG_INFO
#define log_info(M, ...)                                        \
	MULTILINE_DEFINE_BEGINE                                 \
	fprintf(stdout, BOLD "[INFO ] " NONE M, ##__VA_ARGS__); \
	MULTILINE_DEFINE_END
#else
#define log_info(m, ...)
#endif

namespace arc {
namespace common {
} // arc
} // common