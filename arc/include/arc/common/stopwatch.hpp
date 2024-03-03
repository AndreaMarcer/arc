/**
 * @file stopwatch.hpp
 * @author Andrea Marcer (marcera.andrea@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-02-03
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

/*****************************************************************************\
|                                   INCLUDES                                  |
\*****************************************************************************/
#include "pico/stdlib.h"

#include "common/log.hpp"

namespace arc::common {

/*****************************************************************************\
|                                  STOPWATCH                                  |
\*****************************************************************************/
class Stopwatch {
public:
    inline void start() {
        running = true;
        start_time = time_us_64();
    }

    inline Stopwatch &stop() {
        uint64_t end = time_us_64();
        if (running) duration += end - start_time;
        running = false;
        return *this;
    }

    inline Stopwatch &reset() {
        duration = 0;
        return *this;
    }

    Stopwatch &print() {
        if (!running) {
            log_info << "TIMER: " << duration << " us\n";
        } else {
            log_info << "TIMER is still running\n";
        }
        return *this;
    }

private:
    bool running{false};
    uint64_t start_time;
    uint64_t duration{0};
};

}  // namespace arc::common