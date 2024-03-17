/**
 * @file common.hpp
 * @author Andrea Marcer (marcera.andrea@gmail.com)
 * @brief
 * @version 0.1
 * @date 2024-01-13
 *
 * @copyright Copyright (c) 2024
 *
 */

#pragma once

/*****************************************************************************\
|                                   INCLUDES                                  |
\*****************************************************************************/
#include <bitset>
#include <iostream>
#include <iomanip>

/*****************************************************************************\
|                                    MACROS                                   |
\*****************************************************************************/
using namespace std;

#define BYTE2STR(x) bitset<8>(x).to_string()
#define HEX2STR(num, w) \
    uppercase << setfill('0') << setw(w) << hex << num << dec

#define MULTILINE_DEFINE_BEGIN do {
#define MULTILINE_DEFINE_END \
    }                        \
    while (0)

#define BIT(n) (1 << n)

/*****************************************************************************\
|                                    COMMON                                   |
\*****************************************************************************/
namespace arc::common {}  // namespace arc::common