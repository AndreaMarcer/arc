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

#include "math.h"
#include <bitset>

#define TO_BIT_STR(x) std::bitset<8>(x).to_string().c_str()

namespace arc {
namespace common {

inline float modf(float vec[3])
{
	return hypot(hypotf(vec[0], vec[1]), vec[2]);
}

} // arc
} // common