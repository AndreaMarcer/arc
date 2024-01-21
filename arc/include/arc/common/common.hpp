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

#define BYTE2STR(x) std::bitset<8>(x).to_string().c_str()

#define MULTILINE_DEFINE_BEGINE do {
#define MULTILINE_DEFINE_END \
	}                    \
	while (0)

namespace arc {
namespace common {

constexpr float DEG2RAD = M_PI / 180.0f;
constexpr float RAD2DEG = 180.0f / M_PI;

typedef union {
	float vec[3];
	struct {
		float x;
		float y;
		float z;
	};
} acc_t;

constexpr float modf(float vec[3])
{
	return hypot(hypotf(vec[0], vec[1]), vec[2]);
}

} // arc
} // common