#ifndef FAST_SQRT_H
#define FAST_SQRT_H
#include <stdint.h>

float fast_sqrt(float number) {
	int32_t i;
	float x2, y;
	const float threehalves = 1.5f;

	x2 = number * 0.5f;
	y = number;
	i = *(int32_t *)&y;
	i = 0x5f3759df - (i >> 1);
	y = *(float *)&i;
	y = y * (threehalves - (x2 * y * y));
	return number * y;
}

#endif
