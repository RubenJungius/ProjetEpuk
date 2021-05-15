#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include "floatmath.h"

float fixed_to_float(fixed_point in){
	return ((float)in/(float)(1 << SCALE));
}

fixed_point float_to_fixed(float in){
	return (fixed_point)(in * (1 << SCALE));
}

int32_t fixed_to_int32(fixed_point in){
	return in >> SCALE;
}

fixed_point int32_to_fixed(int32_t in){
	return in << SCALE;
}

fixed_point fix_div(fixed_point a, fixed_point b){
	return ((int64_t)a * (1<<SCALE))/b;
}

fixed_point fix_mult(fixed_point a, fixed_point b){
	return ((int64_t)a * (int64_t)b)/(1<<SCALE);
}
