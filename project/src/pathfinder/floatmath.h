#ifndef FLOATMATH_H_
#define FLOATMATH_H_

#define SCALE 16

typedef int32_t fixed_point;

float fixed_to_float(fixed_point in);
fixed_point float_to_fixed(float in);

int32_t fixed_to_int32(fixed_point in);
fixed_point int32_to_fixed(int32_t in);

fixed_point fix_div(fixed_point a, fixed_point b);
fixed_point fix_mult(fixed_point a, fixed_point b);

#endif
