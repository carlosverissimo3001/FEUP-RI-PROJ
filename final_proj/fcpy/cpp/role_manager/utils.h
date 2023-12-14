#pragma once
#include <algorithm>
#include "utils.h"
using std::min;
using std::max;

extern float distance_pt_segment_or_ray(float p_x, float p_y, float start_x, float start_y, float end_x, float end_y, bool is_ray=false);
extern float sq_distance_pt_segment_or_ray(float p_x, float p_y, float start_x, float start_y, float end_x, float end_y, bool is_ray=false);
extern bool check_if_noncollinear_segments_intersect(float a1_x, float a1_y, float a2_x, float a2_y, float b1_x, float b1_y, float b2_x, float b2_y);
extern void rotate_2d_vec(float &x, float &y, float rad);
extern float vectors_angle_rad(float v1_x, float v1_y, const bool v1_is_unit, float v2_x, float v2_y, const bool v2_is_unit);

inline float clip(float n, float lower, float upper) {
  return max(lower, min(n, upper));
}