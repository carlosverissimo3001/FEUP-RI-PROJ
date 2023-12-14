#include <cmath>
#include "utils.h"

/**
 * @brief Distance (point to segment) OR (point to ray)
 * 
 * @param p_x      coordinate x of point p
 * @param p_y      coordinate y of point p
 * @param start_x  coordinate x of segment/ray start
 * @param start_y  coordinate y of segment/ray start
 * @param end_x    coordinate x of segment end OR ray second point
 * @param end_y    coordinate y of segment end OR ray second point
 * @param is_ray   true if is a ray, false if segment
 * @return float 
 */
float distance_pt_segment_or_ray(float p_x, float p_y, float start_x, float start_y, float end_x, float end_y, bool is_ray){

    float sp_x = p_x - start_x; 
    float sp_y = p_y - start_y; 
    float se_x = end_x - start_x;
    float se_y = end_y - start_y;

    float sc_proj_scale = (sp_x*se_x + sp_y*se_y) / (se_x*se_x + se_y*se_y + 1e-8);  // scale = projection length / target vector length
    float sc_proj_x = se_x * sc_proj_scale; // projection of start->center onto start->end
    float sc_proj_y = se_y * sc_proj_scale; // projection of start->center onto start->end

    // check if projection falls on top of trajectory (start->projection = k * start->end)
    float k = abs(se_x)>abs(se_y) ? sc_proj_x/se_x : sc_proj_y/se_y; // we use the largest dimension of start->end to avoid division by 0

    if(k <= 0){
        return sqrtf(sp_x*sp_x + sp_y*sp_y); // distance: center<->start
    }else if(k >= 1 and !is_ray){
        float ep_x = p_x - end_x;
        float ep_y = p_y - end_y;
        return sqrtf(ep_x*ep_x + ep_y*ep_y); // distance: center<->end
    }else{
        float proj_p_x = p_x - (sc_proj_x + start_x);
        float proj_p_y = p_y - (sc_proj_y + start_y);
        return sqrtf(proj_p_x*proj_p_x + proj_p_y*proj_p_y); // distance: center<->projection
    }

}

/**
 * @brief Squared distance (point to segment) OR (point to ray)
 * 
 * @param p_x      coordinate x of point p
 * @param p_y      coordinate y of point p
 * @param start_x  coordinate x of segment/ray start
 * @param start_y  coordinate y of segment/ray start
 * @param end_x    coordinate x of segment end OR ray second point
 * @param end_y    coordinate y of segment end OR ray second point
 * @param is_ray   true if is a ray, false if segment
 * @return float 
 */
float sq_distance_pt_segment_or_ray(float p_x, float p_y, float start_x, float start_y, float end_x, float end_y, bool is_ray){

    float sp_x = p_x - start_x; 
    float sp_y = p_y - start_y; 
    float se_x = end_x - start_x;
    float se_y = end_y - start_y;

    float sc_proj_scale = (sp_x*se_x + sp_y*se_y) / (se_x*se_x + se_y*se_y + 1e-8);  // scale = projection length / target vector length
    float sc_proj_x = se_x * sc_proj_scale; // projection of start->center onto start->end
    float sc_proj_y = se_y * sc_proj_scale; // projection of start->center onto start->end

    // check if projection falls on top of trajectory (start->projection = k * start->end)
    float k = abs(se_x)>abs(se_y) ? sc_proj_x/se_x : sc_proj_y/se_y; // we use the largest dimension of start->end to avoid division by 0

    if(k <= 0){
        return sp_x*sp_x + sp_y*sp_y; // distance: center<->start
    }else if(k >= 1 and !is_ray){
        float ep_x = p_x - end_x;
        float ep_y = p_y - end_y;
        return ep_x*ep_x + ep_y*ep_y; // distance: center<->end
    }else{
        float proj_p_x = p_x - (sc_proj_x + start_x);
        float proj_p_y = p_y - (sc_proj_y + start_y);
        return proj_p_x*proj_p_x + proj_p_y*proj_p_y; // distance: center<->projection
    }

}

bool check_if_noncollinear_segments_intersect(float a1_x, float a1_y, float a2_x, float a2_y, float b1_x, float b1_y, float b2_x, float b2_y){
    // Explanation: https://www.geeksforgeeks.org/check-if-two-given-line-segments-intersect/ 
    // Tested. When touching, a collision may or may not be detected, but that is not relevant.
    bool aux1 = (b2_y-a1_y) * (b1_x-a1_x) > (b1_y-a1_y) * (b2_x-a1_x);
    bool aux2 = (b2_y-a2_y) * (b1_x-a2_x) > (b1_y-a2_y) * (b2_x-a2_x);
    if (aux1 == aux2) return false;
    aux1 = (b1_y-a1_y) * (a2_x-a1_x) > (a2_y-a1_y) * (b1_x-a1_x);
    aux2 = (b2_y-a1_y) * (a2_x-a1_x) > (a2_y-a1_y) * (b2_x-a1_x);
    return aux1 != aux2;
}

void rotate_2d_vec(float &x, float &y, float rad){
    float cos_ang = cosf(rad);
    float sin_ang = sinf(rad);
    float aux = cos_ang*x-sin_ang*y;
    y = sin_ang*x+cos_ang*y;
    x = aux;
}

/**
 * @brief Compute angle in radians between 2 2D vectors
 * 
 * @param v1_x        x component of first vector
 * @param v1_y        y component of first vector
 * @param v1_is_unit  true if v1 has a magnitude of 1 
 * @param v2_x        x component of second vector
 * @param v2_y        y component of second vector
 * @param v2_is_unit  true if v2 has a magnitude of 1 
 * @return float      angle in radians (0 to pi)
 */
float vectors_angle_rad(float v1_x, float v1_y, const bool v1_is_unit, float v2_x, float v2_y, const bool v2_is_unit){

    if((v1_x == 0 and v1_y == 0) or (v2_x == 0 and v2_y == 0)){
        return 0; // return angle of 0 if any vector has a magnitude of zero
    }

    if (!v1_is_unit){
        const float v1_d = sqrt(v1_x*v1_x + v1_y*v1_y);
        v1_x /= v1_d;
        v1_y /= v1_d;
    }

    if (!v2_is_unit){
        const float v2_d = sqrt(v2_x*v2_x + v2_y*v2_y);
        v2_x /= v2_d;
        v2_y /= v2_d;
    }

    // cos(a) = dot(v1_unit, v2_unit)
    return acosf(v1_x*v2_x + v1_y*v2_y); // 0 to pi

}