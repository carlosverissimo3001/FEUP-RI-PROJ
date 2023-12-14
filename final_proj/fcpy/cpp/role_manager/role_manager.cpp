#include <cmath>
#include <algorithm>
#include <limits>
#include "role_manager.h"
#include "utils.h"

//------------ define 13 possible roles GK,CB,LS,RS,LF,RF
#define GK  0
#define CB  1
#define MM1 2
#define MM2 3
#define MM3 4
#define MM4 5
#define LS  6
#define RS  7
#define LF  8
#define RF  9
#define AP  10
#define AP2 11
#define AP3 12
//------------ 

using std::min;
using std::max;

#define MAX_DISTANCE 50.f


/**
 * @brief Get distances of team to static target
 */
void get_target_dist(float tx, float ty, float team[], float ret_dist[]){

    for(int i=0; i<22; i+=2){
        float x = team[i];

        if(x == 1000){ //--------- invalid positions get equally high distance
            ret_dist[i/2] = MAX_DISTANCE; 
            continue;
        }

        float dx = tx - x;
        float dy = ty - team[i+1];
        ret_dist[i/2] = min( sqrtf(dx*dx+dy*dy), MAX_DISTANCE );
    }

}



/**
 * @brief Get intersection with moving ball (intersection point and distance)
 * 
 * @param ball_pos_pred prediction of ball position including current position
 * @param prediction_array_size size of ball_pos_pred
 * @param team 2D position of 11 players (x,y)
 * @param ret_dist returned distances of each player to the point of intersection
 * @param ret_pos returned 2D positions of each point of intersection
 * @param max_robot_sp_per_step maximum speed per step
 */
void get_intersection_with_ball(float ball_pos_pred[], int prediction_array_size, float team[], float ret_dist[], float ret_pos[], float max_robot_sp_per_step, bool is_player_down[]=nullptr){
    /**
     * Future optimization:
     * If ball speed <= robot speed, the robot is able to intersect the ball at any point after a time t
     * If ball speed > robot speed, there is no guarantee that the intersection period is contiguous:
     *      - While the ball is getting closer, there are 0 or 1 intersection periods
     *      - While its getting farther, there are 0,1 or 2 intersection periods
     * We could search linearly until ball speed <= robot speed, after which we would use binary search 
     * However, so far, there was no need to optimize
     */


    for(int i=0; i<22; i+=2){
        float x = team[i];
        
        if(x == 1000){
            ret_dist[i/2] = MAX_DISTANCE; //--------- invalid positions get equally high distance
            // The intersection position is used by active players
            // If the active player has an invalid position (all players have an invalid position)
            // Although this should not be possible, if it happens, the intersection point is the ball
            if(ret_pos != nullptr){
                ret_pos[i]   = ball_pos_pred[0];
                ret_pos[i+1] = ball_pos_pred[1];
            }
            continue;
        }

        float y = team[i+1];
        float robot_max_displacement = 0.2; // each robot has an immediate reach radius of 0.2m
        int j=0;
        
        while(1){
            float vec_x = ball_pos_pred[j++] - x;
            float vec_y = ball_pos_pred[j++] - y;
            float b_dist_sq = vec_x*vec_x + vec_y*vec_y; // squared ball distance
            
            // If robot has reached the ball, or the ball has stopped but the robot is still not there
            if (b_dist_sq <= robot_max_displacement*robot_max_displacement or j>=prediction_array_size){
                float d = min( sqrtf(b_dist_sq), MAX_DISTANCE );
                ret_dist[i/2] = (is_player_down!=nullptr and is_player_down[i/2]) ? d+1.5 : d; // add 1.5m is robot is down
                if(ret_pos != nullptr){
                    ret_pos[i]   = ball_pos_pred[j-2];
                    ret_pos[i+1] = ball_pos_pred[j-1];
                }
                break;
            }
            robot_max_displacement += max_robot_sp_per_step;
        }
    }
}


/**
 * @brief Compute prediction of ball position until the ball stops (up to 6s)
 * This prediction is tuned for ground balls
 * @return size of ball_pos_pred
 */
int compute_ball_pos_prediction(float ball_pos_pred[], float ball[], float teammates[], float opponents[]){

    //-------------------------------- Add bias to ball velocity
    float bx = ball[0];
    float by = ball[1];

    // get closest player to ball from both teams
    float our_dist[11], their_dist[11];
    get_target_dist(bx, by, teammates, our_dist);
    get_target_dist(bx, by, opponents, their_dist);

    float our_closest = *std::min_element(our_dist, our_dist+11);
    float their_closest = *std::min_element(their_dist, their_dist+11);

    // define bias direction according to ball position
    float vec_goal_weight, vec_goal_x, vec_goal_y;
    if (bx < 0){ // defense
        vec_goal_weight = min(-bx,5.f) / 5.f; // how important is our goal direction 0-1
        vec_goal_x = -15.5 - bx; // bias direction: ball to our goal
        vec_goal_y = - by;        
    }else{
        vec_goal_weight = min(bx,12.f) / 12.f; // how important is their goal direction 0-1
        vec_goal_x = bx - 17; // bias direction: their goal to ball
        vec_goal_y = by;
    }

    float vec_goal_d = sqrtf(vec_goal_x*vec_goal_x + vec_goal_y*vec_goal_y) + 1e-6;
    float vec_smooth_x = vec_goal_weight * vec_goal_x / vec_goal_d + vec_goal_weight - 1; // vec becomes (-1,0) if vec_goal_weight=0
    float vec_smooth_y = vec_goal_weight * vec_goal_y / vec_goal_d;                       // vec becomes (-1,0) if vec_goal_weight=0
    float vec_smooth_d = sqrtf(vec_smooth_x*vec_smooth_x + vec_smooth_y*vec_smooth_y) + 1e-6;

    // define bias magnitude according to danger
    float danger = clip(our_closest - their_closest, -1, 1) * 0.5 + 0.5; // danger from 0 to 1
    float bias_magnitude = danger * 0.3 + 0.3; // bias size (min: 0.3 m/s, max: 0.6 m/s)

    // update ball velocity with bias
    float vx = ball[2] + vec_smooth_x*bias_magnitude/vec_smooth_d;
    float vy = ball[3] + vec_smooth_y*bias_magnitude/vec_smooth_d;

    //-------------------------------- Predict ball position

    // acceleration = Rolling Drag Force * mass (constant = 0.026 kg)
    // acceleration = velocity^2 * k1 + velocity * k2
    const float k1 = -0.01;
    const float k2 = -1;

    const float k1_x = (vx < 0) ? -k1 : k1; // invert k1 if vx is negative, because vx^2 absorbs the sign
    const float k1_y = (vy < 0) ? -k1 : k1; // invert k1 if vy is negative, because vy^2 absorbs the sign
    
    ball_pos_pred[0] = bx; // current ball position
    ball_pos_pred[1] = by;

    int counter = 2;

    while(counter < 600){

        // acceleration
        float acc_x = vx*vx*k1_x + vx*k2;
        float acc_y = vy*vy*k1_y + vy*k2;

        // second equation of motion: displacement = v0*t + 0.5*a*t^2
        float dx = vx*0.02 + acc_x*0.0002; // 0.5*0.02^2 = 0.0002
        float dy = vy*0.02 + acc_y*0.0002; // 0.5*0.02^2 = 0.0002

        // abort when displacement is low
        if (fabsf(dx) < 0.001 and fabsf(dy) < 0.001){
            break;
        }

        bx += dx;
        by += dy;

        // position
        ball_pos_pred[counter++] = bx;
        ball_pos_pred[counter++] = by;

        // velocity
        vx += acc_x*0.02;
        vy += acc_y*0.02;

    }

    return counter;
}

void add_AP_intersection_point(float ix, float iy, float bx, float by, float ret[]){

    bool in_x = ix >= -15 and ix <= 15;
    bool in_y = iy >= -10 and iy <= 10;

    // simplify solution by assuming ball is inside field
    bx = max(-15.f, min(bx, 15.f));
    by = max(-10.f, min(by, 10.f));

    if ( in_x and in_y ){ // intersection point is inside field
        ret[0] = ix;
        ret[1] = iy;
    }else{ // intersection point is out of bounds
        float vec_x = ix - bx;
        float vec_y = iy - by;

        if( !in_x ){ // vec_x cannot be zero, bc if bx is inside and ix is outside, ix must be different from bx
            float k = (ix < -15) ? (-15-bx)/vec_x : (15-bx)/vec_x; // intersection of vec(ball->i) and endline
            float iiy = by+k*vec_y;
            if ( iiy >= -10 and iiy <= 10 ){ // check if intersection point is inside field
                ret[0] = bx+k*vec_x;
                ret[1] = iiy;
                return;
            }
        }

        // if this code is reached, it means that the ball will intersect the sideline before going out of bounds
        // vec_y cannot be zero, bc if by is inside and iy is outside, iy must be different from by

        float k = (iy < -10) ? (-10-by)/vec_y : (10-by)/vec_y; // intersection of vec(ball->i) and endline
        float iix = bx+k*vec_x;

        // intersection point is inside field (otherwise it would have returned earlier)
        ret[0] = iix;
        ret[1] = by+k*vec_y;
        return;
    }
}


/**
 * @brief Assign available teammates to given roles
 */
void dynamic_assignment(float roles[], int role_no, float role_assignment[][3], float teammates[], int role_ids[], 
                        float marking_roles_priorities[] = nullptr){

    float cost_map[role_no][11]; // computing for all 11 teammates to simplify code

    for(int i=0; i<role_no; i++){
        get_target_dist(roles[i*2], roles[i*2+1], teammates, cost_map[i]); // Get teammates' distance to each role
    }

    // Increase distance to a given role to increase its weight in the permutation cost (thus increasing its priority over other roles)
    if(marking_roles_priorities != nullptr){
        for(int i=0; i<role_no; i++){
            for(int j=0; j<11; j++){
                cost_map[i][j] *= marking_roles_priorities[i];
            }
        }
    }

    // list available teammates for roles
    int available_teammates[11];
    int no_available_teammates = 0;
    for(int i = 0; i<11; i++){
        if(role_assignment[i][0] == -1){
            available_teammates[no_available_teammates++] = i;
        }
    }

    // compute permutations for available teammates (this works because 'available_teammates' is already sorted)
    float min_permutation_dist = std::numeric_limits<float>::max();
    int best_permutation[role_no];
    do {
        // find total cost: sum of squared distances
        // only the first 'role_no' available teammates are considered for the roles, the remaining teammates are in the dead zone
        float total_cost = powf(cost_map[0][available_teammates[0]],2);
        for(int i=1; i<role_no; i++){
            total_cost += powf(cost_map[i][available_teammates[i]],2);
        }

        if(total_cost < min_permutation_dist){
            min_permutation_dist = total_cost;
            for(int i=0; i<role_no; i++){
                best_permutation[i] = available_teammates[i];
            }
        }

        if(role_no < no_available_teammates){ // only useful for k-permutations
            std::reverse(available_teammates+role_no, available_teammates+no_available_teammates); // skip permutations in dead zone
        }
        
    } while ( std::next_permutation(available_teammates,available_teammates+no_available_teammates) );

    // assign roles to best permutation of teammates 
    for(int i=0; i<role_no; i++){
        role_assignment[best_permutation[i]][0] = role_ids[i];
        role_assignment[best_permutation[i]][1] = roles[i*2];
        role_assignment[best_permutation[i]][2] = roles[i*2+1];
    }

}


void define_man_marking_role_positions(float bx, float by, bool is_defending, float teammates[], float opponents[], 
                                       int best_opponent, float marking_roles[], float marking_roles_priorities[], float role_assignment[][3]){

    // safety is a score that considers the distance to our goal and how easy it is to mark the opponent (to avoid uncertainty)
    float opp_safety[11];  // safety = (distance to our goal)*3 + (distance to closest available teammate)

    for(int i=0; i<11; i++){
        float opp_x = opponents[i*2];  
        if (opp_x == 1000 or                                     // opponent with invalid coordinates is excluded
            (i==best_opponent and (is_defending or opp_x>bx))){  // closest opponent is excluded unless it's behind us and we're attacking
            opp_safety[i] = 1000;
            continue;
        }
        float opp_y = opponents[i*2+1];

        //------------------------------------------ compute (distance to our goal)*3
        float vec_goal_opp_x = opp_x + 15;
        float vec_goal_opp_y = opp_y;
        opp_safety[i] = sqrtf(vec_goal_opp_x*vec_goal_opp_x + vec_goal_opp_y*vec_goal_opp_y) * 3;

        //------------------------------------------ compute distance to closest available teammate
        float min_sq_dist = 50*50; // max. squared distance = 50^2 m
        
        for(int j=0; j<22; j+=2){
            const float x = teammates[j];

            if(x == 1000 or role_assignment[j/2][0] != -1){ // teammate is ignored if invalid position or already assigned
                continue;
            }

            const float dx = opp_x - x;
            const float dy = opp_y - teammates[j+1];
            min_sq_dist = min( min_sq_dist, dx*dx+dy*dy );
        }
        opp_safety[i] += sqrtf(min_sq_dist);
    }

    int idx[11] = {0,1,2,3,4,5,6,7,8,9,10}; // indices of opponents
    std::sort(idx, idx+11, [&opp_safety](size_t i, size_t j) {return opp_safety[i] < opp_safety[j];}); // sort in ascending order

    for(int i=0; i<4; i++){
        const int mr_x = i*2;
        const int mr_y = i*2+1;
        float opp_x, opp_y;
        const bool exclude = (opp_safety[idx[i]] == 1000); // this opponent has no valid coordinates or it should not be marked
        
        if ( exclude ){ 
            opp_x = bx; // teammate marks ball if nothing else is available
            opp_y = by;
            marking_roles_priorities[i] = 0.05;
        }else{
            opp_x = opponents[idx[i]*2];   // x coordinate of dangerous opponent
            opp_y = opponents[idx[i]*2+1]; // y coordinate of dangerous opponent
        }

        const float vec_opp_goal_x = -15 - opp_x;  // vector from dangerous opponent to goal
        const float vec_opp_goal_y = - opp_y;
        const float opp_d = sqrtf(vec_opp_goal_x*vec_opp_goal_x + vec_opp_goal_y*vec_opp_goal_y);

        if (!exclude){ // priority is computed after knowing the distance to our goal
            const float endline_dist = max(0.f, 15+opp_x);
            marking_roles_priorities[i] = endline_dist > 3 ? max(0.05f, 2/(endline_dist-1)) : endline_dist*0.3+0.1; // priority decays as opponents distance deviates in x from 3m
        }

        float vec_size = opp_d > 15 ? 0.1 : 1.5f / (opp_d + 1e-6f); // marking distance: opp_d/10 (min: 1.5m)
        marking_roles[mr_x] = opp_x + vec_opp_goal_x*vec_size;    // opponent position + vec in direction of our goal
        marking_roles[mr_y] = opp_y + vec_opp_goal_y*vec_size;

        //-------------- apply restrictions on marking position
        if (marking_roles[mr_x] < -13 and fabsf(marking_roles[mr_y]) < 3.2){ // no marking role inside our goal area
            marking_roles[mr_x] = -13;
        }
        marking_roles[mr_x] = max(-14.f, min(marking_roles[mr_x], 8.f)); // -14 < x < 8
        marking_roles[mr_y] = max(-9.f,  min(marking_roles[mr_y], 9.f)); //  -9 < y < 9
    }
}


/**
 * @brief Evaluate kick path, based on collision with near robots (considering the angle deviation) and potential obstruction of near opponents
 * 
 * @param target_x        coordinate x of target
 * @param target_y        coordinate y of target
 * @param target_d        target distance
 * @param bx              coordinate x of ball
 * @param by              coordinate y of ball
 * @param max_deviation   maximum angle deviation in radians
 * @param teammates       22 values: 11 pairs of (x,y) teammates' coordinates (invalid coordinates: x=1000, y=1000)
 * @param opponents       22 values: 11 pairs of (x,y) opponents' coordinates (invalid coordinates: x=1000, y=1000)
 * @param AP_index        index of active player
 * @param path_score_weight impact of path obstructions in final score (0-no impact, 1-normal impact)
 * @param kick_area_score_weight impact of kick area obstructions in final score (0-no impact, 1-normal impact)
 * 
 * @return float          score [0.0025,1]
 */
float evaluate_kick_path(float target_x, float target_y, float target_d, float bx, float by, float max_deviation, 
                         float teammates[], float opponents[], int AP_index, float path_score_weight=1, float kick_area_score_weight=1){

    const float vec_b_targ_x = target_x - bx;
    const float vec_b_targ_y = target_y - by;
    const float vec_b_targ_unit_x = vec_b_targ_x / target_d;
    const float vec_b_targ_unit_y = vec_b_targ_y / target_d;
    float vec_b_targ_rot1_x = vec_b_targ_x;
    float vec_b_targ_rot1_y = vec_b_targ_y;
    float vec_b_targ_rot2_x = vec_b_targ_x;
    float vec_b_targ_rot2_y = vec_b_targ_y;
    bool consider_deviation = max_deviation > 0.02; // consider deviation if larger than ~1 deg
    
    if(consider_deviation){
        rotate_2d_vec(vec_b_targ_rot1_x, vec_b_targ_rot1_y, max_deviation* 0.5); // path with half deviation clockwise     (half rotation seems like a good compromise)
        rotate_2d_vec(vec_b_targ_rot2_x, vec_b_targ_rot2_y, max_deviation*-0.5); // path with half deviation anticlockwise (half rotation seems like a good compromise)
    }

    const float targ_rot1_x = bx+vec_b_targ_rot1_x; // target with half deviation clockwise
    const float targ_rot1_y = by+vec_b_targ_rot1_y; // target with half deviation clockwise
    const float targ_rot2_x = bx+vec_b_targ_rot2_x; // target with half deviation anticlockwise
    const float targ_rot2_y = by+vec_b_targ_rot2_y; // target with half deviation anticlockwise

    // center of kick area, in front of kick (purpose: check if opponents are close to kick area)
    const float kick_area_x = bx + vec_b_targ_unit_x * 0.3;
    const float kick_area_y = by + vec_b_targ_unit_y * 0.3;

    //------------ Check collision with near robots (considering the angle deviation) and potential obstruction of near opponents

    float min_path_sq_dist = std::numeric_limits<float>::max();
    float min_kick_area_sq_dist = std::numeric_limits<float>::max();

    for(int i=0; i<22; i+=2){ // teammates

        if (AP_index == i/2){ continue; } // ignore active player
        const float x = teammates[i];
        if(x == 1000){ continue; } // ignore invalid player
        const float y = teammates[i+1];

        // we add 0.1m to the distance since teammates tend to get out of the way or at least not get closer
        min_path_sq_dist = min(min_path_sq_dist, sq_distance_pt_segment_or_ray(x, y,  bx, by,  target_x, target_y) + 0.1f);

        if(consider_deviation){
            min_path_sq_dist = min(min_path_sq_dist, sq_distance_pt_segment_or_ray(x, y,  bx, by,  targ_rot1_x, targ_rot1_y) + 0.1f);
            min_path_sq_dist = min(min_path_sq_dist, sq_distance_pt_segment_or_ray(x, y,  bx, by,  targ_rot2_x, targ_rot2_y) + 0.1f);
        }
    }

    for(int i=0; i<22; i+=2){ // opponents

        const float x = opponents[i];
        if(x == 1000){ continue; } // ignore invalid player
        const float y = opponents[i+1];

        min_path_sq_dist = min(min_path_sq_dist, sq_distance_pt_segment_or_ray(x, y,  bx, by,  target_x, target_y));

        if(consider_deviation){
            min_path_sq_dist = min(min_path_sq_dist, sq_distance_pt_segment_or_ray(x, y,  bx, by,  targ_rot1_x, targ_rot1_y));
            min_path_sq_dist = min(min_path_sq_dist, sq_distance_pt_segment_or_ray(x, y,  bx, by,  targ_rot2_x, targ_rot2_y));
        }

        if(kick_area_score_weight > 0){
            const float kick_area_d_x = kick_area_x - x;
            const float kick_area_d_y = kick_area_y - y;

            // check if any opponent can obstruct the kick itself
            min_kick_area_sq_dist = min(min_kick_area_sq_dist, kick_area_d_x*kick_area_d_x + kick_area_d_y*kick_area_d_y);
        }
    }

    const float path_score      = clip(sqrtf(min_path_sq_dist)      / 0.8, 0.05, 1); // [0.05,1], safe distance: 0.8m (tolerated: safe/2)
    const float kick_area_score = clip(sqrtf(min_kick_area_sq_dist) / 0.8, 0.05, 1); // [0.05,1], safe distance: 0.8m (tolerated: safe/2)

    return (1 + path_score_weight*(path_score-1)) * (1 + kick_area_score_weight*(kick_area_score-1)); // [0.05,1] * [0.05,1] = [0.0025,1]
}




/**
 * @brief Evaluate pass
 * This function does not evaluate if the ball is (nearly) out of bounds, that's the caller's responsability
 * 
 * @param target_x            coordinate x of target
 * @param target_y            coordinate y of target
 * @param target_d            target distance
 * @param bx                  coordinate x of ball
 * @param by                  coordinate y of ball
 * @param max_deviation       maximum angle deviation in radians
 * @param mean_error          mean error of Euclidean distance to target in meters
 * @param teammates           22 values: 11 pairs of (x,y) teammates' coordinates (invalid coordinates: x=1000, y=1000)
 * @param opponents           22 values: 11 pairs of (x,y) opponents' coordinates (invalid coordinates: x=1000, y=1000)
 * @param is_teammate_down    11 values: array of booleans indicating whether a teammate is currently down
 * @param AP_index            index of active player
 * @return float              kick score [0,1]
 */
float evaluate_pass(float target_x, float target_y, float target_d, float bx, float by, float max_deviation, float mean_error, 
                    float teammates[], float opponents[], bool is_teammate_down[], int AP_index){

    const float vec_b_targ_x = target_x - bx;
    const float vec_b_targ_y = target_y - by;
    const float vec_b_targ_unit_x = vec_b_targ_x / target_d;
    const float vec_b_targ_unit_y = vec_b_targ_y / target_d;

    //------------ 1. Check collision with near robots (considering the angle deviation) and potential obstruction of near opponents

    const float path_score = evaluate_kick_path(target_x, target_y, target_d, bx, by, max_deviation, teammates, opponents, AP_index);

    //------------ 2. Check if teammates are closer to target than opponents and self (why kick if I'm the closest teammate?)

    float min_teammate_target_sq_dist = std::numeric_limits<float>::max();
    float min_opponent_target_sq_dist = std::numeric_limits<float>::max();
    float receiver_x = 0;
    float receiver_y = 0;

    for(int i=0; i<22; i+=2){ // teammates

        if (AP_index == i/2 or is_teammate_down[i/2]){ continue; } // ignore active player or fallen players
        const float x = teammates[i];
        if(x == 1000){ continue; } // ignore invalid player
        const float y = teammates[i+1];

        const float vec_teammate_targ_x = target_x - x;
        const float vec_teammate_targ_y = target_y - y;
        const float sq_dist_tm_targ = vec_teammate_targ_x*vec_teammate_targ_x + vec_teammate_targ_y*vec_teammate_targ_y;
        
        if (sq_dist_tm_targ < min_teammate_target_sq_dist){ // save receiver positions (closest teammate)
            min_teammate_target_sq_dist = sq_dist_tm_targ;
            receiver_x = x;
            receiver_y = y;
        }

    }

    for(int i=0; i<22; i+=2){ // opponents

        const float x = opponents[i];
        if(x == 1000){ continue; } // ignore invalid player
        const float y = opponents[i+1];

        const float vec_opponent_targ_x = target_x - x;
        const float vec_opponent_targ_y = target_y - y;
        min_opponent_target_sq_dist = min(min_opponent_target_sq_dist, vec_opponent_targ_x*vec_opponent_targ_x + vec_opponent_targ_y*vec_opponent_targ_y);
    }

    // the mean kick error affects the result mainly when both teams are at similar distances from the target
    const float min_teammate_target_dist = sqrtf(min_teammate_target_sq_dist);
    float target_score = tanhf(sqrtf(min_opponent_target_sq_dist) - min_teammate_target_dist - mean_error) * 0.5 + 0.5; // ]0,1[ low advantage after 2m

    if (target_d < min_teammate_target_dist){ // penalize if the kicker is the closest teammate
        target_score *= 0.5;
    }

    //------------ 3. Check if pass direction is convenient to kicker

    // get angle between player->ball and ball->target
    const float ang1 = vectors_angle_rad(bx - teammates[AP_index*2], by - teammates[AP_index*2+1], false, vec_b_targ_unit_x, vec_b_targ_unit_y, true); // 0 to pi
    const float kicker_convenience_score = 1 - (0.2 / M_PI) * ang1; // 0rad -> 1, PIrad -> 0.8 (this score should have the lowest influence)

    //------------ 4. Check if pass is convenient to receiver

    const float vec_targ_goal_x = 15.5 - target_x;
    const float vec_targ_goal_y = - target_y;

    // get angle between receiver->target and target->goal
    const float ang2 = vectors_angle_rad(target_x - receiver_x, target_y - receiver_y, false, vec_targ_goal_x, vec_targ_goal_y, false); // 0 to pi
    const float receiver_convenience_score = 1 - (0.4 / M_PI) * ang2; // 0rad -> 1, PIrad -> 0.6 (this score should have a low influence)

    //------------ 5. Check if pass is in the right direction (goal)

    float targ_goal_d = sqrtf(vec_targ_goal_x*vec_targ_goal_x + vec_targ_goal_y*vec_targ_goal_y);
    float vec_b_goal_x = 15.5 - bx;
    float vec_b_goal_y = - by;
    float b_goal_d = sqrtf(vec_b_goal_x*vec_b_goal_x + vec_b_goal_y*vec_b_goal_y);

    const float forward_score = tanhf((b_goal_d - targ_goal_d)*0.25) * 0.5f + 0.5f; // ]0,1[ low advantage after 8m

    //------------ 6. Compute pass score ( it should never be zero, to discriminate bad passes )

    return path_score * target_score * kicker_convenience_score * receiver_convenience_score * forward_score;
}


#define MIN_SHOOTING_DIST 17 // this is not controllable, it's the error range
#define MAX_SHOOTING_DIST 20 // this is not controllable, it's the error range
#define MAX_SHOOTING_DEV 0.04 // maximum angle deviation in radians
#define MEAN_SHOOTING_ERROR 1
#define MAX_PASS_DIST 9
#define MIN_PASS_DIST 3
#define MAX_PASS_DEV 0.06 // maximum angle deviation in radians
#define MEAN_PASS_ERROR 0.07 // mean error per passing meter (e.g. for 4m, error=mpe*4)

/**
 * @brief Find the best long kick and save it to ret_kick_targets
 * Shooting to goal is chosen immediately is there is a good chance of scoring
 * Otherwise, all pass directions are evaluated, and either shooting or passing can be chosen
 * If shooting is not an option, the best pass is chosen, but if all passes go out of bounds, the target defaults to (-15,0)
 * 
 * @param teammates           22 values: 11 pairs of (x,y) teammates' coordinates (invalid coordinates: x=1000, y=1000)
 * @param opponents           22 values: 11 pairs of (x,y) opponents' coordinates (invalid coordinates: x=1000, y=1000)
 * @param is_teammate_down    11 values: array of booleans indicating whether a teammate is currently down
 * @param bx                  coordinate x of ball
 * @param by                  coordinate y of ball
 * @param AP_index            index of active player
 * @param ret_kick_targets    returned targets for the long kick
 */
void find_best_long_kick(float teammates[], float opponents[], bool is_teammate_down[], float bx, float by, int AP_index, float ret_kick_targets[]){

    const float GOAL_TARGET_X = 15.07; // this value is a bit over 15 to allow easier vector manipulation below

    // Check if shooting to goal is possible
    float vec_ball_goal_x = GOAL_TARGET_X - bx;
    float vec_ball_goal_x_sq = vec_ball_goal_x*vec_ball_goal_x;

    float vec_ball_gpl_y = 0.9 - by;
    float vec_ball_gpl_d = sqrtf(vec_ball_goal_x_sq + vec_ball_gpl_y*vec_ball_gpl_y);

    float vec_ball_gpr_y = -0.9 - by;
    float vec_ball_gpr_d = sqrtf(vec_ball_goal_x_sq + vec_ball_gpr_y*vec_ball_gpr_y);


    if (vec_ball_gpl_d < MIN_SHOOTING_DIST and vec_ball_gpr_d < MIN_SHOOTING_DIST){ // goal is completely within reach

        // find best shooting direction
        float max_score = 0;
        float max_score_y = 0;
        
        for( int y=-18; y<=18; y++){
            if((vec_ball_goal_x > 1 or fabsf(by) > 1.5) and y % 2 == 0){
                continue; // skip even numbers if the ball is far enough (no need for so much granularity)
            }

            float target_y = y * 0.05;
            float vec_b_targ_x = vec_ball_goal_x;
            float vec_b_targ_y = target_y - by;
            float vec_b_targ_d = sqrtf(vec_ball_goal_x_sq + vec_b_targ_y*vec_b_targ_y);

            //------------ 1. Check collision with goalposts or side of net ( post radius 0.02 + ball radius 0.042 = 0.062 )
            float gpl_sq_d = sq_distance_pt_segment_or_ray(15.02,  1.07, bx, by, GOAL_TARGET_X, target_y, true);
            float gpl_d = sqrtf(gpl_sq_d);
            float gpr_sq_d = sq_distance_pt_segment_or_ray(15.02, -1.07, bx, by, GOAL_TARGET_X, target_y, true);
            float gpr_d = sqrtf(gpr_sq_d);

            // note: if collision_d > 0.07 it means than a collision with the side net would be at x=15.09 (impossible)
            if (gpl_d < 0.07 or gpr_d < 0.07){
                continue;
            }

            // Let BGP be a triangle such that B-ball, G-Goalpost, P-projection of G onto shooting ray
            // Goalpost score [0.05,1] = actual_goalpost_dist / max_error_dist = |GP| / (|BP| * tan(max_dev))
            // Special cases: 1 if B=P (goalpost on side of ball) or gpl_d=|GB| (ball is past the goalpost)
            float lBP = sqrtf(vec_ball_gpl_d*vec_ball_gpl_d -  gpl_sq_d); // BP for left goalpost
            float l_kick_score = clip(gpl_d / (lBP * tanf(MAX_SHOOTING_DEV) + 1e-8f),0.05,1); // see explanation above

            float rBP = sqrtf(vec_ball_gpr_d*vec_ball_gpr_d -  gpr_sq_d); // BP for right goalpost
            float r_kick_score = clip(gpr_d / (rBP * tanf(MAX_SHOOTING_DEV) + 1e-8f),0.05,1); // see explanation above

            //------------ 2. Check collision with near robots and potential obstruction of near opponents

            const float vec_b_targ_unit_x = vec_b_targ_x / vec_b_targ_d;
            const float vec_b_targ_unit_y = vec_b_targ_y / vec_b_targ_d;
            float path_score;
            
            // The first 2m of the path matter the most, and also ignore the shooting deviation
            if(vec_b_targ_d > 2){
                path_score = evaluate_kick_path(bx + vec_b_targ_unit_x*2, by + vec_b_targ_unit_y*2, 2, bx, by, 0, teammates, opponents, AP_index);
                // consider the remaining path but with low weight, and ignore the kick area distance
                path_score *= evaluate_kick_path(bx + vec_b_targ_x, by + vec_b_targ_y, vec_b_targ_d-2, 
                                                 bx + vec_b_targ_unit_x*2, by + vec_b_targ_unit_y*2, 0, teammates, opponents, AP_index, 0.5, 0);
            }else{ // we are close to the goal, consider entire path
                path_score = evaluate_kick_path(bx + vec_b_targ_x, by + vec_b_targ_y, vec_b_targ_d, bx, by, 0, teammates, opponents, AP_index);
            }

            //------------ 3. Is direction convenient for kicker?

            // get angle between player->ball and ball->target
            const float ang = vectors_angle_rad(bx - teammates[AP_index*2], by - teammates[AP_index*2+1], false, vec_b_targ_unit_x, vec_b_targ_unit_y, true); // 0 to pi
            const float convenience_score = 1 - (0.2 / M_PI) * ang; // 0rad -> 1, PIrad -> 0.8 (this score should have a low influence)

            //------------ 4. Get overall score, and save target if better 
            const float kick_score = l_kick_score * r_kick_score * path_score * convenience_score;

            if (kick_score > max_score){
                max_score = kick_score;
                max_score_y = target_y;
            }
        }

        // update return values. they can still be changed below
        ret_kick_targets[3] = GOAL_TARGET_X;
        ret_kick_targets[4] = max_score_y;
        ret_kick_targets[5] = max_score;
        ret_kick_targets[6] = 1;
        

        if (max_score > 0.5){ // if score is good enough, ignore long passes, and shoot to goal
            return;
        } // otherwise, check if long passes are a better alternative
        
    }else{
        ret_kick_targets[5] = -100; // this means that the code below will always overwrite ret_kick_targets
    }


    //------------ Evaluate passes

    // side vectors consider the max deviation when checking if the target is out of bounds
    const float side_d = MAX_SHOOTING_DIST * tanf(MAX_SHOOTING_DEV); // side vector length   
    float best_pass_score = -10; 
    float best_pass_x = 15; // if the player is at the center spot and the kick is strong, these are the best pass coordinates
    float best_pass_y = 0;
    
    for (int i=0; i<360; i+=5){
        const float rad = i * 0.0174532925f; 
        const float front_x = cosf(rad); // front unit vector
        const float front_y = sinf(rad); // front unit vector
        const float left_x = -front_y * side_d;  // left vector
        const float left_y =  front_x * side_d;  // left vector


        float side_x = bx + front_x * MAX_SHOOTING_DIST + left_x; // biased target pos + left vector 
        float side_y = by + front_y * MAX_SHOOTING_DIST + left_y; // biased target pos + left vector 

        
        if ( side_x > 14 or side_x < -14 or side_y > 9 or side_y < -9 ){ continue; } // is left position out of bounds?
        side_x -= left_x + left_x; // invert with 2x right vector
        side_y -= left_y + left_y; // invert with 2x right vector
        if ( side_x > 14 or side_x < -14 or side_y > 9 or side_y < -9 ){ continue; } // is right position out of bounds?


        const float targ_x = bx + front_x * ((MAX_SHOOTING_DIST + MIN_SHOOTING_DIST) * 0.5f); // target x (mean)
        const float targ_y = by + front_y * ((MAX_SHOOTING_DIST + MIN_SHOOTING_DIST) * 0.5f); // target y (mean)

        const float s = evaluate_pass(targ_x, targ_y, (MAX_SHOOTING_DIST + MIN_SHOOTING_DIST) * 0.5f, // target distance
                                      bx, by, MAX_SHOOTING_DEV, MEAN_SHOOTING_ERROR, teammates, opponents, is_teammate_down, AP_index);

        if (s > best_pass_score){ // save best pass
            best_pass_score = s;
            best_pass_x = targ_x;
            best_pass_y = targ_y;
        }
    }

    if ( best_pass_score > ret_kick_targets[5] ){ // overwrite ret_kick_targets
        ret_kick_targets[3] = best_pass_x;
        ret_kick_targets[4] = best_pass_y;
        ret_kick_targets[5] = best_pass_score;
        ret_kick_targets[6] = 0;
    }

    ret_kick_targets[5] = max(0.f,ret_kick_targets[5]); // minimum score: 0
    
}

/**
 * @brief Find best short kick
 * Does not evaluate shooting to goal, since that is done by find_best_long_kick
 * 
 * @param teammates           22 values: 11 pairs of (x,y) teammates' coordinates (invalid coordinates: x=1000, y=1000)
 * @param opponents           22 values: 11 pairs of (x,y) opponents' coordinates (invalid coordinates: x=1000, y=1000)
 * @param is_teammate_down    11 values: array of booleans indicating whether a teammate is currently down
 * @param bx                  coordinate x of ball
 * @param by                  coordinate y of ball
 * @param AP_index            index of active player
 * @param ret_kick_targets    returned targets for the short kick
 */
void find_best_short_kick(float teammates[], float opponents[], bool is_teammate_down[], float bx, float by, int AP_index, float ret_kick_targets[]){

    float best_pass_score = -10; 
    float best_pass_x = 15; // if the player is at the center spot and the kick is strong, these are the best pass coordinates
    float best_pass_y = 0;

    for (int i=0; i<360; i+=5){
        const float rad = i * 0.0174532925f; 
        const float front_x = cosf(rad); // front unit vector
        const float front_y = sinf(rad); // front unit vector

        for( int j=MIN_PASS_DIST; j<=MAX_PASS_DIST; j++){
            const float targ_x = bx + front_x * j; // target x (mean)
            const float targ_y = by + front_y * j; // target y (mean)

            if ( targ_x > 14 or targ_x < -14 or targ_y > 9 or targ_y < -9 ){ break; } // break if out of bounds because going farther is not going to help

            const float s = evaluate_pass(targ_x, targ_y, j, bx, by, MAX_PASS_DEV, MEAN_PASS_ERROR * j, teammates, opponents, is_teammate_down, AP_index);
            
            if (s > best_pass_score){ // save best pass
                best_pass_score = s;
                best_pass_x = targ_x;
                best_pass_y = targ_y;
            }
        }
    }

    ret_kick_targets[0] = best_pass_x;
    ret_kick_targets[1] = best_pass_y;
    ret_kick_targets[2] = max(0.f,best_pass_score); // minimum score: 0
}

/**
 * 
 * @brief Get roles for self and other players with same or higher priority (which should be avoided accordingly in path planning)
 * 
 * @param teammates           22 values: 11 pairs of (x,y) teammates' coordinates (invalid coordinates: x=1000, y=1000)
 * @param opponents           22 values: 11 pairs of (x,y) opponents' coordinates (invalid coordinates: x=1000, y=1000)
 * @param roles_pos           12 values:  6 pairs of (x,y) roles' coordinates: GK,CB,LS,RS,LF,RF
 * @param ball                4  values: ball position (inside field) and velocity (bx,by,bvx,bvy)
 * @param my_index            this robot's (unum-1). This is used to return earlier, after the corresponding role is assigned. 
 *                            Setting this value to -1 means no early return.
 * @param is_teammate_down    11 values: array of booleans indicating whether a teammate is currently down
 * @param ret_role_assignment return value: role for each teammate [(roleID, role_pos_x, role_pos_y), ...]
 * @param ret_kick_targets    kick (short/long) targets for AP: [short_x, short_y, short_score, long_x, long_y, long_score, is_long_to_goal]
 * 
 */
void manage(float teammates[], float opponents[], float roles_pos[], float ball[], bool is_teammate_down[], int my_index,
            float ret_role_assignment[][3], float ret_kick_targets[], float our_max_speed, float their_max_speed){

    float* roles_pos_GK  = roles_pos;
    float* roles_pos_CB  = roles_pos+2;
    float* roles_pos_Sup = roles_pos+4;
    float* roles_pos_Fr  = roles_pos+8;
    float bx = ball[0];
    float by = ball[1];
    float ball_pos_pred[600]; // ball position (x,y) prediction for 300*0.02s = 6s 
    float our_max_sp_per_step = our_max_speed * 0.02;
    float their_max_sp_per_step = their_max_speed * 0.02;

    float prediction_array_size = compute_ball_pos_prediction(ball_pos_pred, ball, teammates, opponents);

    //==================================================================================== 1. Get closest active player (AP)

    float our_dist[11], their_dist[11], our_intersec_pts[22];

    // Get distance to intersection point for both teams
    get_intersection_with_ball(ball_pos_pred, prediction_array_size, teammates, our_dist,   our_intersec_pts, our_max_sp_per_step,  is_teammate_down);
    get_intersection_with_ball(ball_pos_pred, prediction_array_size, opponents, their_dist, nullptr,          their_max_sp_per_step);

    // Get best player from both teams
    int best_teammate = std::min_element(our_dist, our_dist+11) - our_dist;
    int best_opponent = std::min_element(their_dist, their_dist+11) - their_dist;
    float min_teammate_dist = our_dist[best_teammate];
    float min_opponent_dist = their_dist[best_opponent];

    // Assign Active player
    ret_role_assignment[best_teammate][0]=AP; // assign Active Player role
    add_AP_intersection_point( our_intersec_pts[best_teammate*2], our_intersec_pts[best_teammate*2+1], bx, by, ret_role_assignment[best_teammate]+1 );

    if(best_teammate == my_index or my_index == -1){ // compute best kick if AP or index is not known
        find_best_long_kick( teammates, opponents, is_teammate_down, bx, by, best_teammate, ret_kick_targets);
        find_best_short_kick(teammates, opponents, is_teammate_down, bx, by, best_teammate, ret_kick_targets);
        if(best_teammate == my_index){ return; } // return earlier if this robot is: AP
    }

    bool is_defending = min_teammate_dist+0.2 > min_opponent_dist; // condition to be defending

    //==================================================================================== 2. Get GK role

    if( ret_role_assignment[0][0] == -1 ){ // Call unum 1 to be GK (if not the AP)
        ret_role_assignment[0][0] = GK;
        ret_role_assignment[0][1] = roles_pos_GK[0];
        ret_role_assignment[0][2] = roles_pos_GK[1];
    }else{
        int role_ids[] = {GK};
        dynamic_assignment(roles_pos_GK, sizeof(role_ids)/sizeof(role_ids[0]), ret_role_assignment, teammates, role_ids);
    }

    if(my_index>=0 and ret_role_assignment[my_index][0] != -1){ return; } // return earlier if this robot is: GK

    //==================================================================================== 3. Get high-priority roles

    if (is_defending){ //------------------------------------- defending: 2nd active player player goes to ball

        float min_teammate_dist_2 = std::numeric_limits<float>::max();
        int best_teammate_2;

        for(int i=1; i<11; i++){ // exclude GK
            if (i!=best_teammate and our_dist[i] < min_teammate_dist_2){ // exclude AP
                min_teammate_dist_2 = our_dist[i];         // replace 2nd place
                best_teammate_2 = i;                       // replace 2nd place
            }
        }

        ret_role_assignment[best_teammate_2][0]=AP2; // assign AP2
        add_AP_intersection_point( our_intersec_pts[best_teammate_2*2], our_intersec_pts[best_teammate_2*2+1], bx, by, ret_role_assignment[best_teammate_2]+1);

    }else{ //------------------------------------------------- attacking: 1 players goes to ball, LF & RF are a priority

        // Get priority roles: LF,RF (anyone's role except for unum 1)
        int role_ids[] = {LF,RF};
        dynamic_assignment(roles_pos_Fr, sizeof(role_ids)/sizeof(role_ids[0]), ret_role_assignment, teammates, role_ids);

    }

    if(my_index>=0 and ret_role_assignment[my_index][0] != -1){ return; } // return earlier if this robot is: AP2/AP3/LF/RF

    //==================================================================================== 4. Define man-marking role positions

    float marking_roles[14];           // we need 8 floats for marking roles, but we have extra space for the dynamic assignment
    float marking_roles_priorities[7]; // we need 4 floats for marking roles, but we have extra space for the dynamic assignment

    define_man_marking_role_positions(bx, by, is_defending, teammates, opponents, best_opponent, marking_roles, marking_roles_priorities, ret_role_assignment);
    
    //==================================================================================== 5. Get defense

    if (is_defending){ //------------------------------------- defending: 4 man-marking players + CB
        
        int role_ids[] = {MM1,MM2,MM3,MM4,CB};
        marking_roles[8] = roles_pos_CB[0];
        marking_roles[9] = roles_pos_CB[1];
        marking_roles_priorities[4] = 0.125; // equivalent to marking an opponent at 17m
        dynamic_assignment(marking_roles, sizeof(role_ids)/sizeof(role_ids[0]), ret_role_assignment, teammates, role_ids, marking_roles_priorities);

    }else{ //------------------------------------- attacking: 4 man-marking players + CB + support

        int role_ids[] = {MM1,MM2,MM3,MM4,CB,LS,RS};
        marking_roles[8] = roles_pos_CB[0];
        marking_roles[9] = roles_pos_CB[1];
        marking_roles[10] = roles_pos_Sup[0];
        marking_roles[11] = roles_pos_Sup[1];
        marking_roles[12] = roles_pos_Sup[2];
        marking_roles[13] = roles_pos_Sup[3];
        marking_roles_priorities[4] = 0.125; // equivalent to marking an opponent at 17m
        marking_roles_priorities[5] = 1; // equivalent to marking an opponent at 3m
        marking_roles_priorities[6] = 1; // equivalent to marking an opponent at 3m
        dynamic_assignment(marking_roles, sizeof(role_ids)/sizeof(role_ids[0]), ret_role_assignment, teammates, role_ids, marking_roles_priorities);

    }

    if(my_index>=0 and ret_role_assignment[my_index][0] != -1){ return; } // return earlier if already assigned

    //==================================================================================== 6. Get AP3

    if (is_defending){ //------------------------------------- defending: 3rd active player player goes to ball

        float min_teammate_dist_3 = std::numeric_limits<float>::max();
        int best_teammate_3;

        for(int i=1; i<11; i++){ // exclude GK (redundant)
            if (ret_role_assignment[i][0] == -1 and our_dist[i] < min_teammate_dist_3){ // player is not assigned && is closest to predicted intersection
                min_teammate_dist_3 = our_dist[i];         // replace 3rd place
                best_teammate_3 = i;                       // replace 3rd place
            }
        }

        ret_role_assignment[best_teammate_3][0]=AP3; // assign AP3
        add_AP_intersection_point( our_intersec_pts[best_teammate_3*2], our_intersec_pts[best_teammate_3*2+1], bx, by, ret_role_assignment[best_teammate_3]+1);

    }

    //==================================================================================== 7. Get attack

    if (is_defending){ //------------------------------------- attacking: LF+RF
        
        int role_ids[] = {LF,RF};
        dynamic_assignment(roles_pos_Fr, sizeof(role_ids)/sizeof(role_ids[0]), ret_role_assignment, teammates, role_ids);

    }
    
    return;
}
