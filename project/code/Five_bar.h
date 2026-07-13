#ifndef PROJECT_FIVE_BAR_H
#define PROJECT_FIVE_BAR_H

#include "zf_common_typedef.h"

typedef enum
{
    FIVE_BAR_STATUS_OK = 0,
    FIVE_BAR_STATUS_INVALID_ARGUMENT,
    FIVE_BAR_STATUS_INVALID_GEOMETRY,
    FIVE_BAR_STATUS_OUTSIDE_WORKSPACE,
    FIVE_BAR_STATUS_SINGULAR,
    FIVE_BAR_STATUS_JOINT_LIMIT
} five_bar_status_t;

typedef struct
{
    float base_spacing_m;
    float link_a_proximal_m;
    float link_a_distal_m;
    float link_b_proximal_m;
    float link_b_distal_m;
    int8 branch_a;
    int8 branch_b;
    float joint_a_min_rad;
    float joint_a_max_rad;
    float joint_b_min_rad;
    float joint_b_max_rad;
} five_bar_geometry_t;

typedef struct
{
    float joint_a_rad;
    float joint_b_rad;
} five_bar_solution_t;

typedef struct
{
    float x_m;
    float z_m;
} five_bar_point_t;

uint8 five_bar_geometry_is_valid(const five_bar_geometry_t *geometry);

// The pivot line is the origin: motor A=(-base/2, 0), motor B=(base/2, 0).
// +x points vehicle-forward and +z points downward toward the wheel.
five_bar_status_t five_bar_inverse(const five_bar_geometry_t *geometry,
                                   float x_m,
                                   float z_m,
                                   five_bar_solution_t *solution);

// Selects the lower (+z) intersection of the two distal-link circles.
five_bar_status_t five_bar_forward(const five_bar_geometry_t *geometry,
                                   float joint_a_rad,
                                   float joint_b_rad,
                                   five_bar_point_t *point);

#endif
