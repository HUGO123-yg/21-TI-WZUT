#include "Five_bar.h"

#include <math.h>

#define FIVE_BAR_EPSILON_M       (0.000001f)
#define FIVE_BAR_ACOS_TOLERANCE  (0.0001f)

static float five_bar_clamp(float value, float minimum, float maximum)
{
    if (value < minimum)
    {
        return minimum;
    }
    if (value > maximum)
    {
        return maximum;
    }
    return value;
}

static uint8 five_bar_triangle_is_reachable(float proximal,
                                             float distal,
                                             float radius)
{
    return (uint8)((radius > FIVE_BAR_EPSILON_M)
                   && (radius <= proximal + distal + FIVE_BAR_EPSILON_M)
                   && (radius + FIVE_BAR_EPSILON_M
                       >= fabsf(proximal - distal)));
}

uint8 five_bar_geometry_is_valid(const five_bar_geometry_t *geometry)
{
    if (0 == geometry)
    {
        return 0;
    }

    return (uint8)((geometry->base_spacing_m > 0.0f)
                   && (geometry->link_a_proximal_m > 0.0f)
                   && (geometry->link_a_distal_m > 0.0f)
                   && (geometry->link_b_proximal_m > 0.0f)
                   && (geometry->link_b_distal_m > 0.0f)
                   && ((1 == geometry->branch_a) || (-1 == geometry->branch_a))
                   && ((1 == geometry->branch_b) || (-1 == geometry->branch_b))
                   && (geometry->joint_a_min_rad < geometry->joint_a_max_rad)
                   && (geometry->joint_b_min_rad < geometry->joint_b_max_rad));
}

five_bar_status_t five_bar_inverse(const five_bar_geometry_t *geometry,
                                   float x_m,
                                   float z_m,
                                   five_bar_solution_t *solution)
{
    float vector_a_x;
    float vector_b_x;
    float radius_a;
    float radius_b;
    float cosine_a;
    float cosine_b;
    float joint_a;
    float joint_b;

    if ((0 == geometry) || (0 == solution))
    {
        return FIVE_BAR_STATUS_INVALID_ARGUMENT;
    }
    if (!five_bar_geometry_is_valid(geometry))
    {
        return FIVE_BAR_STATUS_INVALID_GEOMETRY;
    }

    vector_a_x = x_m + 0.5f * geometry->base_spacing_m;
    vector_b_x = x_m - 0.5f * geometry->base_spacing_m;
    radius_a = sqrtf(vector_a_x * vector_a_x + z_m * z_m);
    radius_b = sqrtf(vector_b_x * vector_b_x + z_m * z_m);

    if (!five_bar_triangle_is_reachable(geometry->link_a_proximal_m,
                                        geometry->link_a_distal_m,
                                        radius_a)
        || !five_bar_triangle_is_reachable(geometry->link_b_proximal_m,
                                           geometry->link_b_distal_m,
                                           radius_b))
    {
        return FIVE_BAR_STATUS_OUTSIDE_WORKSPACE;
    }

    cosine_a = (geometry->link_a_proximal_m
                * geometry->link_a_proximal_m
                + radius_a * radius_a
                - geometry->link_a_distal_m * geometry->link_a_distal_m)
               / (2.0f * geometry->link_a_proximal_m * radius_a);
    cosine_b = (geometry->link_b_proximal_m
                * geometry->link_b_proximal_m
                + radius_b * radius_b
                - geometry->link_b_distal_m * geometry->link_b_distal_m)
               / (2.0f * geometry->link_b_proximal_m * radius_b);

    if ((cosine_a < -1.0f - FIVE_BAR_ACOS_TOLERANCE)
        || (cosine_a > 1.0f + FIVE_BAR_ACOS_TOLERANCE)
        || (cosine_b < -1.0f - FIVE_BAR_ACOS_TOLERANCE)
        || (cosine_b > 1.0f + FIVE_BAR_ACOS_TOLERANCE))
    {
        return FIVE_BAR_STATUS_OUTSIDE_WORKSPACE;
    }

    cosine_a = five_bar_clamp(cosine_a, -1.0f, 1.0f);
    cosine_b = five_bar_clamp(cosine_b, -1.0f, 1.0f);
    joint_a = atan2f(z_m, vector_a_x)
              + (float)geometry->branch_a * acosf(cosine_a);
    joint_b = atan2f(z_m, vector_b_x)
              + (float)geometry->branch_b * acosf(cosine_b);

    if ((joint_a < geometry->joint_a_min_rad)
        || (joint_a > geometry->joint_a_max_rad)
        || (joint_b < geometry->joint_b_min_rad)
        || (joint_b > geometry->joint_b_max_rad))
    {
        return FIVE_BAR_STATUS_JOINT_LIMIT;
    }

    solution->joint_a_rad = joint_a;
    solution->joint_b_rad = joint_b;
    return FIVE_BAR_STATUS_OK;
}

five_bar_status_t five_bar_forward(const five_bar_geometry_t *geometry,
                                   float joint_a_rad,
                                   float joint_b_rad,
                                   five_bar_point_t *point)
{
    float elbow_a_x;
    float elbow_a_z;
    float elbow_b_x;
    float elbow_b_z;
    float delta_x;
    float delta_z;
    float elbow_distance;
    float along_distance;
    float height_squared;
    float height;
    float direction_x;
    float direction_z;
    float base_x;
    float base_z;
    float candidate_1_x;
    float candidate_1_z;
    float candidate_2_x;
    float candidate_2_z;

    if ((0 == geometry) || (0 == point))
    {
        return FIVE_BAR_STATUS_INVALID_ARGUMENT;
    }
    if (!five_bar_geometry_is_valid(geometry))
    {
        return FIVE_BAR_STATUS_INVALID_GEOMETRY;
    }
    if ((joint_a_rad < geometry->joint_a_min_rad)
        || (joint_a_rad > geometry->joint_a_max_rad)
        || (joint_b_rad < geometry->joint_b_min_rad)
        || (joint_b_rad > geometry->joint_b_max_rad))
    {
        return FIVE_BAR_STATUS_JOINT_LIMIT;
    }

    elbow_a_x = -0.5f * geometry->base_spacing_m
                + geometry->link_a_proximal_m * cosf(joint_a_rad);
    elbow_a_z = geometry->link_a_proximal_m * sinf(joint_a_rad);
    elbow_b_x = 0.5f * geometry->base_spacing_m
                + geometry->link_b_proximal_m * cosf(joint_b_rad);
    elbow_b_z = geometry->link_b_proximal_m * sinf(joint_b_rad);
    delta_x = elbow_b_x - elbow_a_x;
    delta_z = elbow_b_z - elbow_a_z;
    elbow_distance = sqrtf(delta_x * delta_x + delta_z * delta_z);
    if (elbow_distance <= FIVE_BAR_EPSILON_M)
    {
        return FIVE_BAR_STATUS_SINGULAR;
    }
    if ((elbow_distance
         > geometry->link_a_distal_m + geometry->link_b_distal_m
           + FIVE_BAR_EPSILON_M)
        || (elbow_distance + FIVE_BAR_EPSILON_M
            < fabsf(geometry->link_a_distal_m
                    - geometry->link_b_distal_m)))
    {
        return FIVE_BAR_STATUS_OUTSIDE_WORKSPACE;
    }

    along_distance = (geometry->link_a_distal_m
                      * geometry->link_a_distal_m
                      - geometry->link_b_distal_m
                        * geometry->link_b_distal_m
                      + elbow_distance * elbow_distance)
                     / (2.0f * elbow_distance);
    height_squared = geometry->link_a_distal_m
                     * geometry->link_a_distal_m
                     - along_distance * along_distance;
    if (height_squared < -FIVE_BAR_EPSILON_M)
    {
        return FIVE_BAR_STATUS_OUTSIDE_WORKSPACE;
    }
    height = sqrtf((height_squared > 0.0f) ? height_squared : 0.0f);
    direction_x = delta_x / elbow_distance;
    direction_z = delta_z / elbow_distance;
    base_x = elbow_a_x + along_distance * direction_x;
    base_z = elbow_a_z + along_distance * direction_z;
    candidate_1_x = base_x - height * direction_z;
    candidate_1_z = base_z + height * direction_x;
    candidate_2_x = base_x + height * direction_z;
    candidate_2_z = base_z - height * direction_x;

    if (candidate_1_z >= candidate_2_z)
    {
        point->x_m = candidate_1_x;
        point->z_m = candidate_1_z;
    }
    else
    {
        point->x_m = candidate_2_x;
        point->z_m = candidate_2_z;
    }
    return FIVE_BAR_STATUS_OK;
}
