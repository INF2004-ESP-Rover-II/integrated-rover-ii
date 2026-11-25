/**
 * @file obstacle_threshold.h
 * @brief Obstacle threshold firmware module.
 */
#ifndef ROVER_OBSTACLE_THRESHOLD_H
#define ROVER_OBSTACLE_THRESHOLD_H

#include <stdbool.h>
#include <stdint.h>

#define FRONT_STOP_DISTANCE_MM 150u

static inline bool
front_obstacle_detected (uint16_t distance_mm, bool b_fault)
{
    // Zero/unavailable and the 1023 out-of-range sentinel are not obstacles.
    return !b_fault && distance_mm > 0 && distance_mm < FRONT_STOP_DISTANCE_MM;
}
#endif

/* End of file. */
