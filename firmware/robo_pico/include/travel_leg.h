/**
 * @file travel_leg.h
 * @brief Travel leg firmware module.
 */
#ifndef ROVER_TRAVEL_LEG_H
#define ROVER_TRAVEL_LEG_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef enum
{
    LEG_NORTH,
    LEG_EAST,
    LEG_SOUTH,
    LEG_WEST,
    LEG_DIRECTION_UNKNOWN
} leg_direction_t;
typedef enum
{
    LEG_NONE,
    LEG_SCANNING,
    LEG_SELECTING,
    LEG_CONFIGURED,
    LEG_TURNING,
    LEG_APPROACHING,
    LEG_HOLDING,
    LEG_BRAKING,
    LEG_VERIFYING,
    LEG_BACKING_UP,
    LEG_REACQUIRING,
    LEG_SEARCHING_REFERENCE,
    LEG_OBSTACLE_VERIFYING,
    LEG_COMPLETE,
    LEG_CANCELLED,
    LEG_ALIGNING,
    LEG_WAITING_CAMERA,
    LEG_WAITING_INPUT,
    LEG_CORRECTING
} leg_phase_t;

typedef struct
{
    uint32_t
        leg_id; // 0 before the first leg; clear preserves the last issued ID.
    uint8_t departure_node; // Node IDs: 0 = unset, otherwise 1..255.
    uint8_t next_target_node;
    uint8_t arrival_reference_node;
    int16_t target_x;
    int16_t target_y;
    bool    target_cell_valid; // (0,0) is a real cell, not an unknown sentinel.
    bool    configured;
    leg_direction_t direction;
    leg_phase_t     phase;
} travel_leg_t;

// Storage/reporting only. These functions have no map, sensor or motor access.

void travel_leg_init (travel_leg_t * p_leg);


bool travel_leg_clear (travel_leg_t * p_leg);


const char * travel_leg_phase_name (leg_phase_t phase);


const char * travel_leg_direction_name (leg_direction_t direction);


size_t travel_leg_format_json (const travel_leg_t * p_leg, char * p_out,
                               size_t capacity);


size_t travel_leg_format_status (const travel_leg_t * p_leg, char * p_out,
                                 size_t capacity);


#endif

/* End of file. */
