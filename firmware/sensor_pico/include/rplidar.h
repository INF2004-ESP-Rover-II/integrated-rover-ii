#ifndef RPLIDAR_H
#define RPLIDAR_H

#include <stdbool.h>
#include <stdint.h>

typedef struct {
    float angle;
    float distance;
    uint8_t quality;
    bool start_flag;
} rplidar_measurement_t;

// Existing functions
bool rplidar_init(void);
bool rplidar_start_scan(void);
bool rplidar_stop_scan(void);
bool rplidar_get_measurement(rplidar_measurement_t* measurement);
void rplidar_get_stats(uint32_t* points_per_scan, uint32_t* valid_pts, uint32_t* invalid_pts);

// IRQ-based functions
void rplidar_irq_init(void);
bool rplidar_irq_get_measurement(rplidar_measurement_t* measurement);
uint32_t rplidar_irq_get_available_points(void);
void rplidar_irq_clear_buffer(void);
void rplidar_irq_get_buffer_stats(uint32_t* available, uint32_t* overflow, uint32_t* total);

#endif