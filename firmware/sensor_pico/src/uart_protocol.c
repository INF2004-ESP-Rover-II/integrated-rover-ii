#include "uart_protocol.h"
#include "sensor_config.h"
#include "pico/stdlib.h"
#include "hardware/uart.h"
#include <string.h>

uint8_t calculate_packet_checksum(const sensor_packet_t* packet) {
    uint8_t checksum = 0;
    const uint8_t* data = (const uint8_t*)packet;
    
    // XOR all bytes except the checksum field itself
    for (size_t i = 0; i < sizeof(sensor_packet_t) - 1; i++) {
        checksum ^= data[i];
    }
    
    return checksum;
}

bool validate_packet_checksum(const sensor_packet_t* packet) {
    return (calculate_packet_checksum(packet) == packet->checksum);
}

void send_sensor_packet(const sensor_packet_t* packet) {
    // Send entire packet as binary data
    uart_write_blocking(UART_ID, (const uint8_t*)packet, sizeof(sensor_packet_t));
}