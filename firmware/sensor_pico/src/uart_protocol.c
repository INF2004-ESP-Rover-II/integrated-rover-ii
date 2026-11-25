/**
 * @file uart_protocol.c
 * @brief Uart protocol firmware module.
 */
#include "uart_protocol.h"
#include "hardware/uart.h"
#include "pico/stdlib.h"
#include "sensor_config.h"
#include <string.h>

uint8_t
calculate_packet_checksum (const sensor_packet_t * p_packet)
{
    uint8_t         checksum = 0;
    const uint8_t * p_data   = (const uint8_t *) p_packet;

    // XOR all bytes except the checksum field itself
    for (size_t index = 0; index < sizeof(sensor_packet_t) - 1; index++)
    {
        checksum ^= (uint32_t) (p_data[index]);
    }

    return checksum;
}

bool
validate_packet_checksum (const sensor_packet_t * p_packet)
{
    return (calculate_packet_checksum(p_packet) == p_packet->checksum);
}

void
send_sensor_packet (const sensor_packet_t * p_packet)
{
    // Send entire packet as binary data
    uart_write_blocking(UART_ID, (const uint8_t *) p_packet,
                        sizeof(sensor_packet_t));
}

/* End of file. */
