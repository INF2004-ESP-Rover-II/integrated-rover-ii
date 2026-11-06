#include <inttypes.h>
#include <stdio.h>

#include "camera_uart.h"
#include "pico/stdlib.h"

/* Standalone receiver: does not link/init Wi-Fi, Sensor UART or motor code. */
int main(void) {
    stdio_init_all();
    camera_uart_init();
    sleep_ms(1500);
    puts("Camera UART diagnostic: UART1 RX=GP21, 115200 8N1. Motors not initialised.");
    uint32_t last_stats = 0;
    for (;;) {
        camera_uart_poll();
        uint32_t now = to_ms_since_boot(get_absolute_time());
        camera_packet_t packet;
        if (camera_uart_take_packet(&packet)) {
            const vision_frame_t *frame = &packet.frame;
            printf("seq=%u status=%u reported_age=%u effective_age=%" PRIu32 "ms count=%u",
                   (unsigned)frame->sequence, (unsigned)frame->status,
                   (unsigned)frame->age_ms,
                   vision_effective_age_ms(frame, packet.received_ms, now),
                   (unsigned)frame->count);
            for (unsigned i = 0; i < frame->count; ++i) {
                const vision_marker_t *marker = &frame->markers[i];
                printf(" | id=%u range=%umm bearing=%dcdeg quality=%u",
                       (unsigned)marker->id, (unsigned)marker->range_mm,
                       (int)marker->bearing_cdeg, (unsigned)marker->quality);
            }
            puts("");
        }
        if ((uint32_t)(now - last_stats) >= 2000u) {
            camera_uart_stats_t stats;
            camera_uart_get_stats(&stats);
            printf("[RX] bytes=%" PRIu32 " valid=%" PRIu32 " crc=%" PRIu32 " format=%" PRIu32
                   " timeout=%" PRIu32 " hw_errors=%" PRIu32 " overwritten=%" PRIu32 "\n",
                   stats.parser.bytes_received, stats.parser.frames_valid, stats.parser.crc_errors,
                   stats.parser.format_errors, stats.parser.timeouts,
                   stats.uart_errors, stats.mailbox_overwrites);
            last_stats = now;
        }
        sleep_ms(1);
    }
}
