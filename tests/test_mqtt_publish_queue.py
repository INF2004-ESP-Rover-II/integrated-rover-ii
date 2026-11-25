"""Generate a native harness around the actual publisher and g_mission reporter.

Transport/time are mocked; production function bodies are extracted unchanged.
Run this script, then compile build/test_mqtt_publish_queue.c with the policy,
mission_control and travel_leg sources (MSVC /std:c11 /W4 /WX or a C11 compiler).
"""
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[1]


def extract_function(source, signature):
    name = re.search(r'(\w+)\($', signature).group(1)
    match = re.search(r'(?m)^(?:static\s+)?\w+(?:\s*\*)?\s+' + name +
                      r'\s*\([^;{}]*\)\s*\{', source)
    if match is None:
        raise ValueError(f'Function definition not found: {name}')
    start = match.start()
    brace = source.index('{', match.start())
    end, depth = brace + 1, 1
    while depth:
        depth += (source[end] == '{') - (source[end] == '}')
        end += 1
    return source[start:end] + '\n'


def generate(diagnostics=False):
    source = (ROOT / 'firmware/robo_pico/src/wifi_mqtt.c').read_text(encoding='utf-8')
    header = (ROOT / 'firmware/robo_pico/include/wifi_mqtt.h').read_text(encoding='utf-8')
    errors = re.search(r'typedef enum\s*\{\s*WIFI_OK.*?\}\s*wifi_mqtt_error_t;', header, re.S).group()
    reports = re.search(r'typedef enum\s*\{\s*MQTT_REPORT_MISSION.*?\}\s*mqtt_report_t;', header, re.S).group()
    globals_start = re.search(r'static unsigned\s+g_publishes_in_flight', source).start()
    globals_code = source[globals_start:source.index('// FR-08:')]
    functions = ''.join(extract_function(source, signature) for signature in (
        'static void reset_publish_state(', 'static bool publish_warning_due(',
        'static void mqtt_publish_cb(', 'static unsigned mqtt_free_requests(',
        'static size_t mqtt_free_output_bytes('))
    functions += re.search(r'typedef struct\s*\{[^{}]*\}\s*publish_message_context_t;', source).group() + '\n'
    helpers = re.findall(r'(?m)^publish_message_(prepare|inputs|motion|camera)\s*\(', source)
    functions += ''.join(extract_function(source, f'static bool publish_message_{suffix}(')
                         for suffix in helpers)
    functions += ''.join(extract_function(source, signature) for signature in (
        'static wifi_mqtt_error_t publish_message(',
        'wifi_mqtt_error_t mqtt_publish_status(', 'void mqtt_service_status_queue(',
        'wifi_mqtt_error_t mqtt_publish_report('))
    if diagnostics:
        functions += ''.join(extract_function(source, signature) for signature in (
            'void mqtt_diagnostics_loop_begin(', 'void mqtt_diagnostics_poll_begin(',
            'void mqtt_diagnostics_poll_end(', 'void mqtt_diagnostics_report('))
    main = (ROOT / 'firmware/robo_pico/src/main.c').read_text(encoding='utf-8')
    reporter = extract_function(main, 'static void report_mission(')
    prefix = r'''
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <inttypes.h>
#include <string.h>
#include "mqtt_publish_policy.h"
#include "mission_control.h"
#define ROVER_MQTT_DIAGNOSTICS 0
#define MQTT_REQ_MAX_IN_FLIGHT 16u
#define MQTT_OUTPUT_RINGBUF_SIZE 8192u
#define MQTT_TOPIC_STATUS "robot/status"
typedef int err_t;
typedef uint16_t u16_t;
enum { ERR_OK=0, ERR_MEM=-1 };
typedef struct request {
    struct request *next;
    void (*callback)(void *, err_t);
    void *arg;
} request_t;
typedef struct {
    request_t req_list[MQTT_REQ_MAX_IN_FLIGHT];
    struct { uint16_t put, get; } output;
} mock_client_t;
static mock_client_t transport;
static mock_client_t *gp_mqtt_client = &transport;
static bool g_b_mqtt_connected_flag = true;
static uint32_t clock_ms;
static unsigned sdk_calls, sent_count;
static err_t forced_result;
static char sent_messages[128][1024];
static mission_t g_mission;
static uint32_t get_absolute_time(void) { return clock_ms; }
static uint32_t to_ms_since_boot(uint32_t value) { return value; }
#if ROVER_MQTT_DIAGNOSTICS
static uint32_t time_us_32(void) { return clock_ms * 1000u; }
static bool stdio_usb_connected(void) { return true; }
static uint32_t g_last_link_rx_ms;
#endif
static bool mqtt_client_is_connected(mock_client_t *client) { return client != NULL; }
static bool mqtt_is_connected(void) { return g_b_mqtt_connected_flag; }
static err_t mqtt_publish(mock_client_t *client, const char *topic, const char *data, uint16_t length,
    int qos, int retain, void (*callback)(void *, err_t), void *arg)
{
    (void)qos; (void)retain;
    sdk_calls++;
    if (forced_result != ERR_OK) return forced_result;
    assert(length < sizeof(sent_messages[0]) && sent_count < 128);
    memcpy(sent_messages[sent_count], data, (size_t)length + 1u);
    sent_count++;
    for (unsigned i = 0; i < MQTT_REQ_MAX_IN_FLIGHT; ++i) {
        request_t *r = &client->req_list[i];
        if (r->next == r) {
            r->next = NULL; r->callback = callback; r->arg = arg;
            client->output.put = (uint16_t)((client->output.put + length + strlen(topic) + 7u) % MQTT_OUTPUT_RINGBUF_SIZE);
            return ERR_OK;
        }
    }
    assert(!"Application admitted publication without a free request");
    return ERR_MEM;
}
static void reset_transport(void)
{
    memset(&transport, 0, sizeof(transport));
    for (unsigned i = 0; i < MQTT_REQ_MAX_IN_FLIGHT; ++i)
        transport.req_list[i].next = &transport.req_list[i];
}
static void complete_all(void)
{
    for (unsigned i = 0; i < MQTT_REQ_MAX_IN_FLIGHT; ++i) {
        request_t *r = &transport.req_list[i];
        if (r->next != r && r->callback) r->callback(r->arg, ERR_OK);
        r->next = r;
    }
    transport.output.get = transport.output.put;
}
'''
    checks = r'''
int main(void)
{
    reset_transport(); reset_publish_state(); clock_ms = 1000;
    for (unsigned r = 0; r < 4; ++r)
        assert(mqtt_publish_report((mqtt_report_t)r, "initial") == WIFI_OK);
    for (; clock_ms < 5000; clock_ms++)
        for (unsigned r = 0; r < 4; ++r)
            assert(mqtt_publish_report((mqtt_report_t)r, "newer") == MQTT_ERR_BUSY);
    assert(sdk_calls == 4 && g_publishes_in_flight == 4);
    for (unsigned k = PUBLISH_MISSION; k < PUBLISH_COUNT; ++k)
        assert(g_publish_policy.slots[k].pending == 1);
    assert(publish_message("robot/telemetry", "{}", PUBLISH_SENSOR) == WIFI_OK);
    for (unsigned i = 0; i < 4; ++i)
        assert(mqtt_publish_status("immediate reply") == WIFI_OK);
    assert(g_publishes_in_flight == 9); /* Four routines, telemetry, four replies. */
    for (unsigned i = 0; i < MQTT_REPLY_QUEUE_CAPACITY; ++i) {
        char text[32]; snprintf(text, sizeof(text), "queued reply %u", i);
        assert(mqtt_publish_status(text) == WIFI_OK);
    }
    assert(g_reply_count == MQTT_REPLY_QUEUE_CAPACITY && g_publishes_in_flight == 9);
    assert(mqtt_publish_status("overflow") == MQTT_ERR_BUSY && g_reply_dropped == 1);
    assert(strcmp(g_reply_queue[g_reply_head], "queued reply 0") == 0);
    assert(publish_message("robot/camera", "{}", PUBLISH_CAMERA) == MQTT_ERR_BUSY);
    mqtt_service_status_queue();
    uint32_t first_attempt = g_reply_last_attempt_ms;
    for (unsigned i = 0; i < 199; ++i) { clock_ms++; mqtt_service_status_queue(); }
    assert(g_reply_last_attempt_ms == first_attempt && sdk_calls == 9);
    complete_all();
    assert(!g_publishes_in_flight);
    unsigned before = sent_count;
    for (unsigned i = 0; i < MQTT_REPLY_QUEUE_CAPACITY; ++i) {
        clock_ms += 200; mqtt_service_status_queue();
        char expected[32]; snprintf(expected, sizeof(expected), "queued reply %u", i);
        assert(sent_count == before + i + 1 && strcmp(sent_messages[before+i], expected) == 0);
        complete_all();
    }
    assert(!g_reply_count);

    /* Real ring/request checks gate lwIP before an ERR_MEM storm can happen. */
    unsigned calls = sdk_calls;
    transport.output.get = 0; transport.output.put = 8000;
    clock_ms += 1000;
    assert(publish_message("robot/map", "{}", PUBLISH_MAP) == MQTT_ERR_BUSY);
    assert(publish_message("robot/telemetry", "{}", PUBLISH_SENSOR) == MQTT_ERR_BUSY);
    assert(mqtt_publish_status("deferred on bytes") == WIFI_OK && g_reply_count == 1);
    assert(sdk_calls == calls);
    reset_publish_state(); reset_transport();
    assert(!g_reply_count && g_reply_dropped == 2);
    mqtt_service_status_queue(); assert(sdk_calls == calls); /* No stale replay. */
    for (unsigned i = 0; i < 13; ++i) transport.req_list[i].next = NULL;
    clock_ms += 1000;
    assert(publish_message("robot/map", "{}", PUBLISH_MAP) == MQTT_ERR_BUSY);
    assert(publish_message("robot/telemetry", "{}", PUBLISH_SENSOR) == WIFI_OK);
    assert(mqtt_publish_status("reply despite subscription pressure") == WIFI_OK);
    assert(mqtt_free_requests() == 1);
    complete_all(); reset_publish_state(); reset_transport();
    transport.output.get = 8100; transport.output.put = 100;
    assert(mqtt_free_output_bytes() == 8000); /* Wrapped ring indices. */
    reset_transport();

    /* Failed reports release their slot and retry with the latest g_mission state. */
    clock_ms += 1000; forced_result = ERR_MEM; calls = sdk_calls;
    assert(mqtt_publish_report(MQTT_REPORT_TARGET, "old") == MQTT_ERR_BUSY);
    assert(!g_publish_policy.slots[PUBLISH_TARGET].pending && !g_publishes_in_flight);
    forced_result = ERR_OK; clock_ms += 199;
    assert(mqtt_publish_report(MQTT_REPORT_TARGET, "new") == MQTT_ERR_BUSY);
    assert(sdk_calls == calls + 1);
    clock_ms++;
    assert(mqtt_publish_report(MQTT_REPORT_TARGET, "latest") == WIFI_OK);
    assert(strcmp(sent_messages[sent_count-1], "latest") == 0);
    complete_all(); reset_publish_state(); reset_transport();
    mission_init(&g_mission, clock_ms); clock_ms += 1000;
    forced_result = ERR_MEM; calls = sdk_calls;
    report_mission(false);
    assert(sdk_calls == calls + 1);
    g_mission.revision++; g_mission.reason = MISSION_REASON_RESET;
    forced_result = ERR_OK; clock_ms += 50;
    report_mission(false); assert(sdk_calls == calls + 1);
    clock_ms += 150;
    report_mission(false); assert(sdk_calls == calls + 2);
    assert(strstr(sent_messages[sent_count-1], "reason=RESET revision=1"));
    complete_all(); clock_ms += 999;
    report_mission(false); assert(sdk_calls == calls + 2);
    clock_ms++; report_mission(false); assert(sdk_calls == calls + 3);
    complete_all();
#if ROVER_MQTT_DIAGNOSTICS
    mqtt_diagnostics_loop_begin(); mqtt_diagnostics_poll_begin();
    clock_ms += 20;
    mqtt_diagnostics_poll_end(); mqtt_diagnostics_loop_begin(); mqtt_diagnostics_poll_begin();
    assert(g_loop_gap_max_us == 20000 && g_poll_gap_max_us == 20000 && g_poll_duration_max_us == 20000);
    mqtt_diagnostics_report(50);
    assert(!g_loop_gap_max_us && !g_poll_gap_max_us && !g_poll_duration_max_us);
    for (unsigned k = 0; k < PUBLISH_COUNT; ++k) assert(!g_publication_stats[k].pending);
#endif
    puts("MQTT stalled-transport, FIFO, retry, reservation and latest-state tests passed");
    return 0;
}
'''
    if diagnostics:
        prefix = prefix.replace('#define ROVER_MQTT_DIAGNOSTICS 0', '#define ROVER_MQTT_DIAGNOSTICS 1')
    output = ROOT / ('build/test_mqtt_publish_queue_diag.c' if diagnostics else 'build/test_mqtt_publish_queue.c')
    output.parent.mkdir(exist_ok=True)
    output.write_text(prefix + errors + reports + globals_code + functions + reporter + checks, encoding='utf-8')
    print(output)


if __name__ == '__main__':
    generate('--diagnostics' in sys.argv)
