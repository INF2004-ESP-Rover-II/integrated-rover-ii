/*
 * ESP32-S3 USB Host UVC Webcam -> Wi-Fi MJPEG stream
 * ---------------------------------------------------------------------
 * Connects a UVC-class USB webcam (e.g. a Logitech webcam) to the
 * ESP32-S3's native USB-OTG port in HOST mode, pulls MJPEG frames from
 * it via Espressif's usb_stream component, and re-serves those frames
 * over Wi-Fi as an MJPEG stream - same idea as the camera-module sketch,
 * but the video source is now a USB webcam instead of esp_camera.
 *
 * You can point esp32_aruco_stream.py / debug_aruco.py at this exactly
 * the same way, just with a different port/path if you change it below.
 *
 * IMPORTANT - things you will likely need to tune once you can test on
 * real hardware (I can't run this myself, so treat this as a strong
 * starting point rather than guaranteed-correct):
 *   - Your Logitech webcam's supported USB UVC frame sizes/formats.
 *     640x480 is forced to match camera_calib.json, but if the
 *     webcam doesn't offer MJPEG mode at all (rare, but some cheap/older
 *     webcams are YUYV-only), this will need YUV->JPEG conversion added.
 *   - USB power: many webcams draw more current than the ESP32-S3's
 *     OTG port alone can supply - you likely need a powered USB hub or
 *     external 5V injection.
 *   - Buffer sizes (xfer/frame buffer) below are reasonable starting
 *     points but may need adjusting for your webcam's max frame size.
 *
 * Build/flash with VS Code's ESP-IDF extension, target esp32s3.
 */

#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_wifi.h"
#include "esp_http_server.h"
#include "nvs_flash.h"
#include "usb_stream.h"
#include "esp_jpeg_dec.h"
#include "esp_psram.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include "mjpeg_dht.h"
#include "aruco_detect.h"
#include "aruco_pose.h"
#include "vision_frame.h"
#include "../../common/marker_limits.h"
// WIFI_SSID, WIFI_PASSWORD: copy wifi_secrets.example.h
#include "wifi_secrets.h"

static const char * TAG = "web_cam";

// Vision results go to the Robo Pico: UART1 TX on GPIO17 to its GP21 (UART1
// RX).
#define VISION_UART UART_NUM_1
#define VISION_TX_PIN 17
#define VISION_BAUD 115200
// Wire IDs are 1..255; the compiled DICT_6X6_250 table supplies IDs 1..249.
#define VISION_ID_MIN ROVER_MARKER_ID_MIN
#define VISION_ID_MAX ROVER_MARKER_ID_MAX
// Health limits are not agreed with Vivi yet. STALLED is early enough that the
// Pico sees a failure within 1 s; CAMERA_LOST keeps the laptop publisher's 5 s.
#define VISION_STALL_US (500 * 1000)
#define VISION_LOST_US (5000 * 1000)
#define VISION_HEALTH_US (200 * 1000)

// Test without the Pico: jumper GPIO17 to GPIO18, and a second UART reads back
// and logs what really left the pin. Comment out for the rover.
#define VISION_LOOPBACK_TEST
#define LOOPBACK_UART UART_NUM_2
#define LOOPBACK_RX_PIN 18

// ---------------------------------------------------------------------
// Fixed IP on the RoverII router (LAN only, 192.168.50.0/24, gateway .1),
// so the stream URL never changes. Low address to stay clear of DHCP leases.
// Comment out USE_STATIC_IP to go back to DHCP.
// ---------------------------------------------------------------------
#define USE_STATIC_IP
#define STATIC_IP ESP_IP4TOADDR(192, 168, 50, 5)
#define STATIC_GW ESP_IP4TOADDR(192, 168, 50, 1)
#define STATIC_NETMASK ESP_IP4TOADDR(255, 255, 255, 0)

// ---------------------------------------------------------------------
// Latest-frame buffer shared between the UVC callback (producer) and the
// HTTP handlers (consumers). Protected by a mutex since they run on
// different tasks/contexts.
// ---------------------------------------------------------------------
#define MAX_JPEG_FRAME_SIZE (60 * 1024)
#define FRAME_WIDTH 640 // the size camera_calib.json was calibrated at
#define FRAME_HEIGHT 480

static SemaphoreHandle_t frame_mutex;
static uint8_t *     latest_frame_buf = NULL; // holds a copy of the newest JPEG
static size_t        latest_frame_len = 0;
static volatile bool have_frame       = false;
static uint32_t      frames_received  = 0; // guarded by frame_mutex
static int64_t       latest_frame_us =
    0; // guarded by frame_mutex: when the newest frame arrived
static volatile uint32_t frames_too_big     = 0;
static volatile uint32_t frames_lock_missed = 0;

// Buffers required by usb_stream itself (separate from the "latest frame"
// copy above). Allocated from PSRAM since they're sizable.
#define XFER_BUFFER_SIZE (32 * 1024)
#define FRAME_BUFFER_SIZE (60 * 1024)
static uint8_t * xfer_buffer_a    = NULL;
static uint8_t * xfer_buffer_b    = NULL;
static uint8_t * uvc_frame_buffer = NULL;

// ---------------------------------------------------------------------
// Called by usb_stream on every completed UVC frame. Runs in a USB task
// context - keep it fast, just copy the JPEG bytes out.
// ---------------------------------------------------------------------
static void camera_frame_cb (uvc_frame_t * frame, void * user_ptr)
{
    if (frame->frame_format != UVC_FRAME_FORMAT_MJPEG)
    {
        // Only handling MJPEG in this starting version. If your webcam
        // only offers YUYV, you'd add a JPEG encode step here.
        return;
    }
    if (0 == frame->data_bytes || frame->data_bytes > MAX_JPEG_FRAME_SIZE)
    {
        ESP_LOGW(TAG, "Dropping frame of size %u (too large or empty)",
                 (unsigned)frame->data_bytes);
        frames_too_big++;
        return;
    }

    if (pdTRUE == xSemaphoreTake(frame_mutex, pdMS_TO_TICKS(50)))
    {
        memcpy(latest_frame_buf, frame->data, frame->data_bytes);
        latest_frame_len = frame->data_bytes;
        latest_frame_us  = esp_timer_get_time();
        have_frame       = true;
        frames_received++;
        xSemaphoreGive(frame_mutex);
    }
    else
    {
        frames_lock_missed++;
    }
}

// ---------------------------------------------------------------------
// Wi-Fi station setup (blocks until connected)
// ---------------------------------------------------------------------
static EventGroupHandle_t wifi_event_group;
#define WIFI_CONNECTED_BIT BIT0

static esp_netif_t * sta_netif = NULL;

#ifdef USE_STATIC_IP
// Same approach as ESP-IDF's static_ip example: stop the DHCP client and
// set the address once associated; esp_netif then posts IP_EVENT_STA_GOT_IP.
static void set_static_ip (esp_netif_t * netif)
{
    esp_err_t err = esp_netif_dhcpc_stop(netif);
    if (err != ESP_OK && err != ESP_ERR_ESP_NETIF_DHCP_ALREADY_STOPPED)
    {
        ESP_LOGE(TAG, "Failed to stop DHCP client: %s", esp_err_to_name(err));
        return;
    }
    esp_netif_ip_info_t ip = {
        .ip      = {.addr = STATIC_IP},
        .gw      = {.addr = STATIC_GW},
        .netmask = {.addr = STATIC_NETMASK},
    };
    ESP_ERROR_CHECK(esp_netif_set_ip_info(netif, &ip));
}
#endif

static void wifi_event_handler (void * arg, esp_event_base_t event_base,
                                int32_t event_id, void * event_data)
{
    if (WIFI_EVENT == event_base && WIFI_EVENT_STA_START == event_id)
    {
        esp_wifi_connect();
#ifdef USE_STATIC_IP
    }
    else if (WIFI_EVENT == event_base && WIFI_EVENT_STA_CONNECTED == event_id)
    {
        set_static_ip(sta_netif);
#endif
    }
    else if (WIFI_EVENT == event_base &&
             WIFI_EVENT_STA_DISCONNECTED == event_id)
    {
        // Reason codes: 201 = SSID not found (wrong name or 5 GHz only), 15/204
        // = wrong password
        ESP_LOGW(TAG, "Wi-Fi disconnected (reason %d), retrying...",
                 ((wifi_event_sta_disconnected_t *)event_data)->reason);
        esp_wifi_connect();
    }
    else if (IP_EVENT == event_base && IP_EVENT_STA_GOT_IP == event_id)
    {
        ip_event_got_ip_t * event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void wifi_init_sta (void)
{
    wifi_event_group = xEventGroupCreate();

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    sta_netif = esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
                                               &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP,
                                               &wifi_event_handler, NULL));

    wifi_config_t wifi_config = {
        .sta =
            {
                .ssid     = WIFI_SSID,
                .password = WIFI_PASSWORD,
            },
    };
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Connecting to Wi-Fi SSID: %s", WIFI_SSID);
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdTRUE,
                        portMAX_DELAY);
}

// ---------------------------------------------------------------------
// HTTP: single-frame snapshot at /capture
// ---------------------------------------------------------------------
static esp_err_t capture_handler (httpd_req_t * req)
{
    if (!have_frame)
    {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR,
                            "No frame yet");
        return ESP_FAIL;
    }

    esp_err_t res = ESP_OK;
    if (pdTRUE == xSemaphoreTake(frame_mutex, pdMS_TO_TICKS(200)))
    {
        httpd_resp_set_type(req, "image/jpeg");
        res = httpd_resp_send(req, (const char *)latest_frame_buf,
                              latest_frame_len);
        xSemaphoreGive(frame_mutex);
    }
    else
    {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Busy");
        res = ESP_FAIL;
    }
    return res;
}

// ---------------------------------------------------------------------
// HTTP: MJPEG stream at /stream (same multipart format as the camera
// module version, so your Python scripts work unchanged)
// ---------------------------------------------------------------------
#define PART_BOUNDARY "123456789000000000000987654321"
static const char * STREAM_CONTENT_TYPE =
    "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char * STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char * STREAM_PART =
    "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

static esp_err_t stream_handler (httpd_req_t * req)
{
    esp_err_t res = httpd_resp_set_type(req, STREAM_CONTENT_TYPE);
    if (res != ESP_OK)
    {
        return res;
    }

    char part_buf[64];

    while (true)
    {
        if (!have_frame)
        {
            vTaskDelay(pdMS_TO_TICKS(50));
            continue;
        }

        if (xSemaphoreTake(frame_mutex, pdMS_TO_TICKS(200)) != pdTRUE)
        {
            continue;
        }
        // Take a local copy of length/pointer while holding the lock,
        // then release before the (potentially slow) network write.
        size_t len = latest_frame_len;
        // NOTE: for simplicity we send directly out of latest_frame_buf
        // while still holding the mutex, briefly blocking the UVC
        // callback. Fine for this frame rate / size; revisit if you see
        // dropped frames under load.
        res = httpd_resp_send_chunk(req, STREAM_BOUNDARY,
                                    strlen(STREAM_BOUNDARY));
        if (ESP_OK == res)
        {
            size_t hlen = snprintf(part_buf, sizeof(part_buf), STREAM_PART,
                                   (unsigned)len);
            res         = httpd_resp_send_chunk(req, part_buf, hlen);
        }
        if (ESP_OK == res)
        {
            res =
                httpd_resp_send_chunk(req, (const char *)latest_frame_buf, len);
        }
        xSemaphoreGive(frame_mutex);

        if (res != ESP_OK)
        {
            ESP_LOGW(TAG, "Stream client disconnected");
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(30));
    }
    return res;
}

static httpd_handle_t start_webserver (void)
{
    httpd_handle_t server = NULL;
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size     = 8192;

    if (ESP_OK == httpd_start(&server, &config))
    {
        httpd_uri_t capture_uri = {
            .uri     = "/capture",
            .method  = HTTP_GET,
            .handler = capture_handler,
        };
        httpd_register_uri_handler(server, &capture_uri);

        httpd_uri_t stream_uri = {
            .uri     = "/stream",
            .method  = HTTP_GET,
            .handler = stream_handler,
        };
        httpd_register_uri_handler(server, &stream_uri);
    }
    return server;
}

// ---------------------------------------------------------------------
// USB Host / UVC setup
// ---------------------------------------------------------------------
static void uvc_camera_init (void)
{
    xfer_buffer_a =
        (uint8_t *)heap_caps_malloc(XFER_BUFFER_SIZE, MALLOC_CAP_SPIRAM);
    xfer_buffer_b =
        (uint8_t *)heap_caps_malloc(XFER_BUFFER_SIZE, MALLOC_CAP_SPIRAM);
    uvc_frame_buffer =
        (uint8_t *)heap_caps_malloc(FRAME_BUFFER_SIZE, MALLOC_CAP_SPIRAM);
    latest_frame_buf =
        (uint8_t *)heap_caps_malloc(MAX_JPEG_FRAME_SIZE, MALLOC_CAP_SPIRAM);

    if (!xfer_buffer_a || !xfer_buffer_b || !uvc_frame_buffer ||
        !latest_frame_buf)
    {
        ESP_LOGE(TAG, "Failed to allocate USB/frame buffers from PSRAM - check "
                      "PSRAM is enabled");
        return;
    }

    uvc_config_t uvc_config = {
        .frame_width       = FRAME_WIDTH,
        .frame_height      = FRAME_HEIGHT,
        .frame_interval    = FPS2INTERVAL(15),
        .xfer_buffer_size  = XFER_BUFFER_SIZE,
        .xfer_buffer_a     = xfer_buffer_a,
        .xfer_buffer_b     = xfer_buffer_b,
        .frame_buffer_size = FRAME_BUFFER_SIZE,
        .frame_buffer      = uvc_frame_buffer,
        .frame_cb          = &camera_frame_cb,
        .frame_cb_arg      = NULL,
    };

    esp_err_t ret = uvc_streaming_config(&uvc_config);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "uvc_streaming_config failed: %s", esp_err_to_name(ret));
        return;
    }

    ret = usb_streaming_start();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "usb_streaming_start failed: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(TAG, "USB UVC streaming started - waiting for webcam frames...");
}

static void vision_uart_init (void)
{
    uart_config_t config = {
        .baud_rate  = VISION_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    // The driver requires an RX buffer even though this link only sends.
    ESP_ERROR_CHECK(uart_driver_install(VISION_UART, 256, 256, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(VISION_UART, &config));
    ESP_ERROR_CHECK(uart_set_pin(VISION_UART, VISION_TX_PIN, UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
}

static void vision_send (uint8_t seq, uint8_t status, int64_t arrival_us,
                         const vision_marker_t * markers, int n)
{
    uint8_t frame[VISION_FRAME_MAX];
    int64_t age_ms = (esp_timer_get_time() - arrival_us) / 1000;
    size_t  len    = vision_frame_build(frame, seq, status, age_ms, markers, n);
    uart_write_bytes(VISION_UART, frame, len);
#ifdef VISION_LOOPBACK_TEST
    ESP_LOGI(TAG,
             "uart sent: seq %u status %u age %lld N %d, first ID %d range "
             "%.0f bearing %+.2f quality %d",
             seq, status, age_ms, n, n ? markers[0].id : -1,
             n ? markers[0].range_mm : 0, n ? markers[0].bearing_deg : 0,
             n ? markers[0].quality : -1);
#endif
}

#ifdef VISION_LOOPBACK_TEST
static void loopback_task (void * arg)
{
    uart_config_t config = {
        .baud_rate  = VISION_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(LOOPBACK_UART, 1024, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(LOOPBACK_UART, &config));
    ESP_ERROR_CHECK(uart_set_pin(LOOPBACK_UART, UART_PIN_NO_CHANGE,
                                 LOOPBACK_RX_PIN, UART_PIN_NO_CHANGE,
                                 UART_PIN_NO_CHANGE));

    uint8_t          f[VISION_FRAME_MAX];
    const TickType_t wait = pdMS_TO_TICKS(100);
    uint32_t         good = 0, bad = 0, gaps = 0;
    int              last_seq = -1;
    while (true)
    {
        if (uart_read_bytes(LOOPBACK_UART, f, 1, pdMS_TO_TICKS(1000)) != 1)
        {
            ESP_LOGW(TAG,
                     "loopback: nothing on GPIO%d for 1 s. Is the jumper on?",
                     LOOPBACK_RX_PIN);
            continue;
        }
        if (f[0] != VISION_SYNC ||
            uart_read_bytes(LOOPBACK_UART, f + 1, 5, wait) != 5 ||
            f[5] > VISION_MAX_MARKERS)
        {
            bad++;
            continue;
        }
        int rest = f[5] * 6 + 1, len = 6 + rest;
        if (uart_read_bytes(LOOPBACK_UART, f + 6, rest, wait) != rest ||
            vision_crc8(f + 1, len - 2) != f[len - 1])
        {
            bad++;
            ESP_LOGW(TAG, "loopback: bad frame, %lu so far",
                     (unsigned long)bad);
            continue;
        }
        good++;
        if (last_seq >= 0 && f[1] != last_seq &&
            f[1] != (uint8_t)(last_seq + 1))
        {
            gaps++;
        }
        last_seq = f[1];
        ESP_LOGI(TAG,
                 "uart read: seq %u status %u age %u N %u | %d bytes | good "
                 "%lu bad %lu gaps %lu",
                 f[1], f[2], f[3] | f[4] << 8, f[5], len, (unsigned long)good,
                 (unsigned long)bad, (unsigned long)gaps);
        for (int i = 0; i < f[5]; i++)
        {
            const uint8_t * m = f + 6 + i * 6;
            ESP_LOGI(TAG,
                     "uart read:   ID %u range %u bearing %+.2f quality %u",
                     m[0], m[1] | m[2] << 8,
                     (int16_t)(m[3] | m[4] << 8) / 100.0f, m[5]);
        }
    }
}
#endif

// ---------------------------------------------------------------------
// The on-board pipeline: copy the newest JPEG out, decode it, detect markers,
// send the results, and log the timing. esp_new_jpeg has no grayscale output,
// so it decodes to CbYCrY, where every second byte is the luminance, and the
// detector reads that in place.
// ---------------------------------------------------------------------
#define BENCH_REPORT_FRAMES 30
#define MARKER_MM 74.0f // measured side of the printed set

static void vision_task (void * arg)
{
    const size_t in_cap  = MAX_JPEG_FRAME_SIZE + MJPEG_STD_DHT_LEN;
    const size_t out_len = FRAME_WIDTH * FRAME_HEIGHT * 2;
    uint8_t *    raw = heap_caps_malloc(MAX_JPEG_FRAME_SIZE, MALLOC_CAP_SPIRAM);
    uint8_t *    in  = heap_caps_malloc(in_cap, MALLOC_CAP_SPIRAM);
    void * work   = heap_caps_malloc(ARUCO_WORK_SIZE(FRAME_WIDTH, FRAME_HEIGHT),
                                     MALLOC_CAP_SPIRAM);
    uint8_t * out = jpeg_calloc_align(out_len, 16);
    if (!raw || !in || !work || !out)
    {
        ESP_LOGE(TAG, "bench: buffer allocation failed");
        vTaskDelete(NULL);
        return;
    }
    jpeg_dec_config_t config = DEFAULT_JPEG_DEC_CONFIG();
    config.output_type       = JPEG_PIXEL_FORMAT_CbYCrY;

    ESP_LOGI(TAG, "bench: CPU %d MHz, PSRAM %u bytes",
             CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ, (unsigned)esp_psram_get_size());

    uint32_t last_seq = 0, window_start_seq = 0, errors = 0;
    int      n        = 0;
    int64_t  sum_copy = 0, sum_decode = 0, sum_detect = 0, max_decode = 0,
             min_decode = INT64_MAX;
    int64_t  max_detect = 0, min_detect = INT64_MAX;
    size_t   sum_bytes     = 0;
    int frames_with_marker = 0, last_id = -1, last_quality = ARUCO_POSE_FAILED;
    aruco_pose_t   last_pose = {0};
    int64_t        sum_pose  = 0;
    aruco_marker_t markers[4];
    bool           had_dht      = false;
    int64_t        window_start = esp_timer_get_time();
    uint8_t        seq          = 0;
    int64_t        last_arrival_us =
        window_start;                     // the last processed image's arrival
    int64_t last_image_us = window_start; // when that image's results were sent
    int64_t last_seen_us =
        window_start; // the newest frame's arrival, processed or not
    int64_t last_health_us = window_start;

    while (true)
    {
        vTaskDelay(
            1); // always yield: the task watchdog checks this core's idle task
        int64_t now = esp_timer_get_time();
        if (now - last_image_us > VISION_STALL_US &&
            now - last_health_us >= VISION_HEALTH_US)
        {
            uint8_t status = now - last_seen_us > VISION_LOST_US
                                 ? VISION_STATUS_CAMERA_LOST
                                 : VISION_STATUS_STALLED;
            vision_send(seq, status, last_arrival_us, NULL, 0);
            last_health_us = now;
        }
        if (xSemaphoreTake(frame_mutex, pdMS_TO_TICKS(200)) != pdTRUE)
        {
            continue;
        }
        if (frames_received == last_seq)
        {
            xSemaphoreGive(frame_mutex);
            vTaskDelay(pdMS_TO_TICKS(5));
            continue;
        }
        int64_t t0      = esp_timer_get_time();
        size_t  raw_len = latest_frame_len;
        memcpy(raw, latest_frame_buf, raw_len);
        last_seq           = frames_received;
        int64_t arrival_us = latest_frame_us;
        xSemaphoreGive(frame_mutex);
        last_seen_us = arrival_us;

        size_t  in_len = mjpeg_with_dht(raw, raw_len, in, in_cap, &had_dht);
        int64_t t1     = esp_timer_get_time();
        if (0 == in_len)
        {
            errors++;
            ESP_LOGW(TAG, "bench: frame is not a usable JPEG (%u bytes)",
                     (unsigned)raw_len);
            continue;
        }

        jpeg_dec_handle_t dec = NULL;
        jpeg_dec_io_t     io  = {
            .inbuf = in, .inbuf_len = (int)in_len, .outbuf = out};
        jpeg_dec_header_info_t info = {0};
        jpeg_error_t           err  = jpeg_dec_open(&config, &dec);
        if (JPEG_ERR_OK == err)
        {
            err = jpeg_dec_parse_header(dec, &io, &info);
        }
        // The buffers are sized for FRAME_WIDTH x FRAME_HEIGHT: never decode
        // anything larger.
        if (JPEG_ERR_OK == err &&
            (info.width != FRAME_WIDTH || info.height != FRAME_HEIGHT))
        {
            ESP_LOGE(TAG, "bench: frame is %ux%u, expected %ux%u", info.width,
                     info.height, FRAME_WIDTH, FRAME_HEIGHT);
            err = JPEG_ERR_UNSUPPORT_FMT;
        }
        if (JPEG_ERR_OK == err)
        {
            err = jpeg_dec_process(dec, &io);
        }
        if (dec)
        {
            jpeg_dec_close(dec);
        }
        int64_t t2 = esp_timer_get_time();
        if (err != JPEG_ERR_OK)
        {
            errors++;
            ESP_LOGW(TAG, "bench: decode failed: %d (%u bytes, had_dht=%d)",
                     err, (unsigned)raw_len, had_dht);
            continue;
        }

        int     found = aruco_detect(out + 1, FRAME_WIDTH, FRAME_HEIGHT,
                                     FRAME_WIDTH * 2, 2, work, markers, 4);
        int64_t t3    = esp_timer_get_time();
        vision_marker_t results[VISION_MAX_MARKERS];
        int             n_results = 0;
        for (int i = 0; i < found; i++)
        {
            aruco_pose_t    pose    = aruco_pose(markers[i].corners, MARKER_MM);
            aruco_quality_t quality = aruco_pose_quality(&pose);
            if (0 == i)
            {
                frames_with_marker++;
                last_id      = markers[0].id;
                last_pose    = pose;
                last_quality = quality;
            }
            if (markers[i].id >= VISION_ID_MIN &&
                markers[i].id <= VISION_ID_MAX)
            {
                results[n_results++] = (vision_marker_t){
                    .id          = (uint8_t)markers[i].id,
                    .range_mm    = pose.ok ? pose.range_mm : 0,
                    .bearing_deg = pose.ok ? pose.bearing_deg : 0,
                    .quality     = (uint8_t)quality,
                };
            }
        }
        sum_pose += esp_timer_get_time() - t3;

        seq++;
        vision_send(seq, n_results ? VISION_STATUS_OK : VISION_STATUS_NO_MARKER,
                    arrival_us, results, n_results);
        last_arrival_us = arrival_us;
        last_image_us   = esp_timer_get_time();

        sum_copy += t1 - t0;
        sum_decode += t2 - t1;
        sum_detect += t3 - t2;
        sum_bytes += raw_len;
        if (t2 - t1 > max_decode)
        {
            max_decode = t2 - t1;
        }
        if (t2 - t1 < min_decode)
        {
            min_decode = t2 - t1;
        }
        if (t3 - t2 > max_detect)
        {
            max_detect = t3 - t2;
        }
        if (t3 - t2 < min_detect)
        {
            min_detect = t3 - t2;
        }
        n++;

        if (BENCH_REPORT_FRAMES == n)
        {
            float secs = (esp_timer_get_time() - window_start) / 1e6f;
            ESP_LOGI(
                TAG,
                "bench: decode avg %lld min %lld max %lld ms | detect avg %lld "
                "min %lld max %lld ms | "
                "pose %lld us | copy %lld ms | %.1f fps processed, camera %.1f "
                "fps | marker in %d/%d frames, last ID %d | "
                "range %.0f mm, forward %.0f mm, bearing %+.1f deg, side %.1f "
                "px, shape %.2f, quality %d | "
                "jpeg %u bytes, dht=%d | errors %lu, too big %lu, lock missed "
                "%lu",
                sum_decode / n / 1000, min_decode / 1000, max_decode / 1000,
                sum_detect / n / 1000, min_detect / 1000, max_detect / 1000,
                sum_pose / n, sum_copy / n / 1000, n / secs,
                (last_seq - window_start_seq) / secs, frames_with_marker, n,
                last_id, last_pose.range_mm, last_pose.forward_mm,
                last_pose.bearing_deg, last_pose.side_px, last_pose.shape_err,
                last_quality, (unsigned)(sum_bytes / n), had_dht,
                (unsigned long)errors, (unsigned long)frames_too_big,
                (unsigned long)frames_lock_missed);
            n        = 0;
            sum_copy = sum_decode = sum_detect = max_decode = max_detect = 0;
            min_decode = min_detect = INT64_MAX;
            sum_bytes               = 0;
            frames_with_marker      = 0;
            sum_pose                = 0;
            window_start_seq        = last_seq;
            window_start            = esp_timer_get_time();
        }
    }
}

void app_main (void)
{
    ESP_ERROR_CHECK(nvs_flash_init());

    frame_mutex = xSemaphoreCreateMutex();

    wifi_init_sta();

    esp_netif_ip_info_t ip_info;
    esp_netif_t *       netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    esp_netif_get_ip_info(netif, &ip_info);
    ESP_LOGI(TAG, "======================================================");
    ESP_LOGI(TAG, " MJPEG stream will be at: http://" IPSTR "/stream",
             IP2STR(&ip_info.ip));
    ESP_LOGI(TAG, " Snapshot will be at:     http://" IPSTR "/capture",
             IP2STR(&ip_info.ip));
    ESP_LOGI(TAG, "======================================================");

    start_webserver();
    uvc_camera_init();

    vision_uart_init();
#ifdef VISION_LOOPBACK_TEST
    xTaskCreatePinnedToCore(loopback_task, "loopback", 4096, NULL, 2, NULL, 0);
#endif
    // Core 1, low priority: Wi-Fi runs on core 0, and the USB task on core 1
    // outranks this.
    xTaskCreatePinnedToCore(vision_task, "vision", 4096, NULL, 2, NULL, 1);
}
