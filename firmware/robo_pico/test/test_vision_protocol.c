#include <assert.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "vision_protocol.h"

static size_t make_frame(uint8_t *p, uint8_t seq, uint8_t status, uint8_t count) {
    p[0] = VISION_SYNC;
    p[1] = seq;
    p[2] = status;
    p[3] = 0x2c; /* age 300 ms: must not truncate to one byte. */
    p[4] = 0x01;
    p[5] = count;
    for (unsigned i = 0; i < count; ++i) {
        uint8_t *m = p + 6u + i * VISION_MARKER_BYTES;
        m[0] = (uint8_t)(i + 1u);
        m[1] = 0x58; m[2] = 0x02; /* 600 mm */
        m[3] = 0x2e; m[4] = 0xfb; /* -1234 centidegrees */
        m[5] = 0xa5; /* Sync value is legal inside a marker. */
    }
    size_t n = VISION_FRAME_BASE_BYTES + VISION_MARKER_BYTES * (size_t)count;
    p[n - 1u] = vision_crc8(p + 1, n - 2u);
    return n;
}

static void crc_update(uint8_t *p, size_t n) {
    p[n - 1u] = vision_crc8(p + 1, n - 2u);
}

static unsigned feed_bytes(vision_parser_t *parser, const uint8_t *p,
                           size_t n, uint32_t now, vision_frame_t *out) {
    unsigned emitted = 0;
    for (size_t i = 0; i < n; ++i) {
        if (vision_parser_feed(parser, p[i], now, out)) ++emitted;
        assert(parser->used < VISION_MAX_FRAME_BYTES);
    }
    return emitted;
}

static void test_wire_and_images(void) {
    static const uint8_t crc_input[] = "123456789";
    assert(vision_crc8(crc_input, 9) == 0xf4); /* Published CRC-8 check value. */
    /* Literal, independently specified no-marker packet: seq=0 age=0. */
    static const uint8_t golden[] = {0xa5, 0, 1, 0, 0, 0, 0x16};
    vision_parser_t parser;
    vision_parser_init(&parser);
    vision_frame_t out;
    assert(feed_bytes(&parser, golden, sizeof(golden), 0, &out) == 1);
    assert(out.status == VISION_NO_MARKER && out.count == 0);
    uint8_t p[VISION_MAX_FRAME_BYTES];
    for (uint8_t count = 1; count <= 4; ++count) {
        size_t n = make_frame(p, count, VISION_OK, count);
        assert(n == 7u + 6u * count);
        assert(feed_bytes(&parser, p, n, 10, &out) == 1);
        assert(out.count == count && out.sequence == count && out.age_ms == 300);
        assert(out.markers[count - 1u].range_mm == 600);
        assert(out.markers[0].bearing_cdeg == -1234);
        assert(out.markers[0].quality == 0xa5);
        assert(vision_frame_is_image(&out));
    }
    const uint8_t ids[] = {36, 40, 64, 255};
    for (unsigned i = 0; i < sizeof(ids)/sizeof(ids[0]); ++i) {
        size_t length = make_frame(p, (uint8_t)i, VISION_OK, 1);
        p[6] = ids[i]; crc_update(p, length);
        assert(feed_bytes(&parser, p, length, 20, &out) == 1);
        assert(out.markers[0].id == ids[i]);
    }
    assert(parser.stats.frames_valid == 9);
    assert(parser.stats.crc_errors == 0 && parser.stats.format_errors == 0);
}

static void test_failed_pose_packet(void) {
    uint8_t p[VISION_MAX_FRAME_BYTES];size_t n=make_frame(p,1,VISION_OK,1);
    p[7]=p[8]=0;p[11]=VISION_POSE_FAILED;crc_update(p,n);
    vision_parser_t parser;vision_parser_init(&parser);vision_frame_t out;
    assert(feed_bytes(&parser,p,n,1,&out)==1);
    assert(out.markers[0].range_mm==0&&out.markers[0].quality==VISION_POSE_FAILED);
    assert(vision_frame_is_image(&out)); /* Healthy processed image, unusable pose. */
    p[11]=VISION_QUALITY_OK;crc_update(p,n);
    assert(feed_bytes(&parser,p,n,2,&out)==0&&parser.stats.format_errors==1);
}
static void test_invalid_and_resync(void) {
    uint8_t bad[VISION_MAX_FRAME_BYTES], good[VISION_MAX_FRAME_BYTES];
    size_t n = make_frame(good, 20, VISION_OK, 1);
    vision_parser_t parser;
    vision_parser_init(&parser);
    vision_frame_t out;
    memcpy(bad, good, n);
    bad[n - 1u] ^= 1;
    assert(feed_bytes(&parser, bad, n, 1, &out) == 0);
    assert(parser.stats.crc_errors == 1);
    assert(feed_bytes(&parser, good, n, 2, &out) == 1);
    /* Valid CRC does not legitimise a bad ID/range/bearing/status. */
    const unsigned offsets[] = {6, 7, 9, 2, 5};
    for (unsigned i = 0; i < sizeof(offsets) / sizeof(offsets[0]); ++i) {
        vision_parser_init(&parser);
        memcpy(bad, good, n);
        if (i == 0) bad[6] = 0;
        if (i == 1) { bad[7] = bad[8] = 0; bad[11] = VISION_QUALITY_OK; }
        if (i == 2) { bad[9] = 0xff; bad[10] = 0x7f; }
        if (i == 3) bad[2] = VISION_NO_MARKER; /* Must have count=0. */
        if (i == 4) bad[5] = 255;
        crc_update(bad, n);
        assert(feed_bytes(&parser, bad, n, 10, &out) == 0);
        vision_parser_poll(&parser, 40, &out);
        assert(parser.stats.format_errors != 0);
        assert(feed_bytes(&parser, good, n, 41, &out) == 1);
    }
    vision_parser_init(&parser);
    size_t two = make_frame(bad, 1, VISION_OK, 2);
    bad[12] = bad[6]; /* Duplicate ID in one image. */
    crc_update(bad, two);
    assert(feed_bytes(&parser, bad, two, 1, &out) == 0);
}

static void test_truncation_and_receive_time(void) {
    uint8_t p[VISION_MAX_FRAME_BYTES];
    size_t n = make_frame(p, 77, VISION_OK, 1);
    vision_parser_t parser;
    vision_parser_init(&parser);
    vision_frame_t out;
    /* False four-marker header hides a complete later one-marker packet. */
    const uint8_t prefix[] = {0xa5, 0, 0, 0, 0, 4};
    assert(feed_bytes(&parser, prefix, sizeof(prefix), 100, &out) == 0);
    assert(feed_bytes(&parser, p, n, 101, &out) == 0);
    const uint8_t trailing_noise[] = {0x11, 0x22};
    assert(feed_bytes(&parser, trailing_noise, 2, 102, &out) == 0);
    assert(vision_parser_poll(&parser, 127, &out));
    assert(out.sequence == 77 && parser.frame_received_ms == 101);
    assert(parser.used == 0 && parser.stats.timeouts == 1);
    /* The same salvage when the next byte arrives must retain the old time. */
    vision_parser_init(&parser);
    feed_bytes(&parser, prefix, sizeof(prefix), 100, &out);
    feed_bytes(&parser, p, n, 101, &out);
    assert(vision_parser_feed(&parser, 0x11, 200, &out));
    assert(parser.frame_received_ms == 101);
    /* A truncated frame on its own expires without fabricating output. */
    vision_parser_init(&parser);
    feed_bytes(&parser, p, 8, UINT32_MAX - 10u, &out);
    assert(!vision_parser_poll(&parser, 20, &out));
    assert(parser.used == 0 && parser.stats.timeouts == 1);
    assert(feed_bytes(&parser, p, n, 21, &out) == 1);
}

static void test_sequence_health_and_age(void) {
    vision_parser_t parser;
    vision_parser_init(&parser);
    vision_frame_t out;
    uint8_t p[VISION_MAX_FRAME_BYTES];
    const uint8_t sequences[] = {254, 255, 0, 0, 1};
    for (unsigned i = 0; i < sizeof(sequences); ++i) {
        size_t n = make_frame(p, sequences[i], VISION_OK, 4);
        assert(feed_bytes(&parser, p, n, i, &out) == 1);
        assert(out.sequence == sequences[i]);
    }
    /* Transport exposes repeats; a future consumer must not count them as new images. */
    assert(vision_effective_age_ms(&out, 100, 200) == 403);
    assert(vision_effective_age_ms(&out, UINT32_MAX - 9u, 10) == 323);
    assert(vision_effective_age_ms(&out, 0, UINT32_MAX) == UINT32_MAX);
    for (uint8_t status = VISION_NO_MARKER; status <= VISION_CAMERA_LOST; ++status) {
        size_t n = make_frame(p, 1, status, 0);
        p[3] = p[4] = 0xff;
        crc_update(p, n);
        assert(feed_bytes(&parser, p, n, 20, &out) == 1);
        assert(vision_frame_is_image(&out) == (status == VISION_NO_MARKER));
        assert(out.age_ms == 65535);
        assert(vision_effective_age_ms(&out, 20, 20) == 65536);
    }
}

static void test_noise_and_corruption(void) {
    vision_parser_t parser;
    vision_parser_init(&parser);
    vision_frame_t out;
    uint32_t random = 7;
    for (unsigned i = 0; i < 20000; ++i) {
        random = random * 1664525u + 1013904223u;
        vision_parser_feed(&parser, (uint8_t)(random >> 24u), i / 100u, &out);
        assert(parser.used < VISION_MAX_FRAME_BYTES);
    }
    vision_parser_poll(&parser, 1000, &out);
    uint8_t p[VISION_MAX_FRAME_BYTES];
    size_t n = make_frame(p, 91, VISION_OK, 4);
    assert(feed_bytes(&parser, p, n, 1001, &out) == 1);
    assert(out.sequence == 91);
    /* Every single-bit mutation of a one-marker payload/CRC is rejected. */
    n = make_frame(p, 3, VISION_OK, 1);
    for (size_t pos = 1; pos < n; ++pos) {
        for (unsigned bit = 0; bit < 8; ++bit) {
            uint8_t corrupt[VISION_MAX_FRAME_BYTES];
            memcpy(corrupt, p, n);
            corrupt[pos] ^= (uint8_t)(1u << bit);
            vision_parser_init(&parser);
            feed_bytes(&parser, corrupt, n, 1, &out);
            vision_parser_poll(&parser, 30, &out);
            assert(parser.stats.frames_valid == 0);
        }
    }
}

int main(int argc, char **argv) {
    test_wire_and_images();
    test_failed_pose_packet();
    test_invalid_and_resync();
    test_truncation_and_receive_time();
    test_sequence_health_and_age();
    test_noise_and_corruption();
    puts("PASS: vision protocol, validation, resync, receive time, health, wrap and corruption");
    if (argc > 1) {
        FILE *input = fopen(argv[1], "rb");
        assert(input != NULL);
        vision_parser_t parser;
        vision_parser_init(&parser);
        vision_frame_t out;
        int byte;
        while ((byte = fgetc(input)) != EOF) {
            vision_parser_feed(&parser, (uint8_t)byte, 0, &out);
        }
        fclose(input);
        assert(parser.used == 0 && parser.stats.crc_errors == 0 && parser.stats.format_errors == 0);
        if (argc > 2) assert(parser.stats.frames_valid == (uint32_t)strtoul(argv[2], NULL, 10));
        printf("Fixture: decoded %" PRIu32 " valid packets\n", parser.stats.frames_valid);
    }
    return 0;
}
