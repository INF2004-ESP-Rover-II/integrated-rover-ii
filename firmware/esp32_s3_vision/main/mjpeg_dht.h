#ifndef MJPEG_DHT_H
#define MJPEG_DHT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define MJPEG_STD_DHT_LEN 432

// Webcam MJPEG frames often leave out the Huffman tables and rely on the
// standard ones. Copies src to dst, inserting those tables before the scan if
// the frame has none, so ordinary JPEG decoders accept it.
// Returns the bytes written, or 0 if src is not a valid JPEG header or dst is
// too small. Sets *had_dht to whether src already had tables.
size_t mjpeg_with_dht (const uint8_t * src, size_t len, uint8_t * dst,
                       size_t dst_cap, bool * had_dht);

#endif /* MJPEG_DHT_H */
