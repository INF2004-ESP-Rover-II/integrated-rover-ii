#include "aruco_detect.h"

#include <math.h>
#include <string.h>

#include "aruco_dict.h"

// Limits follow OpenCV's DetectorParameters defaults where one exists.
#ifndef ARUCO_THRESH_WINDOW
#define ARUCO_THRESH_WINDOW 23  // adaptive threshold window, pixels (odd)
#endif
#ifndef ARUCO_THRESH_C
#define ARUCO_THRESH_C 7        // adaptiveThreshConstant
#endif
#define POLY_RATE 0.03f         // polygonalApproxAccuracyRate
#define MIN_PERIMETER 40        // pixels of contour
#define MIN_SIDE 6.0f           // pixels
#define BORDER_MARGIN 3         // minDistanceToBorder
#define MAX_BORDER_ERRORS 9     // maxErroneousBitsInBorderRate 0.35 of 28 cells
#define MAX_HAMMING 3           // errorCorrectionRate 0.6 of 5 for DICT_6X6_250
#define MIN_CONTRAST 20         // grey levels between the darkest and lightest samples
#define CELLS 8                 // 6x6 bits plus a 1-cell border

typedef struct { int16_t x, y; } pt_t;

static const int8_t DX[8] = {1, 1, 0, -1, -1, -1, 0, 1};  // clockwise on screen, y down
static const int8_t DY[8] = {0, 1, 1, 1, 0, -1, -1, -1};

#define PX(x, y) (img[(y) * stride + (x) * pixel_step])

// map[i]: 0 background, 1 dark, 2 dark and already on a traced contour.
static void threshold(const uint8_t *img, int w, int h, int stride, int pixel_step,
                      uint8_t *map, uint32_t *col)
{
    const int r = ARUCO_THRESH_WINDOW / 2;
    memset(col, 0, (size_t)w * sizeof(uint32_t));
    for (int y = 0; y <= r && y < h; y++) {
        const uint8_t *p = img + (size_t)y * stride;
        for (int x = 0; x < w; x++, p += pixel_step) col[x] += *p;
    }
    for (int y = 0; y < h; y++) {
        if (y > 0) {
            if (y + r < h) {
                const uint8_t *p = img + (size_t)(y + r) * stride;
                for (int x = 0; x < w; x++, p += pixel_step) col[x] += *p;
            }
            if (y - r - 1 >= 0) {
                const uint8_t *p = img + (size_t)(y - r - 1) * stride;
                for (int x = 0; x < w; x++, p += pixel_step) col[x] -= *p;
            }
        }
        const int y0 = y - r < 0 ? 0 : y - r, y1 = y + r >= h ? h - 1 : y + r, rows = y1 - y0 + 1;
        uint32_t sum = 0;
        for (int x = 0; x <= r && x < w; x++) sum += col[x];
        const uint8_t *p = img + (size_t)y * stride;
        uint8_t *row = map + (size_t)y * w;
        for (int x = 0; x < w; x++, p += pixel_step) {
            if (x > 0) {
                if (x + r < w) sum += col[x + r];
                if (x - r - 1 >= 0) sum -= col[x - r - 1];
            }
            const int x0 = x - r < 0 ? 0 : x - r, x1 = x + r >= w ? w - 1 : x + r;
            // pixel <= mean - C, without dividing per pixel
            row[x] = (uint32_t)(*p + ARUCO_THRESH_C) * (uint32_t)((x1 - x0 + 1) * rows) <= sum;
        }
    }
    // The contour tracer reads neighbours: keep a background frame around the image.
    memset(map, 0, (size_t)w);
    memset(map + (size_t)(h - 1) * w, 0, (size_t)w);
    for (int y = 0; y < h; y++) map[(size_t)y * w] = map[(size_t)y * w + w - 1] = 0;
}

// Moore-neighbour trace from a dark pixel whose left neighbour is background.
// Returns the point count, or 0 if the contour does not fit in cap.
static int trace(uint8_t *map, int w, int sx, int sy, pt_t *pts, int cap)
{
    int x = sx, y = sy, n = 0, first_dir = -1, search = 5;  // start just clockwise of the west neighbour
    for (;;) {
        if (n >= cap) return 0;
        pts[n].x = (int16_t)x;
        pts[n].y = (int16_t)y;
        n++;
        map[(size_t)y * w + x] = 2;
        int dir = -1;
        for (int i = 0; i < 8; i++) {
            int d = (search + i) & 7;
            if (map[(size_t)(y + DY[d]) * w + x + DX[d]]) {
                dir = d;
                break;
            }
        }
        if (dir < 0) return n;  // isolated pixel
        if (x == sx && y == sy) {
            if (first_dir == dir) return n - 1;  // back at the start, heading the same way
            if (first_dir < 0) first_dir = dir;
        }
        x += DX[dir];
        y += DY[dir];
        search = (dir + 5) & 7;
    }
}

static float line_dist2(pt_t a, pt_t b, pt_t p)
{
    const float dx = (float)(b.x - a.x), dy = (float)(b.y - a.y);
    const float px = (float)(p.x - a.x), py = (float)(p.y - a.y);
    const float len2 = dx * dx + dy * dy;
    if (len2 == 0.0f) return px * px + py * py;
    const float cross = dx * py - dy * px;
    return cross * cross / len2;
}

// Douglas-Peucker on the arc from index i to j (j may wrap past n). Adds split
// indices to verts in order; stops early once there are too many for a quad.
static void simplify(const pt_t *pts, int n, int i, int j, float eps2, int *verts, int *nv)
{
    if (*nv > 4 || j - i < 2) return;
    pt_t a = pts[i % n], b = pts[j % n];
    float best = eps2;
    int split = -1;
    for (int k = i + 1; k < j; k++) {
        float d = line_dist2(a, b, pts[k % n]);
        if (d > best) {
            best = d;
            split = k;
        }
    }
    if (split < 0) return;
    simplify(pts, n, i, split, eps2, verts, nv);
    if (*nv <= 4) verts[(*nv)++] = split % n;
    simplify(pts, n, split, j, eps2, verts, nv);
}

static int farthest(const pt_t *pts, int n, pt_t from)
{
    int best = 0, best_d = -1;
    for (int i = 0; i < n; i++) {
        int dx = pts[i].x - from.x, dy = pts[i].y - from.y, d = dx * dx + dy * dy;
        if (d > best_d) {
            best_d = d;
            best = i;
        }
    }
    return best;
}

// Least-squares line through the middle of the contour arc from i to j, as nx*x + ny*y = c.
static void fit_line(const pt_t *pts, int n, int i, int j, float line[3])
{
    const int skip = (j - i) / 8 + 1;
    float sx = 0, sy = 0, sxx = 0, syy = 0, sxy = 0;
    int m = 0;
    for (int k = i + skip; k <= j - skip; k++, m++) {
        const float x = pts[k % n].x, y = pts[k % n].y;
        sx += x;
        sy += y;
    }
    if (m < 2) {
        line[2] = NAN;
        return;
    }
    sx /= m;
    sy /= m;
    for (int k = i + skip; k <= j - skip; k++) {
        const float x = pts[k % n].x - sx, y = pts[k % n].y - sy;
        sxx += x * x;
        syy += y * y;
        sxy += x * y;
    }
    const float angle = 0.5f * atan2f(2 * sxy, sxx - syy);  // direction of the line
    line[0] = -sinf(angle);
    line[1] = cosf(angle);
    line[2] = line[0] * sx + line[1] * sy;
}

static uint8_t otsu(const uint8_t *v, int n)
{
    int hist[256] = {0};
    for (int i = 0; i < n; i++) hist[v[i]]++;
    int total = 0, weight_b = 0, best_t = 127;
    int64_t sum_b = 0;
    float best = -1;
    for (int t = 0; t < 256; t++) total += t * hist[t];
    for (int t = 0; t < 256; t++) {
        weight_b += hist[t];
        if (weight_b == 0) continue;
        const int weight_f = n - weight_b;
        if (weight_f == 0) break;
        sum_b += (int64_t)t * hist[t];
        const float diff = (float)sum_b / weight_b - (float)(total - sum_b) / weight_f;
        const float between = (float)weight_b * weight_f * diff * diff;
        if (between > best) {
            best = between;
            best_t = t;
        }
    }
    return (uint8_t)best_t;
}

// Reads the marker grid through the quad's perspective. Returns 0 on success.
static int read_bits(const uint8_t *img, int w, int h, int stride, int pixel_step,
                     const float q[4][2], uint64_t *bits, int *border_errors)
{
    enum { PER_CELL = 3, SAMPLES = CELLS * CELLS * PER_CELL * PER_CELL };
    static const float OFFSET[PER_CELL] = {0.3f, 0.5f, 0.7f};
    const float dx1 = q[1][0] - q[2][0], dx2 = q[3][0] - q[2][0], sx = q[0][0] - q[1][0] + q[2][0] - q[3][0];
    const float dy1 = q[1][1] - q[2][1], dy2 = q[3][1] - q[2][1], sy = q[0][1] - q[1][1] + q[2][1] - q[3][1];
    const float den = dx1 * dy2 - dx2 * dy1;
    if (den == 0.0f) return -1;
    const float g = (sx * dy2 - dx2 * sy) / den, k = (dx1 * sy - sx * dy1) / den;
    const float a = q[1][0] - q[0][0] + g * q[1][0], b = q[3][0] - q[0][0] + k * q[3][0], c = q[0][0];
    const float d = q[1][1] - q[0][1] + g * q[1][1], e = q[3][1] - q[0][1] + k * q[3][1], f = q[0][1];

    uint8_t v[SAMPLES];
    int n = 0, lo = 255, hi = 0;
    for (int r = 0; r < CELLS; r++) {
        for (int col = 0; col < CELLS; col++) {
            for (int i = 0; i < PER_CELL; i++) {
                for (int j = 0; j < PER_CELL; j++) {
                    const float u = (col + OFFSET[j]) / CELLS, t = (r + OFFSET[i]) / CELLS;
                    const float z = g * u + k * t + 1.0f;
                    const int x = (int)((a * u + b * t + c) / z + 0.5f);
                    const int y = (int)((d * u + e * t + f) / z + 0.5f);
                    if (x < 0 || y < 0 || x >= w || y >= h) return -1;
                    const uint8_t p = PX(x, y);
                    v[n++] = p;
                    if (p < lo) lo = p;
                    if (p > hi) hi = p;
                }
            }
        }
    }
    if (hi - lo < MIN_CONTRAST) return -1;
    const uint8_t t = otsu(v, SAMPLES);

    uint64_t word = 0;
    int errors = 0;
    n = 0;
    for (int r = 0; r < CELLS; r++) {
        for (int col = 0; col < CELLS; col++) {
            int white = 0;
            for (int i = 0; i < PER_CELL * PER_CELL; i++) white += v[n++] > t;
            const int is_white = white * 2 > PER_CELL * PER_CELL;
            if (r == 0 || col == 0 || r == CELLS - 1 || col == CELLS - 1) {
                errors += is_white;
            } else {
                word = (word << 1) | (uint64_t)is_white;
            }
        }
    }
    *bits = word;
    *border_errors = errors;
    return errors > MAX_BORDER_ERRORS ? -1 : 0;
}

static int try_quad(const uint8_t *img, int w, int h, int stride, int pixel_step,
                    const pt_t *pts, int n, aruco_marker_t *m)
{
    const float eps = POLY_RATE * n;
    const int i1 = farthest(pts, n, pts[0]), i0 = farthest(pts, n, pts[i1]);
    const int lo = i0 < i1 ? i0 : i1, hi = i0 < i1 ? i1 : i0;
    int verts[6], nv = 0;
    verts[nv++] = lo;
    simplify(pts, n, lo, hi, eps * eps, verts, &nv);
    if (nv <= 4) verts[nv++] = hi;
    simplify(pts, n, hi, lo + n, eps * eps, verts, &nv);
    if (nv != 4) return -1;

    float q[4][2];
    for (int i = 0; i < 4; i++) {
        q[i][0] = pts[verts[i]].x;
        q[i][1] = pts[verts[i]].y;
        if (q[i][0] < BORDER_MARGIN || q[i][1] < BORDER_MARGIN ||
            q[i][0] >= w - BORDER_MARGIN || q[i][1] >= h - BORDER_MARGIN) return -1;
    }
    for (int i = 0; i < 4; i++) {
        const float *a = q[i], *b = q[(i + 1) & 3], *c = q[(i + 2) & 3];
        const float ex = b[0] - a[0], ey = b[1] - a[1];
        const float cross = ex * (c[1] - b[1]) - ey * (c[0] - b[0]);
        // Outer contours run clockwise on screen, so this also rejects the inside of holes.
        if (cross <= 0 || ex * ex + ey * ey < MIN_SIDE * MIN_SIDE) return -1;
    }

    // Corners from where the fitted edges cross: steadier than single contour pixels.
    float lines[4][3];
    for (int i = 0; i < 4; i++) {
        const int a = verts[i], b = verts[(i + 1) & 3];
        fit_line(pts, n, a, b > a ? b : b + n, lines[i]);
    }
    for (int i = 0; i < 4; i++) {
        const float *l0 = lines[(i + 3) & 3], *l1 = lines[i];
        const float det = l0[0] * l1[1] - l1[0] * l0[1];
        if (isnan(l0[2]) || isnan(l1[2]) || fabsf(det) < 0.1f) continue;
        const float x = (l0[2] * l1[1] - l1[2] * l0[1]) / det, y = (l0[0] * l1[2] - l1[0] * l0[2]) / det;
        if (fabsf(x - q[i][0]) < 4 && fabsf(y - q[i][1]) < 4) {
            q[i][0] = x;
            q[i][1] = y;
        }
    }

    uint64_t bits;
    int border_errors;
    if (read_bits(img, w, h, stride, pixel_step, q, &bits, &border_errors) != 0) return -1;

    int best_h = MAX_HAMMING + 1, best_id = -1, best_rot = 0;
    for (int id = 0; id < ARUCO_DICT_COUNT; id++) {
        for (int r = 0; r < 4; r++) {
            const int dist = __builtin_popcountll(bits ^ ARUCO_DICT[id][r]);
            if (dist < best_h) {
                best_h = dist;
                best_id = id;
                best_rot = r;
            }
        }
    }
    if (best_id < 0) return -1;

    m->id = best_id + ARUCO_FIRST_ID;
    m->hamming = best_h;
    m->border_errors = border_errors;
    m->perimeter = 0;
    for (int i = 0; i < 4; i++) {
        m->corners[i][0] = q[(i + best_rot) & 3][0];
        m->corners[i][1] = q[(i + best_rot) & 3][1];
        m->perimeter += hypotf(q[(i + 1) & 3][0] - q[i][0], q[(i + 1) & 3][1] - q[i][1]);
    }
    return 0;
}

int aruco_detect(const uint8_t *img, int w, int h, int stride, int pixel_step,
                 void *work, aruco_marker_t *out, int max_out)
{
    uint8_t *map = work;
    uint32_t *col = (uint32_t *)((uintptr_t)(map + (size_t)w * h + 3) & ~(uintptr_t)3);
    pt_t *pts = (pt_t *)(col + w);
    const int cap = (int)ARUCO_MAX_CONTOUR(w, h);
    int found = 0;

    threshold(img, w, h, stride, pixel_step, map, col);

    for (int y = 1; y < h - 1; y++) {
        const uint8_t *row = map + (size_t)y * w;
        for (int x = 1; x < w - 1; x++) {
            if (row[x] != 1 || row[x - 1] != 0) continue;
            const int n = trace(map, w, x, y, pts, cap);
            if (n < MIN_PERIMETER) continue;

            aruco_marker_t m;
            if (try_quad(img, w, h, stride, pixel_step, pts, n, &m) != 0) continue;

            int dup = -1;
            for (int i = 0; i < found; i++) {
                if (out[i].id == m.id) dup = i;
            }
            if (dup >= 0) {
                if (m.perimeter > out[dup].perimeter) out[dup] = m;
            } else if (found < max_out) {
                out[found++] = m;
            }
        }
    }
    return found;
}
