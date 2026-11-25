/**
 * @file aruco_detect.c
 * @brief Aruco detect firmware module.
 */
#include "aruco_detect.h"

#include <math.h>
#include <string.h>

#include "aruco_dict.h"

/* Fixed dimensions and iteration limits used by this module. */
#define CONTOUR_NEIGHBOUR_COUNT 8
#define GRAYSCALE_LEVEL_COUNT 256
#define MARKER_CORNER_COUNT 4

// Limits follow OpenCV's DetectorParameters defaults where one exists.
#ifndef ARUCO_THRESH_WINDOW
#define ARUCO_THRESH_WINDOW 23 // adaptive threshold window, pixels (odd)
#endif
#ifndef ARUCO_THRESH_C
#define ARUCO_THRESH_C 7 // adaptiveThreshConstant
#endif
#define POLY_RATE 0.03f     // polygonalApproxAccuracyRate
#define MIN_PERIMETER 40    // pixels of contour
#define MIN_SIDE 6.0f       // pixels
#define BORDER_MARGIN 3     // minDistanceToBorder
#define MAX_BORDER_ERRORS 9 // maxErroneousBitsInBorderRate 0.35 of 28 cells
#define MAX_HAMMING 3       // errorCorrectionRate 0.6 of 5 for DICT_6X6_250
#define MIN_CONTRAST 20 // grey levels between the darkest and lightest samples
#define CELLS 8         // 6x6 bits plus a 1-cell border

typedef struct
{
    int16_t x;
    int16_t y;
} pt_t;

static const int8_t g_dx[8] = {1,  1,  0, -1,
                               -1, -1, 0, 1}; // clockwise on screen, y down
static const int8_t g_dy[8] = {0, 1, 1, 1, 0, -1, -1, -1};

// map[i]: 0 background, 1 dark, 2 dark and already on a traced contour.
static void
threshold (const uint8_t * p_img, int width, int height, int stride,
           int pixel_step, uint8_t * p_map, uint32_t * p_col)
{
    const int result = ARUCO_THRESH_WINDOW / 2;
    memset(p_col, 0, (size_t) width * sizeof(uint32_t));
    for (int y_coordinate = 0; y_coordinate <= result && y_coordinate < height;
         y_coordinate++)
    {
        const uint8_t * p_pointer = p_img + (size_t) y_coordinate * stride;
        for (int x_coordinate = 0; x_coordinate < width;
             x_coordinate++, p_pointer += pixel_step)
        {
            p_col[x_coordinate] += *p_pointer;
        }
    }
    for (int y_coordinate = 0; y_coordinate < height; y_coordinate++)
    {
        if (y_coordinate > 0)
        {
            if (y_coordinate + result < height)
            {

                const uint8_t * p_pointer =
                    p_img + (size_t) (y_coordinate + result) * stride;

                for (int x_coordinate = 0; x_coordinate < width;
                     x_coordinate++, p_pointer += pixel_step)
                {
                    p_col[x_coordinate] += *p_pointer;
                }
            }
            if (y_coordinate - result - 1 >= 0)
            {

                const uint8_t * p_pointer =
                    p_img + (size_t) (y_coordinate - result - 1) * stride;

                for (int x_coordinate = 0; x_coordinate < width;
                     x_coordinate++, p_pointer += pixel_step)
                {
                    p_col[x_coordinate] -= *p_pointer;
                }
            }
        }

        const int origin_y =
            y_coordinate - result < 0 ? 0 : y_coordinate - result;

        const int y1 = y_coordinate + result >= height ? height - 1
                                                       : y_coordinate + result;

        const int rows = y1 - origin_y + 1;
        uint32_t  sum  = 0;
        for (int x_coordinate = 0;
             x_coordinate <= result && x_coordinate < width; x_coordinate++)
        {
            sum += p_col[x_coordinate];
        }
        const uint8_t * p_pointer = p_img + (size_t) y_coordinate * stride;
        uint8_t *       p_row     = p_map + (size_t) y_coordinate * width;
        for (int x_coordinate = 0; x_coordinate < width;
             x_coordinate++, p_pointer += pixel_step)
        {
            if (x_coordinate > 0)
            {
                if (x_coordinate + result < width)
                {
                    sum += p_col[x_coordinate + result];
                }
                if (x_coordinate - result - 1 >= 0)
                {
                    sum -= p_col[x_coordinate - result - 1];
                }
            }

            const int origin_x =
                x_coordinate - result < 0 ? 0 : x_coordinate - result;

            const int x1 = x_coordinate + result >= width
                               ? width - 1
                               : x_coordinate + result;

            // pixel <= mean - C, without dividing per pixel

            p_row[x_coordinate] = (uint32_t) (*p_pointer + ARUCO_THRESH_C) *
                                      (uint32_t) ((x1 - origin_x + 1) * rows) <=
                                  sum;
        }
    }
    // The contour tracer reads neighbours: keep a background frame around the
    // image.
    memset(p_map, 0, (size_t) width);
    memset(p_map + (size_t) (height - 1) * width, 0, (size_t) width);
    for (int y_coordinate = 0; y_coordinate < height; y_coordinate++)
    {

        p_map[(size_t) y_coordinate * width] =
            p_map[(size_t) y_coordinate * width + width - 1] = 0;
    }
}

// Moore-neighbour trace from a dark pixel whose left neighbour is background.
// Returns the point count, or 0 if the contour does not fit in cap.
static int
trace (uint8_t * p_map, int width, int sx, int sy, pt_t * p_pts, int cap)
{
    int x_coordinate = sx;
    int y_coordinate = sy;
    int count        = 0;
    int first_dir    = -1;
    int search       = 5; // start just clockwise of the west neighbour
    for (;;)
    {
        if (count >= cap)
        {
            return 0;
        }
        p_pts[count].x = (int16_t) x_coordinate;
        p_pts[count].y = (int16_t) y_coordinate;
        count++;
        p_map[(size_t) y_coordinate * width + x_coordinate] = 2;
        int dir                                             = -1;
        for (int index = 0; index < CONTOUR_NEIGHBOUR_COUNT; index++)
        {
            int data = (uint32_t) ((search + index)) & (uint32_t) (7);
            if (p_map[(size_t) (y_coordinate + g_dy[data]) * width +
                      x_coordinate + g_dx[data]])
            {
                dir = data;
                break;
            }
        }
        if (dir < 0)
        {
            return count; // isolated pixel
        }
        if (x_coordinate == sx && y_coordinate == sy)
        {
            if (first_dir == dir)
            {
                return count - 1; // back at the start, height the same way
            }
            if (first_dir < 0)
            {
                first_dir = dir;
            }
        }
        x_coordinate += g_dx[dir];
        y_coordinate += g_dy[dir];
        search = (uint32_t) ((dir + 5)) & (uint32_t) (7);
    }
}

static float
line_dist2 (pt_t first, pt_t second, pt_t pointer)
{
    const float delta_x = (float) (second.x - first.x);
    const float delta_y = (float) (second.y - first.y);
    const float pixel_x = (float) (pointer.x - first.x);
    const float pixel_y = (float) (pointer.y - first.y);
    const float len2    = delta_x * delta_x + delta_y * delta_y;
    if (0.0f == len2)
    {
        return pixel_x * pixel_x + pixel_y * pixel_y;
    }
    const float cross = delta_x * pixel_y - delta_y * pixel_x;
    return cross * cross / len2;
}

// Douglas-Peucker on the arc from index i to j (j may wrap past n). Adds split
// indices to verts in order; stops early once there are too many for a quad.
static void
simplify (const pt_t * p_pts, int count, int index, int inner_index, float eps2,
          int * p_verts, int * p_nv)
{
    if (*p_nv > 4 || inner_index - index < 2)
    {
        return;
    }
    pt_t  first  = p_pts[index % count];
    pt_t  second = p_pts[inner_index % count];
    float best   = eps2;
    int   split  = -1;
    for (int item_index = index + 1; item_index < inner_index; item_index++)
    {
        float data = line_dist2(first, second, p_pts[item_index % count]);
        if (data > best)
        {
            best  = data;
            split = item_index;
        }
    }
    if (split < 0)
    {
        return;
    }
    simplify(p_pts, count, index, split, eps2, p_verts, p_nv);
    if (*p_nv <= 4)
    {
        p_verts[(*p_nv)++] = split % count;
    }
    simplify(p_pts, count, split, inner_index, eps2, p_verts, p_nv);
}

static int
farthest (const pt_t * p_pts, int count, pt_t from)
{
    int best   = 0;
    int best_d = -1;
    for (int index = 0; index < count; index++)
    {
        int delta_x = p_pts[index].x - from.x;
        int delta_y = p_pts[index].y - from.y;
        int data    = delta_x * delta_x + delta_y * delta_y;
        if (data > best_d)
        {
            best_d = data;
            best   = index;
        }
    }
    return best;
}

// Least-squares line through the middle of the contour arc from i to j, as nx*x
// + ny*y = c.
static void
fit_line (const pt_t * p_pts, int count, int index, int inner_index,
          float line[3])
{
    const int skip    = (inner_index - index) / 8 + 1;
    float     sx      = 0;
    float     sy      = 0;
    float     sxx     = 0;
    float     syy     = 0;
    float     sxy     = 0;
    int       value_m = 0;
    for (int item_index = index + skip; item_index <= inner_index - skip;
         item_index++, value_m++)
    {
        const float x_coordinate = p_pts[item_index % count].x;
        const float y_coordinate = p_pts[item_index % count].y;
        sx += x_coordinate;
        sy += y_coordinate;
    }
    if (value_m < 2)
    {
        line[2] = NAN;
        return;
    }
    sx /= value_m;
    sy /= value_m;
    for (int item_index = index + skip; item_index <= inner_index - skip;
         item_index++)
    {
        const float x_coordinate = p_pts[item_index % count].x - sx;
        const float y_coordinate = p_pts[item_index % count].y - sy;
        sxx += x_coordinate * x_coordinate;
        syy += y_coordinate * y_coordinate;
        sxy += x_coordinate * y_coordinate;
    }

    const float angle = 0.5f * atan2f(2 * sxy, sxx - syy);
    // direction of the line
    line[0] = -sinf(angle);
    line[1] = cosf(angle);
    line[2] = line[0] * sx + line[1] * sy;
}

static uint8_t
otsu (const uint8_t * p_value, int count)
{
    int hist[256] = {0};
    for (int index = 0; index < count; index++)
    {
        hist[p_value[index]]++;
    }
    int     total    = 0;
    int     weight_b = 0;
    int     best_t   = 127;
    int64_t sum_b    = 0;
    float   best     = -1;
    for (int time_value = 0; time_value < GRAYSCALE_LEVEL_COUNT; time_value++)
    {
        total += time_value * hist[time_value];
    }
    for (int time_value = 0; time_value < GRAYSCALE_LEVEL_COUNT; time_value++)
    {
        weight_b += hist[time_value];
        if (0 == weight_b)
        {
            continue;
        }
        const int weight_f = count - weight_b;
        if (0 == weight_f)
        {
            break;
        }
        sum_b += (int64_t) time_value * hist[time_value];

        const float diff =
            (float) sum_b / weight_b - (float) (total - sum_b) / weight_f;

        const float between = (float) weight_b * weight_f * diff * diff;
        if (between > best)
        {
            best   = between;
            best_t = time_value;
        }
    }
    return (uint8_t) best_t;
}

// Reads the marker grid through the quad's perspective. Returns 0 on success.
static int
read_bits (const uint8_t * p_img, int width, int height, int stride,
           int pixel_step, const float value_q[4][2], uint64_t * p_bits,
           int * p_border_errors)
{

    enum
    {
        PER_CELL = 3,
        SAMPLES  = CELLS * CELLS * PER_CELL * PER_CELL
    };

    static const float sample_offsets[PER_CELL] = {0.3f, 0.5f, 0.7f};
    const float        dx1                      = value_q[1][0] - value_q[2][0];
    const float        dx2                      = value_q[3][0] - value_q[2][0];

    const float sx =
        value_q[0][0] - value_q[1][0] + value_q[2][0] - value_q[3][0];

    const float dy1 = value_q[1][1] - value_q[2][1];
    const float dy2 = value_q[3][1] - value_q[2][1];

    const float sy =
        value_q[0][1] - value_q[1][1] + value_q[2][1] - value_q[3][1];

    const float den = dx1 * dy2 - dx2 * dy1;
    if (0.0f == den)
    {
        return -1;
    }
    const float value_g    = (sx * dy2 - dx2 * sy) / den;
    const float item_index = (dx1 * sy - sx * dy1) / den;
    const float first = value_q[1][0] - value_q[0][0] + value_g * value_q[1][0];

    const float second =
        value_q[3][0] - value_q[0][0] + item_index * value_q[3][0];

    const float value_c = value_q[0][0];
    const float data = value_q[1][1] - value_q[0][1] + value_g * value_q[1][1];

    const float error_value =
        value_q[3][1] - value_q[0][1] + item_index * value_q[3][1];

    const float factor = value_q[0][1];

    uint8_t value[SAMPLES];
    int     count       = 0;
    int     lower_bound = 255;
    int     upper_bound = 0;
    for (int result = 0; result < CELLS; result++)
    {
        for (int col = 0; col < CELLS; col++)
        {
            for (int index = 0; index < PER_CELL; index++)
            {
                for (int inner_index = 0; inner_index < PER_CELL; inner_index++)
                {

                    const float coordinate_u =
                        (col + sample_offsets[inner_index]) / CELLS;

                    const float time_value =
                        (result + sample_offsets[index]) / CELLS;

                    const float z_coordinate =
                        value_g * coordinate_u + item_index * time_value + 1.0f;

                    const int x_coordinate =
                        (int) ((first * coordinate_u + second * time_value +
                                value_c) /
                                   z_coordinate +
                               0.5f);

                    const int y_coordinate =
                        (int) ((data * coordinate_u + error_value * time_value +
                                factor) /
                                   z_coordinate +
                               0.5f);

                    if (x_coordinate < 0 || y_coordinate < 0 ||
                        x_coordinate >= width || y_coordinate >= height)
                    {
                        return -1;
                    }

                    const uint8_t pointer =
                        p_img[(size_t) y_coordinate * stride +
                              (size_t) x_coordinate * pixel_step];

                    value[count++] = pointer;
                    if (pointer < lower_bound)
                    {
                        lower_bound = pointer;
                    }
                    if (pointer > upper_bound)
                    {
                        upper_bound = pointer;
                    }
                }
            }
        }
    }
    if (upper_bound - lower_bound < MIN_CONTRAST)
    {
        return -1;
    }
    const uint8_t time_value = otsu(value, SAMPLES);

    uint64_t word   = 0;
    int      errors = 0;
    count           = 0;
    for (int result = 0; result < CELLS; result++)
    {
        for (int col = 0; col < CELLS; col++)
        {
            int white = 0;
            for (int index = 0; index < PER_CELL * PER_CELL; index++)
            {
                white += value[count++] > time_value;
            }
            const int is_white = white * 2 > PER_CELL * PER_CELL;
            if (0 == result || 0 == col || CELLS - 1 == result ||
                CELLS - 1 == col)
            {
                errors += is_white;
            }
            else
            {
                word = (word << (uint32_t) (1)) | (uint64_t) is_white;
            }
        }
    }
    *p_bits          = word;
    *p_border_errors = errors;
    return errors > MAX_BORDER_ERRORS ? -1 : 0;
}

static int
try_quad (const uint8_t * p_img, int width, int height, int stride,
          int pixel_step, const pt_t * p_pts, int count,
          aruco_marker_t * p_value_m)
{
    const float eps         = POLY_RATE * count;
    const int   i1          = farthest(p_pts, count, p_pts[0]);
    const int   i0          = farthest(p_pts, count, p_pts[i1]);
    const int   lower_bound = i0 < i1 ? i0 : i1;
    const int   upper_bound = i0 < i1 ? i1 : i0;
    int         verts[6];
    int         nv = 0;
    verts[nv++]    = lower_bound;
    simplify(p_pts, count, lower_bound, upper_bound, eps * eps, verts, &nv);
    if (nv <= 4)
    {
        verts[nv++] = upper_bound;
    }
    simplify(p_pts, count, upper_bound, lower_bound + count, eps * eps, verts,
             &nv);
    if (nv != 4)
    {
        return -1;
    }

    float value_q[4][2];
    for (int index = 0; index < MARKER_CORNER_COUNT; index++)
    {
        value_q[index][0] = p_pts[verts[index]].x;
        value_q[index][1] = p_pts[verts[index]].y;
        if (value_q[index][0] < BORDER_MARGIN ||
            value_q[index][1] < BORDER_MARGIN ||
            value_q[index][0] >= width - BORDER_MARGIN ||
            value_q[index][1] >= height - BORDER_MARGIN)
        {
            return -1;
        }
    }
    for (int index = 0; index < MARKER_CORNER_COUNT; index++)
    {
        const float * p_first = value_q[index];

        const float * p_second =
            value_q[(uint32_t) ((index + 1)) & (uint32_t) (3)];

        const float * p_value_c =
            value_q[(uint32_t) ((index + 2)) & (uint32_t) (3)];

        const float ex = p_second[0] - p_first[0];
        const float ey = p_second[1] - p_first[1];

        const float cross = ex * (p_value_c[1] - p_second[1]) -
                            ey * (p_value_c[0] - p_second[0]);

        // Outer contours run clockwise on screen, so this also rejects the
        // inside of holes.
        if (cross <= 0 || ex * ex + ey * ey < MIN_SIDE * MIN_SIDE)
        {
            return -1;
        }
    }

    // Corners from where the fitted edges cross: steadier than single contour
    // pixels.
    float lines[4][3];
    for (int index = 0; index < MARKER_CORNER_COUNT; index++)
    {
        const int first  = verts[index];
        const int second = verts[(uint32_t) ((index + 1)) & (uint32_t) (3)];
        fit_line(p_pts, count, first, second > first ? second : second + count,
                 lines[index]);
    }
    for (int index = 0; index < MARKER_CORNER_COUNT; index++)
    {
        const float * p_l0 = lines[(uint32_t) ((index + 3)) & (uint32_t) (3)];
        const float * p_l1 = lines[index];
        const float   det  = p_l0[0] * p_l1[1] - p_l1[0] * p_l0[1];
        if (isnan(p_l0[2]) || isnan(p_l1[2]) || fabsf(det) < 0.1f)
        {
            continue;
        }

        const float x_coordinate =
            (p_l0[2] * p_l1[1] - p_l1[2] * p_l0[1]) / det;

        const float y_coordinate =
            (p_l0[0] * p_l1[2] - p_l1[0] * p_l0[2]) / det;

        if (fabsf(x_coordinate - value_q[index][0]) < 4 &&
            fabsf(y_coordinate - value_q[index][1]) < 4)
        {
            value_q[index][0] = x_coordinate;
            value_q[index][1] = y_coordinate;
        }
    }

    uint64_t bits          = 0;
    int      border_errors = 0;
    if (read_bits(p_img, width, height, stride, pixel_step, value_q, &bits,
                  &border_errors) != 0)
    {
        return -1;
    }

    int best_h   = MAX_HAMMING + 1;
    int best_id  = -1;
    int best_rot = 0;
    for (int identifier = 0; identifier < ARUCO_DICT_COUNT; identifier++)
    {
        for (int result = 0; result < MARKER_CORNER_COUNT; result++)
        {

            const int dist =
                __builtin_popcountll(bits ^ g_aruco_dict[identifier][result]);

            if (dist < best_h)
            {
                best_h   = dist;
                best_id  = identifier;
                best_rot = result;
            }
        }
    }
    if (best_id < 0)
    {
        return -1;
    }

    p_value_m->id            = best_id + ARUCO_FIRST_ID;
    p_value_m->hamming       = best_h;
    p_value_m->border_errors = border_errors;
    p_value_m->perimeter     = 0;
    for (int index = 0; index < MARKER_CORNER_COUNT; index++)
    {

        p_value_m->corners[index][0] =
            value_q[(uint32_t) ((index + best_rot)) & (uint32_t) (3)][0];

        p_value_m->corners[index][1] =
            value_q[(uint32_t) ((index + best_rot)) & (uint32_t) (3)][1];

        p_value_m->perimeter +=
            hypotf(value_q[(uint32_t) ((index + 1)) & (uint32_t) (3)][0] -
                       value_q[index][0],
                   value_q[(uint32_t) ((index + 1)) & (uint32_t) (3)][1] -
                       value_q[index][1]);
    }
    return 0;
}

int
aruco_detect (const uint8_t * p_img, int width, int height, int stride,
              int pixel_step, void * p_work, aruco_marker_t * p_out,
              int max_out)
{
    uint8_t * p_map = p_work;

    uint32_t * p_col =
        (uint32_t *) ((uintptr_t) (p_map + (size_t) width * height + 3) &
                      ~(uintptr_t) 3);

    pt_t *    p_pts = (pt_t *) (p_col + width);
    const int cap   = (int) ARUCO_MAX_CONTOUR(width, height);
    int       found = 0;

    threshold(p_img, width, height, stride, pixel_step, p_map, p_col);

    for (int y_coordinate = 1; y_coordinate < height - 1; y_coordinate++)
    {
        const uint8_t * p_row = p_map + (size_t) y_coordinate * width;
        for (int x_coordinate = 1; x_coordinate < width - 1; x_coordinate++)
        {
            if (p_row[x_coordinate] != 1 || p_row[x_coordinate - 1] != 0)
            {
                continue;
            }

            const int count =
                trace(p_map, width, x_coordinate, y_coordinate, p_pts, cap);

            if (count < MIN_PERIMETER)
            {
                continue;
            }

            aruco_marker_t value_m;
            if (try_quad(p_img, width, height, stride, pixel_step, p_pts, count,
                         &value_m) != 0)
            {
                continue;
            }

            int dup = -1;
            for (int index = 0; index < found; index++)
            {
                if (p_out[index].id == value_m.id)
                {
                    dup = index;
                }
            }
            if (dup >= 0)
            {
                if (value_m.perimeter > p_out[dup].perimeter)
                {
                    p_out[dup] = value_m;
                }
            }
            else if (found < max_out)
            {
                p_out[found++] = value_m;
            }
        }
    }
    return found;
}

/* End of file. */
