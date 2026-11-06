/* rover_map.c - see rover_map.h. Port of rover_mapping/mapping/graph.py. */
#include "rover_map.h"

#include <string.h>

static const int8_t DX[4] = { 0, 1, 0, -1 };   /* N E S W */
static const int8_t DY[4] = { 1, 0, -1, 0 };

uint8_t map_opposite(uint8_t d) { return (uint8_t)((d + 2u) & 3u); }

int map_turn_steps(int a, int b)
{
    int diff = ((a - b) % 4 + 4) % 4;
    return diff < 4 - diff ? diff : 4 - diff;
}

void map_init(rover_map_t *m)
{
    memset(m, 0, sizeof *m);
    memset(m->index_of, MAP_NONE, sizeof m->index_of);
    m->start_id = -1;
    m->current = MAP_NONE;
}

uint8_t map_index(const rover_map_t *m, uint8_t marker_id)
{
    return marker_id < MAP_ID_LIMIT ? m->index_of[marker_id] : MAP_NONE;
}

uint8_t map_count(const rover_map_t *m) { return m->count; }

static uint8_t add_marker(rover_map_t *m, uint8_t id, int x, int y)
{
    uint8_t i = m->count;
    map_marker_t *mk = &m->markers[i];
    mk->id = id;
    mk->x = (int8_t)x;
    mk->y = (int8_t)y;
    for (int d = 0; d < 4; d++) {
        mk->exits[d] = EXIT_UNCHECKED;
        mk->nbr[d] = MAP_NONE;
        mk->blocked[d] = 0;
    }
    m->index_of[id] = i;
    m->count++;
    return i;
}

bool map_start(rover_map_t *m, uint8_t marker_id)
{
    if (marker_id >= MAP_ID_LIMIT) return false;
    map_init(m);
    uint8_t i = add_marker(m, marker_id, 0, 0);
    m->start_id = marker_id;
    m->current = i;
    return true;
}

static uint8_t marker_at(const rover_map_t *m, int x, int y)
{
    for (uint8_t i = 0; i < m->count; i++)
        if (m->markers[i].x == x && m->markers[i].y == y) return i;
    return MAP_NONE;
}

static void link_idx(rover_map_t *m, uint8_t a, uint8_t d, uint8_t b)
{
    m->markers[a].nbr[d] = b;
    m->markers[b].nbr[map_opposite(d)] = a;
    m->markers[a].exits[d] = EXIT_EXPLORED;
    m->markers[b].exits[map_opposite(d)] = EXIT_EXPLORED;
}

map_reached_t map_marker_reached(rover_map_t *m, uint8_t marker_id, uint8_t heading)
{
    if (m->current == MAP_NONE || heading > 3 || marker_id >= MAP_ID_LIMIT) return MAP_REACHED_ERROR;
    const map_marker_t *cur = &m->markers[m->current];
    int px = cur->x + DX[heading], py = cur->y + DY[heading];
    uint8_t known = m->index_of[marker_id];
    bool is_new = known == MAP_NONE;
    uint8_t idx;
    if (is_new) {
        uint8_t clash = marker_at(m, px, py);
        if (clash != MAP_NONE) {
            m->inconsistencies = (uint8_t)(m->inconsistencies < 255 ? m->inconsistencies + 1 : 255);
            m->last_clash_id = marker_id;
            return MAP_REACHED_CONFLICT;
        }
        if (m->count >= MAP_MAX_MARKERS) return MAP_REACHED_ERROR;
        idx = add_marker(m, marker_id, px, py);
    } else {
        idx = known;
        if (m->markers[idx].x != px || m->markers[idx].y != py) {
            m->inconsistencies = (uint8_t)(m->inconsistencies < 255 ? m->inconsistencies + 1 : 255);
            m->last_clash_id = marker_id;
            return MAP_REACHED_CONFLICT;
        }
    }
    link_idx(m, m->current, heading, idx);
    m->current = idx;
    return is_new ? MAP_REACHED_NEW : MAP_REACHED_KNOWN;
}

void map_set_exit_status(rover_map_t *m, uint8_t marker_id, uint8_t d, exit_status_t s)
{
    uint8_t i = map_index(m, marker_id);
    if (i == MAP_NONE || d > 3) return;
    /* never downgrade EXPLORED (a travelled exit stays explored), except to BLOCKED */
    if (m->markers[i].exits[d] == EXIT_EXPLORED && s != EXIT_BLOCKED) return;
    m->markers[i].exits[d] = (uint8_t)s;
}

void map_apply_exit_report(rover_map_t *m, uint8_t marker_id, const uint8_t exits[4])
{
    for (uint8_t d = 0; d < 4; d++) map_set_exit_status(m, marker_id, d, (exit_status_t)exits[d]);
}

uint8_t map_neighbour_at(const rover_map_t *m, uint8_t idx, uint8_t d)
{
    if (idx >= m->count || d > 3) return MAP_NONE;
    uint8_t n = m->markers[idx].nbr[d];
    if (n != MAP_NONE) return n;
    return marker_at(m, m->markers[idx].x + DX[d], m->markers[idx].y + DY[d]);
}

void map_mark_blocked(rover_map_t *m, uint8_t marker_id, uint8_t d)
{
    uint8_t i = map_index(m, marker_id);
    if (i == MAP_NONE || d > 3) return;
    m->markers[i].exits[d] = EXIT_BLOCKED;
    uint8_t other = map_neighbour_at(m, i, d);
    if (other != MAP_NONE) {
        m->markers[i].blocked[d] = 1;
        m->markers[other].blocked[map_opposite(d)] = 1;
        m->markers[other].exits[map_opposite(d)] = EXIT_BLOCKED;
    }
}

void map_link(rover_map_t *m, uint8_t a_id, uint8_t d, uint8_t b_id)
{
    uint8_t a = map_index(m, a_id), b = map_index(m, b_id);
    if (a == MAP_NONE || b == MAP_NONE || d > 3) return;
    link_idx(m, a, d, b);
}

int map_is_probable(const rover_map_t *m, uint8_t idx, uint8_t d)
{
    if (idx >= m->count || d > 3) return -1;
    if (m->markers[idx].nbr[d] != MAP_NONE) return -1;            /* already a driven link */
    uint8_t b = map_neighbour_at(m, idx, d);
    if (b == MAP_NONE || m->markers[idx].blocked[d]) return -1;
    uint8_t sa = m->markers[idx].exits[d], sb = m->markers[b].exits[map_opposite(d)];
    if (sa == EXIT_BLOCKED || sb == EXIT_BLOCKED) return -1;
    return (sa == EXIT_UNEXPLORED || sb == EXIT_UNEXPLORED) ? 1 : 0;
}

int map_direction_between(const rover_map_t *m, uint8_t a_id, uint8_t b_id)
{
    uint8_t a = map_index(m, a_id), b = map_index(m, b_id);
    if (a == MAP_NONE || b == MAP_NONE) return -1;
    for (int d = 0; d < 4; d++)
        if (m->markers[a].nbr[d] == b) return d;
    for (int d = 0; d < 4; d++)                                    /* probable edge: use the grid geometry */
        if (m->markers[a].x + DX[d] == m->markers[b].x && m->markers[a].y + DY[d] == m->markers[b].y) return d;
    return -1;
}

uint8_t map_usable_neighbours(const rover_map_t *m, uint8_t idx, bool use_probable, map_edge_t out[4])
{
    uint8_t n = 0;
    if (idx >= m->count) return 0;
    for (uint8_t d = 0; d < 4; d++) {
        uint8_t nb = m->markers[idx].nbr[d];
        if (nb != MAP_NONE && !m->markers[idx].blocked[d]) {
            out[n].dir = d; out[n].idx = nb; out[n].probable = false; n++;
        } else if (use_probable && map_is_probable(m, idx, d) >= 0) {
            out[n].dir = d; out[n].idx = map_neighbour_at(m, idx, d); out[n].probable = true; n++;
        }
    }
    for (uint8_t i = 1; i < n; i++) {                              /* insertion sort by neighbour marker ID */
        map_edge_t e = out[i];
        int j = i - 1;
        while (j >= 0 && m->markers[out[j].idx].id > m->markers[e.idx].id) { out[j + 1] = out[j]; j--; }
        out[j + 1] = e;
    }
    return n;
}
