#include "rover_map.h"
#include <string.h>

const int8_t rover_dx[4] = {0, 1, 0, -1};
const int8_t rover_dy[4] = {1, 0, -1, 0};

const rover_cell_t *rover_map_at(const rover_map_t *m, int x, int y) {
    for (unsigned i = 0; i < m->node_count; ++i)
        if (m->nodes[i].x == x && m->nodes[i].y == y) return &m->nodes[i];
    return NULL;
}
const rover_cell_t *rover_map_find(const rover_map_t *m, uint8_t id, int *x, int *y) {
    if (!id) return NULL;
    for (unsigned i = 0; i < m->node_count; ++i) {
        const rover_node_t *node = &m->nodes[i];
        if (node->marker_id != id) continue;
        if (x) *x = node->x;
        if (y) *y = node->y;
        return node;
    }
    return NULL;
}
static rover_node_t *find_mutable(rover_map_t *m, uint8_t id) {
    for (unsigned i = 0; i < m->node_count; ++i)
        if (m->nodes[i].marker_id == id) return &m->nodes[i];
    return NULL;
}
unsigned rover_map_rows(const rover_map_t *m) {
    return m->node_count ? (unsigned)(m->max_y - m->min_y + 1) : 0;
}
unsigned rover_map_columns(const rover_map_t *m) {
    return m->node_count ? (unsigned)(m->max_x - m->min_x + 1) : 0;
}
void rover_map_init(rover_map_t *m, uint8_t id) {
    memset(m, 0, sizeof(*m));
    if (id && rover_map_place(m, id, 0, 0)) {
        m->start = id;
        rover_map_visit(m, id);
    }
}
bool rover_map_place(rover_map_t *m, uint8_t id, int x, int y) {
    if (!id || x < -ROVER_COORD_LIMIT || x > ROVER_COORD_LIMIT ||
        y < -ROVER_COORD_LIMIT || y > ROVER_COORD_LIMIT) return false;
    const rover_node_t *existing = rover_map_find(m, id, NULL, NULL);
    if (existing) return existing->x == x && existing->y == y;
    if (m->node_count >= ROVER_MAX_NODES || rover_map_at(m, x, y)) return false;

    rover_node_t *node = &m->nodes[m->node_count];
    *node = (rover_node_t){.marker_id = id, .x = (int16_t)x, .y = (int16_t)y};
    for (unsigned d = 0; d < 4; ++d) {
        const rover_node_t *neighbor = rover_map_at(m, x + rover_dx[d], y + rover_dy[d]);
        if (neighbor) node->exits[d] = neighbor->exits[(d + 2u) % 4u];
    }
    if (!m->node_count) {
        m->min_x = m->max_x = (int16_t)x;
        m->min_y = m->max_y = (int16_t)y;
    } else {
        if (x < m->min_x) m->min_x = (int16_t)x;
        if (x > m->max_x) m->max_x = (int16_t)x;
        if (y < m->min_y) m->min_y = (int16_t)y;
        if (y > m->max_y) m->max_y = (int16_t)y;
    }
    ++m->node_count;
    ++m->revision;
    return true;
}
bool rover_map_visit(rover_map_t *m, uint8_t id) {
    rover_node_t *node = find_mutable(m, id);
    if (!node) return false;
    if (!node->visited) { node->visited = true; ++m->visited_count; ++m->revision; }
    if (m->current != id) { m->current = id; ++m->revision; }
    return true;
}
bool rover_map_exit(rover_map_t *m, uint8_t id, unsigned dir, uint8_t state) {
    rover_node_t *node = find_mutable(m, id);
    if (dir > 3 || state > 3 || !node) return false;
    /* A clear scan must not downgrade proof that an edge was traversed. */
    if (!(node->exits[dir] == MAP_EXPLORED && state == MAP_OPEN) && node->exits[dir] != state) {
        node->exits[dir] = state; ++m->revision;
    }
    const rover_node_t *other = rover_map_at(m, node->x + rover_dx[dir], node->y + rover_dy[dir]);
    if (other) {
        rover_node_t *mutable_other = find_mutable(m, other->marker_id);
        unsigned reverse = (dir + 2u) % 4u;
        if (mutable_other->exits[reverse] != node->exits[dir]) {
            mutable_other->exits[reverse] = node->exits[dir]; ++m->revision;
        }
    }
    return true;
}
bool rover_map_connect(rover_map_t *m, uint8_t a, uint8_t b, unsigned dir) {
    int ax, ay, bx, by;
    if (dir > 3 || !rover_map_find(m, a, &ax, &ay) || !rover_map_find(m, b, &bx, &by) ||
        bx != ax + rover_dx[dir] || by != ay + rover_dy[dir]) return false;
    return rover_map_exit(m, a, dir, MAP_EXPLORED);
}
