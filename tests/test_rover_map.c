#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "rover_map.h"

static void output(const rover_map_t *map)
{
    char json[ROVER_MAP_JSON_CAPACITY];
    size_t length = rover_map_format(map, "0123456789abcdef0123456789abcdef", json, sizeof(json));
    assert(length && length == strlen(json) && length < 8192 - 32);
    puts(json);
}

int main(void)
{
    rover_map_t map;
    rover_map_init(&map, 0x12345678);
    assert(!map.count && !map.current_node && !rover_map_columns(&map));
    assert(rover_map_start(&map, 7) == MAP_OK);
    assert(map.current_node == 7 && map.nodes[0].visited);
    assert(rover_map_columns(&map) == 1 && rover_map_rows(&map) == 1);
    output(&map);
    assert(rover_map_add(&map, 12, 1, 0, false) == MAP_OK);
    assert(rover_map_columns(&map) == 2 && rover_map_rows(&map) == 1);
    assert(rover_map_add(&map, 3, 1, 1, false) == MAP_OK);
    assert(rover_map_columns(&map) == 2 && rover_map_rows(&map) == 2);
    assert(rover_map_add(&map, 9, -1, 0, false) == MAP_OK);
    assert(rover_map_columns(&map) == 3 && rover_map_rows(&map) == 2);
    output(&map);
    assert(rover_map_find_cell(&map, -1, 0)->marker_id == 9);
    assert(rover_map_find_id(&map, 12)->x == 1);
    uint32_t revision = map.revision;
    assert(rover_map_add(&map, 12, 1, 0, false) == MAP_OK && map.revision == revision);
    assert(rover_map_add(&map, 12, 2, 0, true) == MAP_ID_CONFLICT);
    assert(rover_map_add(&map, 20, 1, 0, true) == MAP_CELL_OCCUPIED);
    assert(rover_map_add(&map, 0, 2, 0, false) == MAP_INVALID_ID);
    assert(map.revision == revision && map.count == 4);
    assert(rover_map_arrive(&map, 12) == MAP_OK && map.current_node == 12);
    assert(rover_map_find_id(&map, 12)->visited);
    assert(rover_map_set_exit(&map, 12, MAP_WEST, MAP_EXIT_TRAVERSED) == MAP_OK);
    assert(rover_map_set_exit(&map, 12, (map_direction_t)4, MAP_EXIT_OPEN) == MAP_INVALID_EXIT);
    assert(rover_map_set_exit(&map, 12, MAP_EAST, (map_exit_t)4) == MAP_INVALID_EXIT);
    rover_map_depart(&map);
    assert(map.current_node == MAP_NODE_UNKNOWN && rover_map_find_id(&map, 12)->visited);
    revision = map.revision;
    rover_map_depart(&map);
    assert(map.revision == revision);
    assert(rover_map_arrive(&map, 99) == MAP_NOT_FOUND);
    assert(rover_map_add(&map, 20, 0, -1, false) == MAP_OK);
    assert(map.min_y == -1 && rover_map_rows(&map) == 3);
    output(&map);

    // Capacity is independent of dimensions: fill an 8x8 discovered layout.
    rover_map_init(&map, 0x12345678);
    for (unsigned id = 1; id <= ROVER_MAP_CAPACITY; ++id)
        assert(rover_map_add(&map, (uint8_t)id, (int16_t)((id - 1) % 8), (int16_t)((id - 1) / 8), false) == MAP_OK);
    assert(map.count == ROVER_MAP_CAPACITY && rover_map_columns(&map) == 8 && rover_map_rows(&map) == 8);
    revision = map.revision;
    assert(rover_map_add(&map, 250, -1, -1, false) == MAP_FULL);
    assert(map.revision == revision && !rover_map_find_id(&map, 250));
    output(&map);
    char small[20];
    assert(!rover_map_format(&map, "", small, sizeof(small)) && !small[0]);
    assert(!rover_map_format(&map, "injection\"", small, sizeof(small)));

    // Dimensions are 32-bit even though coordinates are signed 16-bit.
    rover_map_init(&map, 0x12345678);
    assert(rover_map_add(&map, 254, INT16_MIN, INT16_MIN, false) == MAP_OK);
    assert(rover_map_add(&map, 255, INT16_MAX, INT16_MAX, false) == MAP_OK);
    assert(rover_map_columns(&map) == 65536 && rover_map_rows(&map) == 65536);
    output(&map);
    assert(rover_map_start(&map, 255) == MAP_OK && map.count == 1 && map.current_node == 255);
    assert(map.min_x == 0 && map.min_y == 0 && rover_map_columns(&map) == 1);
    printf("Map tests passed; map storage=%u bytes, JSON budget=%u bytes\n",
           (unsigned)sizeof(map), (unsigned)ROVER_MAP_JSON_CAPACITY);
    return 0;
}
