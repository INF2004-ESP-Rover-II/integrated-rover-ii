/**
 * @file test_dfs_backtrack.c
 * @brief Host test: the route planner used for backtracking while a DFS is
 *        still exploring, on the three 6 x 6 DFS test maps.
 *
 * A small host-side depth-first search explores each maze on a real
 * rover_map_t, using the same order as FR-19 (forward, right, left, back,
 * relative to the heading). Whenever it has to go back to the most recent
 * stack node that still has an unexplored exit, two things are compared:
 *
 *   - run A retraces the DFS stack one node at a time, as the onboard DFS
 *     does today. At every step the planner's next hop must be the same
 *     single hop, and for a longer return the planner's route must never
 *     be longer than the stack path.
 *   - run B drives the planner's route (route_planner_next_hop, one hop at
 *     a time, re-planned after every arrival) and must still explore every
 *     cell.
 *
 * NOTE: this is a model of the DFS, not Buddy 1's onboard_dfs.c. It scans all
 * four exits on arrival and treats a loop-closing exit as driven there and
 * back (2 hops, the same in both runs).
 */

#include <inttypes.h>
#include <stdio.h>
#include <string.h>
#include "rover_map.h"
#include "route_planner.h"
#include "test_support.h"

/* ---- Constants ---------------------------------------------------------- */

/** The mazes are 6 x 6 cells of 600 mm. */
#define TEST_SIZE (6u)

/** Cells in a maze. */
#define TEST_CELLS (TEST_SIZE * TEST_SIZE)

/** Directions on the grid. */
#define TEST_DIRECTIONS (4u)

/** Longest wall list. */
#define TEST_MAX_WALLS (8u)

/** Cell the rover starts in (A1). */
#define TEST_START_ID (1u)

/** Steps after which a run is stopped as stuck. */
#define TEST_STEP_LIMIT (1000u)

/** Most dead ends listed per maze. */
#define TEST_MAX_DEAD_ENDS (3u)

/* ---- Types -------------------------------------------------------------- */

/** Wall direction: between columns (vertical) or between rows. */
typedef enum
{
    TEST_WALL_VERTICAL = 0,
    TEST_WALL_HORIZONTAL = 1
} test_wall_kind_t;

/**
 * One run of wall segments. For a vertical wall, boundary is the column to
 * the left of the wall (A|B is 1) and first/last are rows. For a horizontal
 * wall, boundary is the row above the wall ("under row 1" is 1) and
 * first/last are columns. Rows and columns are numbered from 1.
 */
typedef struct
{
    test_wall_kind_t kind;
    uint32_t boundary;
    uint32_t first;
    uint32_t last;
} test_wall_t;

/** A maze: its walls and what the drawing says about it. */
typedef struct
{
    const char * p_name;
    uint32_t wall_count;
    test_wall_t walls[TEST_MAX_WALLS];
    uint32_t inside_segments;
    uint32_t dead_end_count;
    uint8_t dead_ends[TEST_MAX_DEAD_ENDS];
} test_maze_t;

/** Totals from one simulated exploration. */
typedef struct
{
    uint32_t hops;           /**< Every hop driven. */
    uint32_t returns;        /**< Returns to an earlier stack node. */
    uint32_t shortcuts;      /**< Returns where the planner was shorter. */
    uint32_t single_steps;   /**< One-step backtracks checked. */
    uint32_t stack_hops;     /**< Hops the stack path would have taken. */
    uint32_t planner_hops;   /**< Hops the planner path takes. */
    uint32_t visited;        /**< Cells visited at the end. */
    bool legal;              /**< Every driven hop was a verified edge. */
} test_stats_t;

/** The exploring rover in the model. */
typedef struct
{
    rover_map_t map;
    uint8_t stack[ROVER_MAX_NODES];
    uint32_t depth;
    uint32_t heading;
    bool use_planner;
} test_rover_t;

/* ---- Static data -------------------------------------------------------- */

/** The three DFS test maps, read from the drawing. */
static const test_maze_t g_mazes[] =
{
    {
        "current layout", 8u,
        {
            {TEST_WALL_VERTICAL, 1u, 2u, 5u},
            {TEST_WALL_VERTICAL, 2u, 2u, 2u},
            {TEST_WALL_VERTICAL, 2u, 4u, 6u},
            {TEST_WALL_VERTICAL, 3u, 5u, 6u},
            {TEST_WALL_VERTICAL, 4u, 5u, 5u},
            {TEST_WALL_VERTICAL, 5u, 2u, 5u},
            {TEST_WALL_HORIZONTAL, 1u, 2u, 5u},
            {TEST_WALL_HORIZONTAL, 4u, 4u, 4u}
        },
        20u, 3u, {8u, 33u, 28u}
    },
    {
        "S layout", 2u,
        {
            {TEST_WALL_VERTICAL, 2u, 1u, 5u},
            {TEST_WALL_VERTICAL, 4u, 2u, 6u},
            {TEST_WALL_VERTICAL, 0u, 0u, 0u},
            {TEST_WALL_VERTICAL, 0u, 0u, 0u},
            {TEST_WALL_VERTICAL, 0u, 0u, 0u},
            {TEST_WALL_VERTICAL, 0u, 0u, 0u},
            {TEST_WALL_VERTICAL, 0u, 0u, 0u},
            {TEST_WALL_VERTICAL, 0u, 0u, 0u}
        },
        10u, 0u, {0u, 0u, 0u}
    },
    {
        "loop and dead ends", 5u,
        {
            {TEST_WALL_VERTICAL, 2u, 1u, 4u},
            {TEST_WALL_VERTICAL, 3u, 4u, 5u},
            {TEST_WALL_VERTICAL, 4u, 3u, 6u},
            {TEST_WALL_VERTICAL, 5u, 2u, 5u},
            {TEST_WALL_HORIZONTAL, 4u, 5u, 5u},
            {TEST_WALL_VERTICAL, 0u, 0u, 0u},
            {TEST_WALL_VERTICAL, 0u, 0u, 0u},
            {TEST_WALL_VERTICAL, 0u, 0u, 0u}
        },
        15u, 2u, {23u, 29u, 0u}
    }
};

/** Number of mazes. */
#define TEST_MAZE_COUNT (3u)

/* ---- Private prototypes ------------------------------------------------- */

static uint8_t test_cell_id(uint32_t row, uint32_t col);
static uint32_t test_row_of(uint8_t marker_id);
static uint32_t test_col_of(uint8_t marker_id);
static uint8_t test_neighbour_id(uint8_t marker_id, uint32_t direction);
static int32_t test_heading(const test_rover_t * p_rover);
static bool test_wall_between(const test_maze_t * p_maze,
                              uint32_t row,
                              uint32_t col,
                              uint32_t direction);
static bool test_cell_exists(int32_t row, int32_t col);
static uint32_t test_open_count(const test_maze_t * p_maze,
                                uint32_t row,
                                uint32_t col);
static uint32_t test_count_segments(const test_maze_t * p_maze);
static uint32_t test_reachable(const test_maze_t * p_maze);
static void test_check_maze_reading(const test_maze_t * p_maze);
static void test_scan(const test_maze_t * p_maze, test_rover_t * p_rover);
static bool test_has_open_exit(const rover_node_t * p_node);
static uint32_t test_find_stack_target(const test_rover_t * p_rover);
static bool test_move(test_rover_t * p_rover,
                      uint8_t next_id,
                      uint32_t direction,
                      test_stats_t * p_stats);
static bool test_step_back(test_rover_t * p_rover, test_stats_t * p_stats);
static void test_return_to(test_rover_t * p_rover,
                           uint32_t target_index,
                           test_stats_t * p_stats);
static bool test_explore_forward(const test_maze_t * p_maze,
                                 test_rover_t * p_rover,
                                 test_stats_t * p_stats);
static void test_explore(const test_maze_t * p_maze,
                         bool use_planner,
                         test_stats_t * p_stats);
static void test_dead_end_backtrack(const test_maze_t * p_maze);
static void test_all_mazes(void);

/* ---- Public functions --------------------------------------------------- */

int main(void)
{
    test_all_mazes();

    return test_finish("test_dfs_backtrack");
}

/* ---- Maze model --------------------------------------------------------- */

/** Marker ID of a cell: A1 is 1, B1 is 2, ... F6 is 36. */
static uint8_t test_cell_id(uint32_t row, uint32_t col)
{
    /* row and col are 0 to 5, so the ID is 1 to 36 and fits in uint8_t. */
    return (uint8_t)((row * TEST_SIZE) + col + 1u);
}

/** Row (0 to 5) of a marker ID. */
static uint32_t test_row_of(uint8_t marker_id)
{
    uint32_t cell = marker_id;

    return (cell - 1u) / TEST_SIZE;
}

/** Column (0 to 5) of a marker ID. */
static uint32_t test_col_of(uint8_t marker_id)
{
    uint32_t cell = marker_id;

    return (cell - 1u) % TEST_SIZE;
}

/** ID of the cell next to a marker in a direction (the cell must exist). */
static uint8_t test_neighbour_id(uint8_t marker_id, uint32_t direction)
{
    /* The row and column are 0 to 5, so they fit in int32_t. */
    int32_t row = (int32_t)test_row_of(marker_id)
                  - (int32_t)rover_dy[direction];
    int32_t col = (int32_t)test_col_of(marker_id)
                  + (int32_t)rover_dx[direction];

    /* Rows and columns are 0 to 5 here, so the ID is 1 to 36. */
    return (uint8_t)((row * (int32_t)TEST_SIZE) + col + 1);
}

/** Heading as the signed type the planner takes. */
static int32_t test_heading(const test_rover_t * p_rover)
{
    /* The heading is 0 to 3, so it fits in int32_t. */
    return (int32_t)p_rover->heading;
}

/** Is there a cell at this row and column? */
static bool test_cell_exists(int32_t row, int32_t col)
{
    /* TEST_SIZE is 6, so it fits in int32_t. */
    return (row >= 0) && (col >= 0) && (row < (int32_t)TEST_SIZE)
           && (col < (int32_t)TEST_SIZE);
}

/**
 * Is there a wall (or the outer edge) on one side of a cell? row and col
 * count from 0, direction is MAP_NORTH, MAP_EAST, MAP_SOUTH or MAP_WEST.
 */
static bool test_wall_between(const test_maze_t * p_maze,
                              uint32_t row,
                              uint32_t col,
                              uint32_t direction)
{
    bool wall = false;
    uint32_t index = 0u;
    uint32_t boundary = 0u;
    uint32_t along = 0u;
    test_wall_kind_t kind = TEST_WALL_VERTICAL;
    /* row and col are 0 to 5, so they fit in int32_t. */
    int32_t next_row = (int32_t)row - (int32_t)rover_dy[direction];
    int32_t next_col = (int32_t)col + (int32_t)rover_dx[direction];

    if (!test_cell_exists(next_row, next_col))
    {
        wall = true;
    }

    if ((MAP_EAST == direction) || (MAP_WEST == direction))
    {
        kind = TEST_WALL_VERTICAL;
        /* The column on the left of the wall, counted from 1. */
        if (MAP_EAST == direction)
        {
            boundary = col + 1u;
        }
        else
        {
            boundary = col;
        }

        along = row + 1u;
    }
    else
    {
        kind = TEST_WALL_HORIZONTAL;
        /* The row above the wall, counted from 1. */
        if (MAP_SOUTH == direction)
        {
            boundary = row + 1u;
        }
        else
        {
            boundary = row;
        }

        along = col + 1u;
    }

    for (index = 0u; index < p_maze->wall_count; index++)
    {
        if ((kind == p_maze->walls[index].kind)
            && (boundary == p_maze->walls[index].boundary)
            && (along >= p_maze->walls[index].first)
            && (along <= p_maze->walls[index].last))
        {
            wall = true;
        }
    }

    return wall;
}

/** Number of open sides of a cell. */
static uint32_t test_open_count(const test_maze_t * p_maze,
                                uint32_t row,
                                uint32_t col)
{
    uint32_t count = 0u;
    uint32_t direction = 0u;

    for (direction = 0u; direction < TEST_DIRECTIONS; direction++)
    {
        if (!test_wall_between(p_maze, row, col, direction))
        {
            count++;
        }
    }

    return count;
}

/** Inside wall segments (each 600 mm), counted from the wall list. */
static uint32_t test_count_segments(const test_maze_t * p_maze)
{
    uint32_t total = 0u;
    uint32_t index = 0u;

    for (index = 0u; index < p_maze->wall_count; index++)
    {
        total += (p_maze->walls[index].last - p_maze->walls[index].first)
                 + 1u;
    }

    return total;
}

/** Cells reachable from A1, found by a flood fill over open sides. */
static uint32_t test_reachable(const test_maze_t * p_maze)
{
    bool seen[TEST_CELLS];
    uint32_t queue[TEST_CELLS];
    uint32_t head = 0u;
    uint32_t tail = 0u;
    uint32_t cell = 0u;
    uint32_t direction = 0u;
    uint32_t row = 0u;
    uint32_t col = 0u;

    (void)memset(seen, 0, sizeof(seen));
    seen[0] = true;
    queue[tail] = 0u;
    tail++;

    while (head < tail)
    {
        cell = queue[head];
        head++;
        row = cell / TEST_SIZE;
        col = cell % TEST_SIZE;

        for (direction = 0u; direction < TEST_DIRECTIONS; direction++)
        {
            if (!test_wall_between(p_maze, row, col, direction))
            {
                cell = test_neighbour_id(test_cell_id(row, col), direction);
                cell -= 1u;

                if (!seen[cell])
                {
                    seen[cell] = true;
                    queue[tail] = cell;
                    tail++;
                }
            }
        }
    }

    return tail;
}

/** The wall lists must reproduce what the drawing says about each maze. */
static void test_check_maze_reading(const test_maze_t * p_maze)
{
    uint32_t row = 0u;
    uint32_t col = 0u;
    uint32_t dead_ends = 0u;
    uint32_t listed = 0u;
    bool is_listed = false;

    test_check(p_maze->inside_segments == test_count_segments(p_maze),
               __LINE__);
    test_check(TEST_CELLS == test_reachable(p_maze), __LINE__);

    for (row = 0u; row < TEST_SIZE; row++)
    {
        for (col = 0u; col < TEST_SIZE; col++)
        {
            if ((1u == test_open_count(p_maze, row, col))
                && (0u != (row + col)))
            {
                dead_ends++;
                is_listed = false;

                for (listed = 0u; listed < p_maze->dead_end_count; listed++)
                {
                    if (test_cell_id(row, col) == p_maze->dead_ends[listed])
                    {
                        is_listed = true;
                    }
                }

                test_check(is_listed, __LINE__);
            }
        }
    }

    test_check(p_maze->dead_end_count == dead_ends, __LINE__);
}

/* ---- Simulated exploring rover ------------------------------------------ */

/** Does the node have an exit that has not been driven or blocked? */
static bool test_has_open_exit(const rover_node_t * p_node)
{
    uint32_t direction = 0u;
    bool open = false;

    for (direction = 0u; direction < TEST_DIRECTIONS; direction++)
    {
        if (MAP_OPEN == p_node->exits[direction])
        {
            open = true;
        }
    }

    return open;
}

/** Scan all four exits of the current node, like the DFS scan step. */
static void test_scan(const test_maze_t * p_maze, test_rover_t * p_rover)
{
    const rover_node_t * p_node = NULL;
    uint32_t row = 0u;
    uint32_t col = 0u;
    uint32_t direction = 0u;
    uint8_t state = MAP_UNCHECKED;

    p_node = rover_map_find(&p_rover->map, p_rover->map.current, NULL, NULL);
    row = test_row_of(p_node->marker_id);
    col = test_col_of(p_node->marker_id);

    for (direction = 0u; direction < TEST_DIRECTIONS; direction++)
    {
        if (MAP_UNCHECKED == p_node->exits[direction])
        {
            if (test_wall_between(p_maze, row, col, direction))
            {
                state = MAP_BLOCKED;
            }
            else
            {
                state = MAP_OPEN;
            }

            test_check(rover_map_exit(&p_rover->map, p_node->marker_id,
                                      direction, state), __LINE__);
        }
    }
}

/** Index in the stack of the most recent node below the top with an open exit,
 *  or ROVER_MAX_NODES when there is none. */
static uint32_t test_find_stack_target(const test_rover_t * p_rover)
{
    uint32_t found = ROVER_MAX_NODES;
    uint32_t index = p_rover->depth - 1u;
    const rover_node_t * p_node = NULL;

    while ((ROVER_MAX_NODES == found) && (index > 0u))
    {
        index--;
        p_node = rover_map_find(&p_rover->map, p_rover->stack[index], NULL,
                                NULL);

        if (test_has_open_exit(p_node))
        {
            found = index;
        }
    }

    return found;
}

/**
 * Drive one hop to an adjacent, verified node. Checks that the edge is
 * explored at both ends and that the node is where the direction says.
 */
static bool test_move(test_rover_t * p_rover,
                      uint8_t next_id,
                      uint32_t direction,
                      test_stats_t * p_stats)
{
    const rover_node_t * p_from = NULL;
    const rover_node_t * p_to = NULL;
    bool legal = false;

    p_from = rover_map_find(&p_rover->map, p_rover->map.current, NULL, NULL);
    p_to = rover_map_find(&p_rover->map, next_id, NULL, NULL);

    if ((NULL != p_from) && (NULL != p_to) && (direction < TEST_DIRECTIONS))
    {
        legal = (MAP_EXPLORED == p_from->exits[direction])
                && (p_to->x == (p_from->x + rover_dx[direction]))
                && (p_to->y == (p_from->y + rover_dy[direction]))
                && (MAP_EXPLORED
                    == p_to->exits[(direction + 2u) % TEST_DIRECTIONS]);
    }

    if (legal)
    {
        test_check(rover_map_visit(&p_rover->map, next_id), __LINE__);
        p_rover->heading = direction;
        p_stats->hops++;
    }
    else
    {
        p_stats->legal = false;
    }

    return legal;
}

/** Run A: go back one node along the stack, checking the planner agrees. */
static bool test_step_back(test_rover_t * p_rover, test_stats_t * p_stats)
{
    route_planner_step_t step;
    route_planner_status_t status;
    uint8_t previous = p_rover->stack[p_rover->depth - 2u];
    bool moved = false;

    status = route_planner_next_hop(&p_rover->map, p_rover->map.current,
                                    previous, test_heading(p_rover),
                                    &step);
    test_check(ROUTE_PLANNER_OK == status, __LINE__);
    test_check((previous == step.next_id) && (1u == step.hops_to_go),
               __LINE__);
    p_stats->single_steps++;
    moved = test_move(p_rover, step.next_id, step.direction, p_stats);

    if (moved)
    {
        p_rover->depth--;
    }

    return moved;
}

/**
 * Go back to the stack node at target_index. Run A retraces the stack and
 * compares lengths; run B drives the planner's route and trims the stack.
 */
static void test_return_to(test_rover_t * p_rover,
                           uint32_t target_index,
                           test_stats_t * p_stats)
{
    route_planner_route_t route;
    route_planner_step_t step;
    uint8_t target_id = p_rover->stack[target_index];
    uint32_t stack_hops = (p_rover->depth - 1u) - target_index;
    uint32_t guard = 0u;
    bool moved = true;

    test_check(ROUTE_PLANNER_OK == route_planner_plan(
                   &p_rover->map, p_rover->map.current, target_id,
                   test_heading(p_rover), &route), __LINE__);
    p_stats->returns++;
    p_stats->stack_hops += stack_hops;
    p_stats->planner_hops += route.hops;

    if (route.hops < stack_hops)
    {
        p_stats->shortcuts++;
    }

    if (!p_rover->use_planner)
    {
        test_check(route.hops <= stack_hops, __LINE__);

        while (((p_rover->depth - 1u) > target_index)
               && (guard < ROVER_MAX_NODES))
        {
            moved = test_step_back(p_rover, p_stats);
            test_check(moved, __LINE__);
            guard++;
        }
    }
    else
    {
        while ((target_id != p_rover->map.current) && (guard < ROVER_MAX_NODES)
               && (moved))
        {
            test_check(ROUTE_PLANNER_OK == route_planner_next_hop(
                           &p_rover->map, p_rover->map.current, target_id,
                           test_heading(p_rover), &step), __LINE__);
            moved = test_move(p_rover, step.next_id, step.direction,
                              p_stats);
            guard++;
        }

        /* The target is on the stack, so the stack is cut back to it. */
        p_rover->depth = target_index + 1u;
    }

    test_check(target_id == p_rover->map.current, __LINE__);
}

/**
 * Take the first open exit in the order forward, right, left, back. Returns
 * true if the rover moved or closed a loop, false if no exit is open.
 */
static bool test_explore_forward(const test_maze_t * p_maze,
                                 test_rover_t * p_rover,
                                 test_stats_t * p_stats)
{
    const rover_node_t * p_node = NULL;
    const rover_node_t * p_known = NULL;
    uint32_t order[TEST_DIRECTIONS];
    uint32_t pick = 0u;
    uint32_t direction = 0u;
    bool found = false;
    uint8_t next_id = 0u;
    int next_x = 0;
    int next_y = 0;

    order[0] = p_rover->heading;
    order[1] = (p_rover->heading + 1u) % TEST_DIRECTIONS;
    order[2] = (p_rover->heading + 3u) % TEST_DIRECTIONS;
    order[3] = (p_rover->heading + 2u) % TEST_DIRECTIONS;
    p_node = rover_map_find(&p_rover->map, p_rover->map.current, NULL, NULL);

    for (pick = 0u; (pick < TEST_DIRECTIONS) && (!found); pick++)
    {
        direction = order[pick];

        if (MAP_OPEN == p_node->exits[direction])
        {
            found = true;
        }
    }

    if (found)
    {
        /* The loop above stopped one past the exit it picked. */
        direction = order[pick - 1u];
        next_id = test_neighbour_id(p_node->marker_id, direction);
        next_x = p_node->x + rover_dx[direction];
        next_y = p_node->y + rover_dy[direction];
        p_known = rover_map_at(&p_rover->map, next_x, next_y);

        if (NULL == p_known)
        {
            test_check(rover_map_place(&p_rover->map, next_id, next_x,
                                       next_y), __LINE__);
        }

        test_check(rover_map_connect(&p_rover->map, p_node->marker_id,
                                     next_id, direction), __LINE__);

        if (NULL != p_known)
        {
            /* Loop closure: drive there and back, the same in both runs. */
            p_stats->hops += 2u;
        }
        else
        {
            test_check(test_move(p_rover, next_id, direction, p_stats),
                       __LINE__);
            p_rover->stack[p_rover->depth] = next_id;
            p_rover->depth++;
            test_scan(p_maze, p_rover);
        }
    }

    return found;
}

/** Explore a whole maze in run A (stack) or run B (planner). */
static void test_explore(const test_maze_t * p_maze,
                         bool use_planner,
                         test_stats_t * p_stats)
{
    test_rover_t * p_rover = NULL;
    uint32_t target = 0u;
    uint32_t steps = 0u;
    bool exploring = true;

    /* Static: the model is larger than is comfortable on the stack. */
    static test_rover_t rover;

    p_rover = &rover;
    (void)memset(p_rover, 0, sizeof(*p_rover));
    (void)memset(p_stats, 0, sizeof(*p_stats));
    p_stats->legal = true;
    p_rover->use_planner = use_planner;
    p_rover->heading = MAP_EAST;
    rover_map_init(&p_rover->map, TEST_START_ID);
    p_rover->stack[0] = TEST_START_ID;
    p_rover->depth = 1u;
    test_scan(p_maze, p_rover);

    while ((exploring) && (steps < TEST_STEP_LIMIT))
    {
        steps++;

        if (test_explore_forward(p_maze, p_rover, p_stats))
        {
            /* Moved on, or closed a loop and stayed. */
        }
        else
        {
            target = test_find_stack_target(p_rover);

            if (ROVER_MAX_NODES == target)
            {
                exploring = false;
            }
            else
            {
                test_return_to(p_rover, target, p_stats);
            }
        }
    }

    test_check(steps < TEST_STEP_LIMIT, __LINE__);
    p_stats->visited = p_rover->map.visited_count;
}

/** Backtracking from each dead end is the single hop to the previous node. */
static void test_dead_end_backtrack(const test_maze_t * p_maze)
{
    uint32_t index = 0u;
    test_stats_t stats;

    /* Run A already checks every one-step backtrack, dead ends included. */
    test_explore(p_maze, false, &stats);
    test_check(stats.single_steps >= p_maze->dead_end_count, __LINE__);

    for (index = 0u; index < p_maze->dead_end_count; index++)
    {
        test_check(0u != p_maze->dead_ends[index], __LINE__);
    }
}

/** Run every maze in both modes and print the totals. */
static void test_all_mazes(void)
{
    uint32_t index = 0u;
    test_stats_t stack_run;
    test_stats_t planner_run;

    for (index = 0u; index < TEST_MAZE_COUNT; index++)
    {
        test_check_maze_reading(&g_mazes[index]);

        test_explore(&g_mazes[index], false, &stack_run);
        test_explore(&g_mazes[index], true, &planner_run);
        test_dead_end_backtrack(&g_mazes[index]);

        test_check(TEST_CELLS == stack_run.visited, __LINE__);
        test_check(TEST_CELLS == planner_run.visited, __LINE__);
        test_check(stack_run.legal && planner_run.legal, __LINE__);
        test_check(stack_run.planner_hops <= stack_run.stack_hops, __LINE__);

        (void)printf("%-20s returns=%" PRIu32 " shortcuts=%" PRIu32
                     " back-hops stack=%" PRIu32 " planner=%" PRIu32
                     " | total hops stack-run=%" PRIu32
                     " planner-run=%" PRIu32 "\n",
                     g_mazes[index].p_name, stack_run.returns,
                     stack_run.shortcuts, stack_run.stack_hops,
                     stack_run.planner_hops, stack_run.hops,
                     planner_run.hops);
    }
}

/* end of file test_dfs_backtrack.c */
