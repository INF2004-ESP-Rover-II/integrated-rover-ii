#include "dfs_explorer.h"
#include <string.h>

static uint8_t direction_bit(dfs_direction_t direction) {
    return (uint8_t)(1u << (unsigned)direction);
}

static dfs_direction_t opposite(dfs_direction_t direction) {
    return (dfs_direction_t)(((unsigned)direction + 2u) % 4u);
}

static bool valid_direction(dfs_direction_t direction) {
    return (unsigned)direction < 4u;
}

static bool valid_type(dfs_marker_type_t type) {
    return type == DFS_PLAIN || type == DFS_JUNCTION;
}

static uint16_t find_index(const dfs_explorer_t *dfs, uint32_t marker_id) {
    for (uint16_t i = 0; i < dfs->node_count; ++i) {
        if (dfs->nodes[i].marker_id == marker_id) return i;
    }
    return DFS_NO_NODE;
}

const dfs_node_t *dfs_find_marker(const dfs_explorer_t *dfs, uint32_t marker_id) {
    if (dfs == NULL) return NULL;
    uint16_t index = find_index(dfs, marker_id);
    return index == DFS_NO_NODE ? NULL : &dfs->nodes[index];
}

static void fail(dfs_explorer_t *dfs, dfs_fault_t fault) {
    dfs->state = DFS_FAULT;
    dfs->fault = fault;
}

static uint16_t add_marker(dfs_explorer_t *dfs, uint32_t marker_id, dfs_marker_type_t type) {
    if (dfs->node_count >= DFS_MAX_MARKERS) return DFS_NO_NODE;
    uint16_t index = dfs->node_count++;
    dfs_node_t *node = &dfs->nodes[index];
    memset(node, 0, sizeof(*node));
    node->marker_id = marker_id;
    node->type = type;
    for (unsigned i = 0; i < 4u; ++i) node->neighbor[i] = DFS_NO_NODE;
    return index;
}

dfs_config_t dfs_default_config(uint32_t session_id, uint16_t target_markers) {
    dfs_config_t config = {0};
    config.session_id = session_id;
    config.target_markers = target_markers;
    config.segment_timeout_ms = 10000u;
    config.priority[0] = 0; // front
    config.priority[1] = 3; // left
    config.priority[2] = 1; // right
    config.priority[3] = 2; // back, if an untried non-entry exit exists
    return config;
}

bool dfs_start(dfs_explorer_t *dfs, const dfs_config_t *config, uint32_t marker_id,
               dfs_marker_type_t type, dfs_direction_t heading) {
    if (dfs == NULL || config == NULL || !valid_type(type) || !valid_direction(heading) ||
        config->session_id == 0 || config->target_markers == 0 ||
        config->target_markers > DFS_MAX_MARKERS || config->segment_timeout_ms == 0) return false;
    uint8_t seen = 0;
    for (unsigned i = 0; i < 4u; ++i) {
        if (config->priority[i] > 3u) return false;
        uint8_t bit = (uint8_t)(1u << config->priority[i]);
        if (seen & bit) return false;
        seen |= bit;
    }
    // Copy first in case config points into dfs during an explicit reset.
    dfs_config_t copy = *config;
    memset(dfs, 0, sizeof(*dfs));
    dfs->config = copy;
    dfs->current = add_marker(dfs, marker_id, type);
    dfs->stack[0] = dfs->current;
    dfs->depth = 1;
    dfs->heading = heading;
    dfs->current_confirmed = true;
    dfs->next_segment_id = 1;
    dfs->return_node = DFS_NO_NODE;
    dfs->state = copy.target_markers == 1 ? DFS_COMPLETE : DFS_READY;
    return true;
}

static void block_exit(dfs_explorer_t *dfs, uint16_t from, dfs_direction_t direction) {
    dfs_node_t *node = &dfs->nodes[from];
    uint8_t bit = direction_bit(direction);
    node->known_exits |= bit;
    node->blocked_exits |= bit;
    node->open_exits &= (uint8_t)~bit;
    uint16_t neighbor = node->neighbor[direction];
    if (neighbor != DFS_NO_NODE) {
        dfs_node_t *other = &dfs->nodes[neighbor];
        bit = direction_bit(opposite(direction));
        other->known_exits |= bit;
        other->blocked_exits |= bit;
        other->open_exits &= (uint8_t)~bit;
    }
}

bool dfs_observe_exits(dfs_explorer_t *dfs, uint32_t marker_id,
                       uint8_t open_exits, uint8_t blocked_exits) {
    if (dfs == NULL || dfs->state != DFS_READY || !dfs->current_confirmed ||
        dfs->return_kind != DFS_MOVE_NONE || dfs->nodes[dfs->current].marker_id != marker_id ||
        (open_exits & blocked_exits) != 0 ||
        ((open_exits | blocked_exits) & 0xF0u) != 0) return false;
    dfs_node_t *node = &dfs->nodes[dfs->current];
    node->known_exits |= open_exits | blocked_exits;
    node->open_exits |= open_exits;
    for (unsigned i = 0; i < 4u; ++i) {
        if (blocked_exits & (1u << i)) block_exit(dfs, dfs->current, (dfs_direction_t)i);
    }
    node->open_exits &= (uint8_t)~node->blocked_exits;
    return true;
}

void dfs_tick(dfs_explorer_t *dfs, uint32_t now_ms) {
    if (dfs != NULL && dfs->state == DFS_MOVING &&
        (uint32_t)(now_ms - dfs->issued_ms) >= dfs->config.segment_timeout_ms) {
        fail(dfs, DFS_FAULT_TIMEOUT);
    }
}

static bool issue_move(dfs_explorer_t *dfs, dfs_move_kind_t kind,
                       dfs_direction_t direction, uint16_t target, uint32_t now_ms) {
    if (dfs->next_segment_id == 0) {
        fail(dfs, DFS_FAULT_SEQUENCE);
        return false;
    }
    dfs_segment_t segment = {0};
    segment.session_id = dfs->config.session_id;
    segment.segment_id = dfs->next_segment_id++;
    segment.from_marker = dfs->nodes[dfs->current].marker_id;
    segment.from_confirmed = dfs->current_confirmed;
    segment.target_known = target != DFS_NO_NODE;
    if (segment.target_known) segment.target_marker = dfs->nodes[target].marker_id;
    segment.direction = direction;
    segment.kind = kind;
    dfs->pending = segment;
    dfs->issued_ms = now_ms;
    dfs->current_confirmed = false;
    dfs->state = DFS_MOVING;
    return true;
}

dfs_action_t dfs_next_action(dfs_explorer_t *dfs, uint32_t now_ms) {
    dfs_action_t action = {0}; // WAIT until a state-specific action is available.
    if (dfs == NULL) return action;
    dfs_tick(dfs, now_ms);
    if (dfs->node_count > 0) action.marker_id = dfs->nodes[dfs->current].marker_id;
    if (dfs->state == DFS_READY) {
        if (dfs->return_kind != DFS_MOVE_NONE) {
            issue_move(dfs, dfs->return_kind, dfs->return_direction, dfs->return_node, now_ms);
        } else {
            dfs_node_t *node = &dfs->nodes[dfs->current];
            uint8_t allowed = node->type == DFS_PLAIN ? direction_bit(dfs->heading) : DFS_ALL_EXITS;
            uint8_t candidates = (uint8_t)(node->open_exits & allowed &
                (uint8_t)~node->blocked_exits & (uint8_t)~node->traversed_exits);
            for (unsigned i = 0; i < 4u; ++i) {
                dfs_direction_t direction = (dfs_direction_t)(
                    ((unsigned)dfs->heading + dfs->config.priority[i]) % 4u);
                if (candidates & direction_bit(direction)) {
                    issue_move(dfs, DFS_EXPLORE, direction, DFS_NO_NODE, now_ms);
                    break;
                }
            }
            if (dfs->state == DFS_READY) {
                if ((allowed & (uint8_t)~node->known_exits) != 0) {
                    action.kind = DFS_ACTION_SCAN;
                    return action; // Unknown is not an open path or a dead end.
                }
                if (dfs->depth == 1) {
                    dfs->state = DFS_EXHAUSTED;
                } else {
                    uint16_t parent = dfs->stack[dfs->depth - 2u];
                    bool found = false;
                    for (unsigned i = 0; i < 4u; ++i) {
                        uint8_t bit = (uint8_t)(1u << i);
                        if (node->neighbor[i] == parent && (node->open_exits & bit) &&
                            (node->blocked_exits & bit) == 0) {
                            issue_move(dfs, DFS_BACKTRACK, (dfs_direction_t)i, parent, now_ms);
                            found = true;
                            break;
                        }
                    }
                    if (!found) fail(dfs, DFS_FAULT_RETURN_BLOCKED);
                }
            }
        }
        if (dfs->state == DFS_MOVING) {
            action.kind = DFS_ACTION_MOVE;
            action.segment = dfs->pending;
            return action;
        }
    }
    if (dfs->state == DFS_COMPLETE) action.kind = DFS_ACTION_COMPLETE;
    else if (dfs->state == DFS_EXHAUSTED) action.kind = DFS_ACTION_EXHAUSTED;
    else if (dfs->state == DFS_PAUSED) action.kind = DFS_ACTION_PAUSED;
    else if (dfs->state == DFS_FAULT) {
        action.kind = DFS_ACTION_FAULT;
        action.fault = dfs->fault;
    }
    return action;
}

static bool matches(dfs_explorer_t *dfs, uint32_t session_id,
                     uint32_t segment_id, uint32_t now_ms) {
    if (dfs == NULL || dfs->state != DFS_MOVING || session_id != dfs->config.session_id ||
        segment_id != dfs->pending.segment_id) return false;
    dfs_tick(dfs, now_ms);
    return dfs->state == DFS_MOVING;
}

static bool link_nodes(dfs_explorer_t *dfs, uint16_t from, uint16_t to,
                       dfs_direction_t direction) {
    dfs_direction_t reverse = opposite(direction);
    dfs_node_t *source = &dfs->nodes[from];
    dfs_node_t *target = &dfs->nodes[to];
    if (from == to || (source->neighbor[direction] != DFS_NO_NODE &&
        source->neighbor[direction] != to) || (target->neighbor[reverse] != DFS_NO_NODE &&
        target->neighbor[reverse] != from)) return false;
    source->neighbor[direction] = to;
    target->neighbor[reverse] = from;
    source->known_exits |= direction_bit(direction);
    source->open_exits |= direction_bit(direction);
    source->traversed_exits |= direction_bit(direction);
    target->known_exits |= direction_bit(reverse);
    target->open_exits |= direction_bit(reverse);
    target->traversed_exits |= direction_bit(reverse);
    return true;
}

bool dfs_arrive(dfs_explorer_t *dfs, uint32_t session_id, uint32_t segment_id,
                uint32_t marker_id, dfs_marker_type_t type,
                dfs_direction_t heading, uint32_t now_ms) {
    if (!valid_type(type) || !valid_direction(heading) ||
        !matches(dfs, session_id, segment_id, now_ms)) return false;
    if (dfs->pending.target_known && dfs->pending.target_marker != marker_id) {
        fail(dfs, DFS_FAULT_TOPOLOGY);
        return false;
    }
    uint16_t from = dfs->current;
    uint16_t to = find_index(dfs, marker_id);
    bool is_new = to == DFS_NO_NODE;
    if (!is_new && dfs->nodes[to].type != type) {
        fail(dfs, DFS_FAULT_TOPOLOGY);
        return false;
    }
    if (is_new) {
        to = add_marker(dfs, marker_id, type);
        if (to == DFS_NO_NODE) {
            fail(dfs, DFS_FAULT_CAPACITY);
            return false;
        }
    }
    dfs_move_kind_t kind = dfs->pending.kind;
    if ((kind == DFS_EXPLORE || kind == DFS_BACKTRACK) &&
        !link_nodes(dfs, from, to, dfs->pending.direction)) {
        fail(dfs, DFS_FAULT_TOPOLOGY);
        return false;
    }
    dfs->return_kind = DFS_MOVE_NONE;
    if (kind == DFS_EXPLORE) {
        if (is_new) {
            if (dfs->depth >= DFS_MAX_MARKERS) {
                fail(dfs, DFS_FAULT_CAPACITY);
                return false;
            }
            dfs->stack[dfs->depth++] = to;
        } else {
            // A loop closure is real movement. Return physically to the source;
            // do not pop/teleport the DFS tree stack to the revisited marker.
            dfs->return_node = from;
            dfs->return_direction = opposite(dfs->pending.direction);
            dfs->return_kind = DFS_LOOP_RETURN;
        }
    } else if (kind == DFS_BACKTRACK) {
        dfs->depth--;
    }
    dfs->current = to;
    dfs->current_confirmed = true;
    dfs->heading = heading;
    dfs->state = dfs->node_count >= dfs->config.target_markers ? DFS_COMPLETE : DFS_READY;
    memset(&dfs->pending, 0, sizeof(dfs->pending));
    return true;
}

bool dfs_segment_blocked(dfs_explorer_t *dfs, uint32_t session_id,
                         uint32_t segment_id, uint32_t now_ms) {
    if (!matches(dfs, session_id, segment_id, now_ms)) return false;
    // A failed mid-corridor return is not an observation at the source marker.
    if (dfs->pending.kind != DFS_BLOCKED_RETURN) {
        block_exit(dfs, dfs->current, dfs->pending.direction);
    }
    if (dfs->pending.kind != DFS_EXPLORE) {
        fail(dfs, DFS_FAULT_RETURN_BLOCKED);
        return true;
    }
    dfs->return_node = dfs->current;
    dfs->return_direction = opposite(dfs->pending.direction);
    dfs->return_kind = DFS_BLOCKED_RETURN;
    dfs->state = DFS_READY;
    // current names the last source, not a confirmed current position.
    dfs->current_confirmed = false;
    memset(&dfs->pending, 0, sizeof(dfs->pending));
    return true;
}

bool dfs_pause(dfs_explorer_t *dfs) {
    if (dfs == NULL || (dfs->state != DFS_READY && dfs->state != DFS_MOVING)) return false;
    dfs->state = DFS_PAUSED;
    dfs->current_confirmed = false;
    dfs->return_kind = DFS_MOVE_NONE;
    memset(&dfs->pending, 0, sizeof(dfs->pending));
    return true;
}

bool dfs_resume(dfs_explorer_t *dfs, uint32_t marker_id, dfs_direction_t heading) {
    if (dfs == NULL || dfs->state != DFS_PAUSED || !valid_direction(heading) ||
        dfs->depth == 0 || dfs->nodes[dfs->stack[dfs->depth - 1u]].marker_id != marker_id) return false;
    dfs->current = dfs->stack[dfs->depth - 1u];
    dfs->current_confirmed = true;
    dfs->heading = heading;
    dfs->state = DFS_READY;
    return true;
}
