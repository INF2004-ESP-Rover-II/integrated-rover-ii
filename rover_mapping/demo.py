"""Run a full scenario in the simulator: Phase 1 (map), then Phase 2 (route and re-plan).
    python demo.py            # 4 x 6 random maze, as in the 240 x 360 cm arena at 60 cm pitch
"""
from mapping import events as ev
from mapping.controller import MappingController, State
from mapping.config import DIR_NAMES
from mapping.planner import plan_route
from sim.maze import Maze
from sim.sim_rover import SimRover, run_phase1, run_phase2


def main(seed: int = 3, n: int = 24):
    maze = Maze.random(4, 6, extra_links=5, seed=seed)
    ctrl = MappingController()
    rover = SimRover(maze, start_id=0)

    ctrl.start(n, 0)
    run_phase1(ctrl, rover)
    print(f"Phase 1 finished in state {ctrl.state.value}; {ctrl.graph.count()} markers found")
    print(ctrl.graph.to_ascii())
    print("unexplored exits left:", ctrl.graph.unexplored_exits())
    ctrl.drain()
    if ctrl.state != State.MAP_COMPLETE:
        return

    target = max(ctrl.graph.markers, key=lambda i: (abs(ctrl.graph.markers[i].pos[0]) +
                                                    abs(ctrl.graph.markers[i].pos[1])))
    path = plan_route(ctrl.graph, ctrl.graph.current, target, ctrl.heading)
    print(f"\nRover at marker {ctrl.graph.current}; target {target}; planned route {path}")
    if len(path) > 2:
        maze.block(path[1], path[2])
        print(f"Obstacle placed on the edge {path[1]} -> {path[2]}")
    ctrl.set_target(target)
    sent = run_phase2(ctrl, rover)
    for s in sent:
        print(f"  segment seq={s.seq} -> marker {s.target_marker} heading {DIR_NAMES[s.heading]}"
              f"{' (last)' if s.last else ''}")
    print("result:", ctrl.sent_log[-1])
    print("state path:", " > ".join(s.value for s in ctrl.history[-12:]))


if __name__ == "__main__":
    main()
