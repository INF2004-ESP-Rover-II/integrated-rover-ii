"""Generate test vectors for the C port (firmware/robo_pico/src/{rover_map,route_planner,map_fsm}.c).

Drives the PYTHON MappingController through random scenarios and records, after every operation, what the
Python code did. The C test (firmware/robo_pico/test/test_map_port.c) replays the same operations on the C
code and fails on the first difference. So the C port is checked against the code that was tested in simulation.

    python3 rover_mapping/tools_gen_c_vectors.py firmware/robo_pico/test/vectors/map_vectors.txt [seed] [scenarios]
"""
import os
import random
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__))))

from mapping import events as ev                       # noqa: E402
from mapping.controller import MappingController, State  # noqa: E402
from mapping.planner import plan_route, path_headings   # noqa: E402

DIRS = {0: (0, 1), 1: (1, 0), 2: (0, -1), 3: (-1, 0)}


def snap(c, out):
    g = c.graph
    last = next((m for m in reversed(c.sent_log) if isinstance(m, ev.RouteResult)), None)
    cur = g.current if g.current is not None else -1
    tgt = c.target if c.target is not None else -1
    seg = c.outstanding
    hop = f"{seg.heading}:{seg.target_marker}:{int(seg.last)}" if seg else "-"
    res = "NONE" if last is None else last.result
    rt = "-" if not c.path else ",".join(map(str, c.path))
    out.append(f"EXPECT {c.state.value} n={c.n or 0} count={g.count()} cur={cur} conf={int(c.position_confirmed)} "
               f"tgt={tgt} route={rt} hop={c.hop} cur_hop={hop} res={res} resT={last.target if last else -1} "
               f"inc={len(g.inconsistencies)}")


def scenario(rng, out, ops=70):
    cells = [(x, y) for x in range(6) for y in range(6)]
    ids = rng.sample(range(1, 37), 36)
    cell_id = dict(zip(cells, ids))
    walls = set()
    for _ in range(rng.randint(0, 14)):
        x, y = rng.choice(cells)
        dx, dy = DIRS[rng.randint(0, 3)]
        walls.add(frozenset(((x, y), (x + dx, y + dy))))
    c = MappingController()
    out.append("FSM_INIT")
    snap(c, out)
    pos = rng.choice(cells)
    n = rng.randint(2, 14)
    out.append(f"START {n} {cell_id[pos]}")
    ok = c.start(n, cell_id[pos])
    out.append(f"RET {int(ok)}")
    snap(c, out)

    def expect_ret(v):
        out.append(f"RET {int(bool(v))}")

    for _ in range(ops):
        s = c.state
        r = rng.random()
        if s == State.MAPPING:
            if r < 0.55:                       # drive one hop (sometimes with a wrong heading -> conflicts)
                d = rng.randint(0, 3)
                nx, ny = pos[0] + DIRS[d][0], pos[1] + DIRS[d][1]
                if 0 <= nx < 6 and 0 <= ny < 6 and frozenset((pos, (nx, ny))) not in walls:
                    mid = cell_id[(nx, ny)]
                    if rng.random() < 0.08:
                        d = (d + rng.randint(1, 3)) % 4        # wrong heading reported
                    out.append(f"REACHED {mid} {d}")
                    c.on_rover_event(ev.MarkerReached(mid, d))
                    pos = next((p for p in cells if cell_id[p] == c.graph.current), pos)
                else:
                    continue
            elif r < 0.68:
                mid = rng.choice(list(c.graph.markers))
                ex = [rng.randint(0, 3) for _ in range(4)]
                out.append(f"EXITREP {mid} {ex[0]} {ex[1]} {ex[2]} {ex[3]}")
                c.on_rover_event(ev.ExitReport(mid, tuple(ev.ExitStatus(v) for v in ex)))
            elif r < 0.74:
                mid, d = rng.choice(list(c.graph.markers)), rng.randint(0, 3)
                out.append(f"EXITEXP {mid} {d}")
                c.on_rover_event(ev.ExitExplored(mid, d))
            elif r < 0.82:
                mid, d = rng.choice(list(c.graph.markers)), rng.randint(0, 3)
                out.append(f"EXITBLK {mid} {d}")
                c.on_rover_event(ev.ExitBlocked(mid, d))
            elif r < 0.86:
                out.append("EXPLDONE")
                c.on_rover_event(ev.ExploreDoneLocal())
            elif r < 0.90:
                out.append("STOP"); c.operator_stop()
            elif r < 0.93:
                out.append("FAULT"); c.mission_fault()
            elif r < 0.97:
                out.append("FINISH"); expect_ret(c.finish_early())
            else:
                mid = rng.choice(list(c.graph.markers))
                out.append(f"CONFIRM {mid}")
                expect_ret(c.on_marker_seen(mid))
        elif s == State.PAUSED:
            if r < 0.3:
                mid = rng.choice(list(c.graph.markers))
                out.append(f"CONFIRM {mid}"); expect_ret(c.on_marker_seen(mid))
            elif r < 0.55:
                out.append("RESUME"); expect_ret(c.operator_resume(True))
            elif r < 0.7:
                mid = rng.choice(list(c.graph.markers) + [99 - 70])
                nn = rng.randint(1, 20)
                out.append(f"RESUMESTART {nn} {mid}"); expect_ret(c.resume_start(nn, mid))
            elif r < 0.85:
                out.append("FINISH"); expect_ret(c.finish_early())
            else:
                out.append("RESETMAP"); expect_ret(c.reset_map())
        elif s == State.MAP_COMPLETE:
            if r < 0.15:
                mid = rng.choice(list(c.graph.markers))
                out.append(f"CONFIRM {mid}"); expect_ret(c.on_marker_seen(mid))
            elif r < 0.85:
                t = rng.choice(list(c.graph.markers) + ([70 - 20] if rng.random() < 0.1 else []))
                out.append(f"GOTO {t}"); expect_ret(c.set_target(t))
            elif r < 0.92:
                out.append("STOP"); c.operator_stop()
            else:
                out.append("RESETMAP"); expect_ret(c.reset_map())
        elif s == State.WAITING_SEGMENT:
            seg = c.outstanding
            if r < 0.5:
                out.append("HOPDONE")
                c.on_rover_event(ev.SegmentDone(seg.seq))
            elif r < 0.62:
                out.append("HOPDONE_STALE")
                c.on_rover_event(ev.SegmentDone((seg.seq + 1) % 256))
            elif r < 0.85:
                out.append(f"HOPBLOCKED {c.graph.current} {seg.heading}")
                c.on_rover_event(ev.ExitBlocked(c.graph.current, seg.heading, ref_seq=seg.seq))
            elif r < 0.92:
                out.append("STOP"); c.operator_stop()
            else:
                out.append("FAULT"); c.mission_fault()
        elif s == State.FAULT_WAIT:
            if r < 0.7:
                out.append("RESETFAULT"); expect_ret(c.operator_reset(True))
            else:
                mid = rng.choice(list(c.graph.markers))
                nn = rng.randint(1, 20)
                out.append(f"RESUMESTART {nn} {mid}"); expect_ret(c.resume_start(nn, mid))
        elif s == State.MAP_EXHAUSTED:
            out.append("RESETFAULT"); expect_ret(c.operator_reset(True))
        else:
            break
        snap(c, out)
        # planner queries on the current graph
        if rng.random() < 0.5 and len(c.graph.markers) >= 2:
            for _ in range(3):
                a, b = rng.sample(list(c.graph.markers), 2)
                hd = rng.choice([None, 0, 1, 2, 3])
                up = rng.random() < 0.7
                p = plan_route(c.graph, a, b, hd, up)
                out.append(f"PLAN {a} {b} {-1 if hd is None else hd} {int(up)} "
                           f"{'-' if p is None else ','.join(map(str, p))}")
                if p is not None and len(p) > 1:
                    hops = path_headings(c.graph, p)
                    out.append("HEADINGS " + ",".join(f"{d}:{t}" for d, t in hops))
    out.append("END_SCENARIO")


if __name__ == "__main__":
    path = sys.argv[1]
    seed = int(sys.argv[2]) if len(sys.argv) > 2 else 1
    count = int(sys.argv[3]) if len(sys.argv) > 3 else 400
    rng = random.Random(seed)
    lines = []
    for _ in range(count):
        scenario(rng, lines)
    with open(path, "w") as fh:
        fh.write("\n".join(lines) + "\n")
    print(f"wrote {len(lines)} lines, {count} scenarios -> {path}")
