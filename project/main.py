from __future__ import annotations

import copy
import heapq
import math
import random
import re
import sys
import time
from dataclasses import dataclass
from enum import Enum, auto
from pathlib import Path

INSTANCES_DIR = Path(__file__).parent / "instances"

ALPHA_ARC = 0.0981
BETA_VEH = 2.11
EFF_D = 1.11
EFF_M = 1.25
BATTERY_CAPACITY_KWH = 110.0
VEHICLE_CAPACITY_TONS = 3.0
VEHICLE_CURB_WEIGHT_KG = 2000.0
JOULES_TO_KWH = 1.0 / 3_600_000.0
SPEED_SET_KMH = [30, 40, 60, 80]
MILES_TO_M = 1609.344

T0 = 20.0
ALPHA_COOLING = 0.97
T_MIN = 0.01
K_TABU = 12
GAMMA_CAP = 200.0
GAMMA_BATT = 100.0
UMBRAL = 100

BATTERY_RESERVE_LEVELS = {"0%": 0.0, "10%": 10.0, "20%": 20.0}

SMALL_INSTANCES = [f"C{n}R2" for n in range(12, 25)]
SMALL_INSTANCES_EXTENDED = [f"C{n}R2" for n in range(10, 25)]
SMALL_INSTANCES_OWN = [f"C{n}R2" for n in range(10, 12)]
LARGE_INSTANCES = []
for _n in (25, 50, 75, 100, 150):
    for _s in (2, 4, 6, 8):
        for _v in (1, 2):
            LARGE_INSTANCES.append(f"C{_n}R{_s}-{_v}")
ALL_INSTANCES = SMALL_INSTANCES_EXTENDED + LARGE_INSTANCES


@dataclass
class Vertex:
    idx: int
    x: float
    y: float
    demand_tons: float = 0.0
    is_depot: bool = False
    is_station: bool = False
    is_customer: bool = False


@dataclass
class Arc:
    i: int
    j: int
    dist_m: float
    speed_ms: float
    energy_kwh_empty: float = 0.0


class MoveKind(Enum):
    RELOCATE = auto()
    EXCHANGE = auto()
    TWO_OPT_STAR = auto()
    STATION_INRE = auto()


@dataclass
class Move:
    kind: MoveKind
    r1: int = 0
    p1: int = 0
    r2: int = 0
    p2: int = 0
    station: int = -1
    client: int = -1
    origin_route: int = -1
    client2: int = -1
    origin_route2: int = -1

    def tabu_keys(self) -> list[tuple[int, int]]:
        """TABUROUTE: par (cliente, ruta_origen), Exchange prohibe ambos clientes."""
        if self.kind == MoveKind.RELOCATE and self.client >= 0:
            return [(self.client, self.origin_route)]
        if self.kind == MoveKind.EXCHANGE and self.client >= 0:
            keys = [(self.client, self.origin_route)]
            if self.client2 >= 0:
                keys.append((self.client2, self.origin_route2))
            return keys
        return []


def battery_reserve_kwh(reserve_pct: float) -> float:
    return BATTERY_CAPACITY_KWH * reserve_pct / 100.0


def relative_gap(solution: float, reference: float) -> float:
    """Gap relativo (Sol - ref) / ref x 100%."""
    return (solution - reference) / reference * 100.0 if reference > 0 else 0.0


def parse_instance_name(name: str) -> tuple[int, int]:
    base = name.split("-")[0]
    m = re.match(r"C(\d+)R(\d+)", base)
    if not m:
        raise ValueError(f"Nombre inválido: {name}")
    return int(m.group(1)), int(m.group(2))


def _instance_path(name: str) -> Path:
    for sub in ("small", "large"):
        path = INSTANCES_DIR / sub / f"{name}.txt"
        if path.exists():
            return path
    raise FileNotFoundError(
        f"No se encontró la instancia '{name}' en "
        f"{INSTANCES_DIR}/small ni {INSTANCES_DIR}/large"
    )


def _parse_meta_line(line: str) -> tuple[str, str] | None:
    if ":" in line:
        key, value = line.split(":", 1)
        return key.strip(), value.strip()
    if " " in line:
        key, value = line.split(maxsplit=1)
        return key.strip(), value.strip()
    return None


def _meta_float(meta: dict[str, str], *keys: str, default: float) -> float:
    for key in keys:
        if key in meta:
            return float(meta[key])
    return default


def load_instance(name: str) -> tuple[list[Vertex], dict[tuple[int, int], Arc]]:
    path = _instance_path(name)
    lines = [
        ln.strip()
        for ln in path.read_text(encoding="utf-8").splitlines()
        if ln.strip() and not ln.startswith("#")
    ]
    meta: dict[str, str] = {}
    i = 0
    section_headers = {
        "NODE_COORD_SECTION",
        "DISTANCE_MATRIX_MILES",
        "SPEED_MATRIX_KMH",
        "NODES",
        "SPEEDS",
        "EOF",
    }
    while i < len(lines) and lines[i] not in section_headers:
        parsed = _parse_meta_line(lines[i])
        if parsed:
            meta[parsed[0]] = parsed[1]
        i += 1

    miles_to_m = _meta_float(meta, "MILES_TO_METERS", default=MILES_TO_M)
    alpha = _meta_float(meta, "ALPHA", "ALPHA_ARC", default=ALPHA_ARC)
    beta = _meta_float(meta, "BETA", "BETA_VEH", default=BETA_VEH)
    eff_d = _meta_float(meta, "EFF_BATTERY_DISCHARGE", "EFF_D", default=EFF_D)
    eff_m = _meta_float(meta, "EFF_MOTOR", "EFF_M", default=EFF_M)
    curb_kg = _meta_float(meta, "CURB_WEIGHT_KG", default=VEHICLE_CURB_WEIGHT_KG)

    vertices: list[Vertex] = []
    id_map: dict[int, Vertex] = {}
    dist_miles: list[list[float]] = []
    speed_kmh: list[list[int]] = []
    speed_map: dict[tuple[int, int], int] = {}

    while i < len(lines) and lines[i] != "EOF":
        section = lines[i]
        if section == "NODE_COORD_SECTION":
            i += 1
            while i < len(lines) and lines[i] not in section_headers:
                parts = re.split(r"\s+", lines[i])
                vid = int(parts[0])
                kind = parts[1]
                x_m = float(parts[2]) * miles_to_m
                y_m = float(parts[3]) * miles_to_m
                demand = float(parts[4])
                v = Vertex(vid, x_m, y_m, demand)
                if kind == "D":
                    v.is_depot = True
                elif kind == "S":
                    v.is_station = True
                else:
                    v.is_customer = True
                vertices.append(v)
                id_map[vid] = v
                i += 1
            continue

        if section == "NODES":
            i += 1
            while i < len(lines) and lines[i] not in section_headers:
                parts = lines[i].split()
                vid = int(parts[0])
                x, y, demand = float(parts[1]), float(parts[2]), float(parts[3])
                kind = parts[4] if len(parts) > 4 else "customer"
                v = Vertex(vid, x, y, demand)
                if kind in ("depot_start", "depot_end", "depot"):
                    v.is_depot = True
                elif kind == "station":
                    v.is_station = True
                else:
                    v.is_customer = True
                vertices.append(v)
                id_map[vid] = v
                i += 1
            continue

        if section == "DISTANCE_MATRIX_MILES":
            i += 1
            while i < len(lines) and lines[i] not in section_headers:
                dist_miles.append([float(x) for x in re.split(r"\s+", lines[i])])
                i += 1
            continue

        if section == "SPEED_MATRIX_KMH":
            i += 1
            while i < len(lines) and lines[i] not in section_headers:
                speed_kmh.append([int(x) for x in re.split(r"\s+", lines[i])])
                i += 1
            continue

        if section == "SPEEDS":
            i += 1
            while i < len(lines) and lines[i] not in section_headers:
                a, b, spd = lines[i].split()
                speed_map[(int(a), int(b))] = int(spd)
                i += 1
            continue

        i += 1

    vertices.sort(key=lambda v: v.idx)
    n = len(vertices)

    if speed_kmh:
        speed_map = {
            (a, b): speed_kmh[a][b]
            for a in range(n)
            for b in range(n)
            if a != b and speed_kmh[a][b] > 0
        }
    elif not speed_map:
        seed = int(meta.get("SPEED_SEED", 0))
        rng = random.Random(seed)
        ids = [v.idx for v in vertices]
        for a in ids:
            for b in ids:
                if a != b:
                    speed_map[(a, b)] = rng.choice(SPEED_SET_KMH)

    arcs: dict[tuple[int, int], Arc] = {}
    for (a, b), spd_val in speed_map.items():
        if dist_miles:
            dist = dist_miles[a][b] * miles_to_m
        else:
            va, vb = id_map[a], id_map[b]
            dx, dy = va.x - vb.x, va.y - vb.y
            dist = math.sqrt(dx * dx + dy * dy)
        spd_ms = spd_val * 1000.0 / 3600.0
        e_empty = (
            (alpha * curb_kg * dist + beta * spd_ms**2 * dist)
            * eff_d
            * eff_m
            * JOULES_TO_KWH
        )
        arcs[(a, b)] = Arc(a, b, dist, spd_ms, e_empty)
    return vertices, arcs


def arc_energy_kwh(arcs: dict, i: int, j: int, load_kg: float) -> float:
    arc = arcs.get((i, j))
    if arc is None:
        return 1e9
    e_load = ALPHA_ARC * load_kg * arc.dist_m * EFF_D * EFF_M * JOULES_TO_KWH
    return arc.energy_kwh_empty + e_load


def arc_dist(arcs: dict, i: int, j: int) -> float:
    a = arcs.get((i, j))
    return a.dist_m if a else 1e9


def route_total_load_kg(seq: list[int], vertices: dict[int, Vertex]) -> float:
    return sum(vertices[i].demand_tons * 1000.0 for i in seq if vertices[i].is_customer)


def load_leaving_position(
    seq: list[int], pos: int, vertices: dict[int, Vertex]
) -> float:
    load = route_total_load_kg(seq, vertices)
    for k in range(1, pos + 1):
        if vertices[seq[k]].is_customer:
            load -= vertices[seq[k]].demand_tons * 1000.0
    return max(0.0, load)


def capacity_violation_kg(seq: list[int], vertices: dict[int, Vertex]) -> float:
    load = route_total_load_kg(seq, vertices)
    return max(0.0, load - VEHICLE_CAPACITY_TONS * 1000.0)


def is_charge_node(v: Vertex) -> bool:
    return v.is_depot or v.is_station


def charge_nodes_for_segment(
    to_j: int,
    station_idxs: list[int],
    depot_idxs: list[int],
) -> list[int]:
    """Estaciones + depósito inicio; depósito fin solo como destino final de la ruta."""
    nodes = list(station_idxs) + [depot_idxs[0]]
    end_depot = depot_idxs[-1]
    if end_depot != depot_idxs[0] and to_j == end_depot:
        nodes.append(end_depot)
    return list(dict.fromkeys(nodes))


def evaluate_route_exact(
    seq: list[int],
    vertices: dict[int, Vertex],
    arcs: dict,
    reserve_pct: float = 0.0,
) -> tuple[float, float]:
    if len(seq) < 2:
        return 0.0, 0.0
    reserve_kwh = battery_reserve_kwh(reserve_pct)
    load_kg = route_total_load_kg(seq, vertices)
    batt = BATTERY_CAPACITY_KWH
    total_e = 0.0
    batt_viol = 0.0
    for step in range(len(seq) - 1):
        i, j = seq[step], seq[step + 1]
        if vertices[i].is_customer:
            load_kg -= vertices[i].demand_tons * 1000.0
        e = arc_energy_kwh(arcs, i, j, max(0.0, load_kg))
        total_e += e
        batt -= e
        if is_charge_node(vertices[j]):
            batt = BATTERY_CAPACITY_KWH
        if batt < 0:
            batt_viol += abs(batt)
            batt = 0.0
        elif batt < reserve_kwh and not is_charge_node(vertices[j]):
            batt_viol += reserve_kwh - batt
            batt = reserve_kwh
    return total_e, batt_viol


def min_energy_path(
    from_i: int,
    to_j: int,
    load_kg: float,
    start_batt: float,
    vertices: dict[int, Vertex],
    arcs: dict,
    charge_nodes: list[int],
    min_end_batt: float = 0.0,
    reserve_kwh: float = 0.0,
) -> tuple[float, list[int], float]:
    if from_i == to_j:
        return 0.0, [from_i], start_batt

    allowed = set(charge_nodes) | {from_i, to_j}
    best_cost: dict[tuple[int, float], float] = {}
    predecessor: dict[tuple[int, float], tuple[int, float]] = {}
    pq: list[tuple[float, int, float]] = []
    start_key = (from_i, round(start_batt, 4))
    best_cost[start_key] = 0.0
    heapq.heappush(pq, (0.0, from_i, start_batt))

    while pq:
        cost, u, batt = heapq.heappop(pq)
        key = (u, round(batt, 4))
        if best_cost.get(key, 1e18) < cost - 1e-9:
            continue
        if u == to_j:
            need = (
                max(min_end_batt, reserve_kwh)
                if not is_charge_node(vertices[u])
                else 0.0
            )
            if batt + 1e-9 >= need:
                path = [u]
                state = key
                while state in predecessor:
                    state = predecessor[state]
                    path.append(state[0])
                path.reverse()
                return cost, path, batt
            continue

        for v in allowed:
            if v == u:
                continue
            if vertices[v].is_customer and v != to_j:
                continue
            e = arc_energy_kwh(arcs, u, v, load_kg)
            if batt + 1e-9 < e:
                continue
            nb = batt - e
            if is_charge_node(vertices[v]):
                nb = BATTERY_CAPACITY_KWH
            elif nb + 1e-9 < reserve_kwh:
                continue
            nkey = (v, round(nb, 4))
            nc = cost + e
            if nc < best_cost.get(nkey, 1e18):
                best_cost[nkey] = nc
                predecessor[nkey] = key
                heapq.heappush(pq, (nc, v, nb))

    if min_end_batt > 0.0:
        best_detour: tuple[float, list[int], float] | None = None
        ordered = [s for s in charge_nodes if vertices[s].is_station] + [
            s for s in charge_nodes if vertices[s].is_depot
        ]
        for st in ordered:
            if st == from_i:
                continue
            e1, p1, b1 = min_energy_path(
                from_i,
                st,
                load_kg,
                start_batt,
                vertices,
                arcs,
                charge_nodes,
                0.0,
                reserve_kwh,
            )
            if e1 >= 1e17:
                continue
            e2, p2, b2 = min_energy_path(
                st, to_j, load_kg, b1, vertices, arcs, charge_nodes, 0.0, reserve_kwh
            )
            if e2 >= 1e17:
                continue
            need_end = (
                max(min_end_batt, reserve_kwh)
                if not is_charge_node(vertices[to_j])
                else 0.0
            )
            if b2 + 1e-6 < need_end:
                continue
            total = e1 + e2
            if best_detour is None or total < best_detour[0]:
                best_detour = (total, p1 + p2[1:], b2)
        if best_detour is not None:
            return best_detour

    return 1e18, [], 0.0


def _charge_detour_to(
    current: int,
    target: int,
    load_kg: float,
    batt: float,
    vertices: dict[int, Vertex],
    arcs: dict,
    charge_nodes: list[int],
    reserve_kwh: float,
) -> tuple[float, list[int], float] | None:
    """Camino factible vía un nodo de recarga (estación o depósito)."""
    best: tuple[float, list[int], float] | None = None
    for st in charge_nodes:
        if st == current:
            continue
        e1, p1, b1 = min_energy_path(
            current, st, load_kg, batt, vertices, arcs, charge_nodes, 0.0, reserve_kwh
        )
        if e1 >= 1e17:
            continue
        e2, p2, b2 = min_energy_path(
            st, target, load_kg, b1, vertices, arcs, charge_nodes, 0.0, reserve_kwh
        )
        if e2 >= 1e17:
            continue
        total = e1 + e2
        path = p1 + p2[1:]
        if best is None or total < best[0]:
            best = (total, path, b2)
    return best


def greedy_station_insert_route(
    client_seq: list[int],
    vertices: dict[int, Vertex],
    arcs: dict,
    station_idxs: list[int],
    depot_idxs: list[int],
    reserve_pct: float = 0.0,
) -> list[int]:
    """Paper §4.1.1 step 3: insert stations when battery cannot reach next customer."""
    if len(client_seq) < 2:
        return list(client_seq)
    reserve_kwh = battery_reserve_kwh(reserve_pct)
    charge_nodes = charge_nodes_for_segment(client_seq[-1], station_idxs, depot_idxs)
    full = [client_seq[0]]
    batt = BATTERY_CAPACITY_KWH
    cust_idx = 0

    while cust_idx < len(client_seq) - 1:
        current = full[-1]
        next_cust = client_seq[cust_idx + 1]
        load = load_leaving_position(client_seq, cust_idx, vertices)
        seg_nodes = charge_nodes_for_segment(next_cust, station_idxs, depot_idxs)
        seg_e, seg_path, end_batt = min_energy_path(
            current,
            next_cust,
            load,
            batt,
            vertices,
            arcs,
            seg_nodes,
            reserve_kwh=reserve_kwh,
        )
        if seg_e < 1e17:
            full.extend(seg_path[1:])
            batt = end_batt
            cust_idx += 1
            continue

        best_seg: list[int] | None = None
        best_batt = 0.0
        best_cost = 1e18
        for st in charge_nodes:
            if st == current:
                continue
            e1, p1, b1 = min_energy_path(
                current,
                st,
                load,
                batt,
                vertices,
                arcs,
                seg_nodes,
                reserve_kwh=reserve_kwh,
            )
            if e1 >= 1e17:
                continue
            e2, p2, b2 = min_energy_path(
                st,
                next_cust,
                load,
                b1,
                vertices,
                arcs,
                seg_nodes,
                reserve_kwh=reserve_kwh,
            )
            if e2 >= 1e17:
                continue
            if e1 + e2 < best_cost:
                best_cost = e1 + e2
                best_seg = p1[1:] + p2[1:]
                best_batt = b2

        if best_seg is not None:
            full.extend(best_seg)
            batt = best_batt
            cust_idx += 1
        else:
            detour = _charge_detour_to(
                current,
                next_cust,
                load,
                batt,
                vertices,
                arcs,
                seg_nodes,
                reserve_kwh,
            )
            if detour is not None:
                _, path, batt = detour
                full.extend(path[1:])
                cust_idx += 1
            else:
                break
    if full[-1] != client_seq[-1]:
        tail = _charge_detour_to(
            full[-1],
            client_seq[-1],
            load_leaving_position(client_seq, len(client_seq) - 2, vertices),
            batt,
            vertices,
            arcs,
            charge_nodes,
            reserve_kwh,
        )
        if tail is not None:
            _, path, _ = tail
            full.extend(path[1:])
        elif full[-1] != client_seq[-1]:
            e, path, _ = min_energy_path(
                full[-1],
                client_seq[-1],
                0.0,
                batt,
                vertices,
                arcs,
                charge_nodes,
                reserve_kwh=reserve_kwh,
            )
            if e < 1e17:
                full.extend(path[1:])
    return full


def dp_station_insertion(
    client_seq: list[int],
    vertices: dict[int, Vertex],
    arcs: dict,
    station_idxs: list[int],
    depot_idxs: list[int],
    reserve_pct: float = 0.0,
) -> list[int]:
    if len(client_seq) < 2:
        return list(client_seq)
    reserve_kwh = battery_reserve_kwh(reserve_pct)
    n = len(client_seq)
    labels: dict[int, list[tuple[float, float, list[int]]]] = {
        0: [(BATTERY_CAPACITY_KWH, 0.0, [client_seq[0]])]
    }

    for step in range(n - 1):
        node_i = client_seq[step]
        node_j = client_seq[step + 1]
        load = load_leaving_position(client_seq, step, vertices)
        seg_nodes = charge_nodes_for_segment(node_j, station_idxs, depot_idxs)
        need_end = reserve_kwh if step + 1 < n - 1 else 0.0
        new_labels: list[tuple[float, float, list[int]]] = []
        for batt, cost, path in labels[step]:
            seg_e, seg_path, end_batt = min_energy_path(
                node_i,
                node_j,
                load,
                batt,
                vertices,
                arcs,
                seg_nodes,
                min_end_batt=need_end,
                reserve_kwh=reserve_kwh,
            )
            if seg_e >= 1e17:
                continue
            merged = path + seg_path[1:]
            new_labels.append((end_batt, cost + seg_e, merged))
        if not new_labels:
            return greedy_station_insert_route(
                client_seq, vertices, arcs, station_idxs, depot_idxs, reserve_pct
            )
        pareto: list[tuple[float, float, list[int]]] = []
        for cand in new_labels:
            dominated = False
            for other in new_labels:
                if other is cand:
                    continue
                if other[0] >= cand[0] - 1e-6 and other[1] <= cand[1] + 1e-6:
                    if other[0] > cand[0] + 1e-6 or other[1] < cand[1] - 1e-6:
                        dominated = True
                        break
            if not dominated:
                pareto.append(cand)
        labels[step + 1] = pareto

    final = labels[n - 1]
    best = min(final, key=lambda t: t[1])
    return best[2]


def f_gen_route(
    seq: list[int], vertices: dict[int, Vertex], arcs: dict, reserve_pct: float = 0.0
) -> float:
    e, bv = evaluate_route_exact(seq, vertices, arcs, reserve_pct)
    cv = capacity_violation_kg(seq, vertices)
    return e + GAMMA_CAP * cv + GAMMA_BATT * bv


def f_gen_solution(
    routes: list[list[int]],
    vertices: dict[int, Vertex],
    arcs: dict,
    reserve_pct: float = 0.0,
) -> float:
    return sum(f_gen_route(r, vertices, arcs, reserve_pct) for r in routes)


def exact_energy_solution(
    routes: list[list[int]],
    vertices: dict[int, Vertex],
    arcs: dict,
    reserve_pct: float = 0.0,
) -> float:
    return sum(evaluate_route_exact(r, vertices, arcs, reserve_pct)[0] for r in routes)


def surrogate_batt_violation_delta(
    routes: list[list[int]],
    move: Move,
    vertices: dict[int, Vertex],
    arcs: dict,
    reserve_pct: float = 0.0,
) -> float:
    """L'_batt: solo arcos nuevos del movimiento (Zhang et al. §4.1.2)."""
    reserve_kwh = battery_reserve_kwh(reserve_pct)
    trial = apply_move(copy.deepcopy(routes), move)
    delta = 0.0
    for ri in affected_routes(move):
        if ri >= len(routes) or ri >= len(trial):
            continue
        before, after = routes[ri], trial[ri]
        if len(before) < 2 or len(after) < 2:
            continue
        load = route_total_load_kg(before, vertices)
        batt = BATTERY_CAPACITY_KWH
        for step in range(len(after) - 1):
            i, j = after[step], after[step + 1]
            if vertices[i].is_customer:
                load -= vertices[i].demand_tons * 1000.0
            e = arc_energy_kwh(arcs, i, j, max(0.0, load))
            batt -= e
            if is_charge_node(vertices[j]):
                batt = BATTERY_CAPACITY_KWH
            pair = (i, j)
            in_before = any(
                before[k] == pair[0] and before[k + 1] == pair[1]
                for k in range(len(before) - 1)
            )
            if not in_before:
                if batt < 0:
                    delta += abs(batt)
                    batt = 0.0
                elif batt < reserve_kwh and not is_charge_node(vertices[j]):
                    delta += reserve_kwh - batt
    return delta


def arc_delta(i: int, j: int, load_kg: float, sign: int, arcs: dict) -> float:
    return sign * arc_energy_kwh(arcs, i, j, load_kg)


def delta_approx_move(
    routes: list[list[int]],
    move: Move,
    vertices: dict[int, Vertex],
    arcs: dict,
    reserve_pct: float = 0.0,
) -> float:
    if move.kind == MoveKind.RELOCATE:
        return _delta_relocate(routes, move, vertices, arcs, reserve_pct)
    if move.kind == MoveKind.EXCHANGE:
        return _delta_exchange(routes, move, vertices, arcs, reserve_pct)
    if move.kind == MoveKind.TWO_OPT_STAR:
        return _delta_two_opt_star(routes, move, vertices, arcs, reserve_pct)
    if move.kind == MoveKind.STATION_INRE:
        return _delta_station_inre(routes, move, vertices, arcs, reserve_pct)
    return 1e18


def _delta_relocate(routes, move, vertices, arcs, reserve_pct=0.0) -> float:
    r_src, pos_src, r_dst, ins = move.r1, move.p1, move.r2, move.p2
    seq1, seq2 = routes[r_src], routes[r_dst]
    if pos_src <= 0 or pos_src + 1 >= len(seq1):
        return 1e18
    if ins <= 0 or ins >= len(seq2):
        return 1e18
    client = seq1[pos_src]
    prev1, next1 = seq1[pos_src - 1], seq1[pos_src + 1]
    load1 = load_leaving_position(seq1, pos_src - 1, vertices)
    d = 0.0
    d -= arc_energy_kwh(arcs, prev1, client, load1)
    d -= arc_energy_kwh(arcs, client, next1, load1)
    d += arc_energy_kwh(arcs, prev1, next1, load1)
    prev2 = seq2[ins - 1]
    next2 = seq2[ins] if ins < len(seq2) else seq2[-1]
    load2 = load_leaving_position(seq2, ins - 1, vertices)
    d -= arc_energy_kwh(arcs, prev2, next2, load2)
    d += arc_energy_kwh(arcs, prev2, client, load2)
    d += arc_energy_kwh(arcs, client, next2, load2)
    cap_before = sum(capacity_violation_kg(r, vertices) for r in routes)
    trial = apply_move(copy.deepcopy(routes), move)
    cap_after = sum(capacity_violation_kg(r, vertices) for r in trial)
    batt_d = surrogate_batt_violation_delta(routes, move, vertices, arcs, reserve_pct)
    return d + GAMMA_CAP * (cap_after - cap_before) + GAMMA_BATT * batt_d


def _swap_client_delta(
    seq: list[int], pos: int, old_c: int, new_c: int, vertices: dict, arcs: dict
) -> float:
    if pos <= 0 or pos + 1 >= len(seq):
        return 1e18
    prev_n, next_n = seq[pos - 1], seq[pos + 1]
    load = load_leaving_position(seq, pos - 1, vertices)
    d = -arc_energy_kwh(arcs, prev_n, old_c, load)
    d -= arc_energy_kwh(arcs, old_c, next_n, load)
    d += arc_energy_kwh(arcs, prev_n, new_c, load)
    d += arc_energy_kwh(arcs, new_c, next_n, load)
    return d


def _delta_exchange(routes, move, vertices, arcs, reserve_pct=0.0) -> float:
    r1, p1, r2, p2 = move.r1, move.p1, move.r2, move.p2
    c1, c2 = routes[r1][p1], routes[r2][p2]
    d = _swap_client_delta(routes[r1], p1, c1, c2, vertices, arcs)
    d += _swap_client_delta(routes[r2], p2, c2, c1, vertices, arcs)
    trial = apply_move(copy.deepcopy(routes), move)
    cap_d = sum(capacity_violation_kg(r, vertices) for r in trial) - sum(
        capacity_violation_kg(r, vertices) for r in routes
    )
    batt_d = surrogate_batt_violation_delta(routes, move, vertices, arcs, reserve_pct)
    return d + GAMMA_CAP * cap_d + GAMMA_BATT * batt_d


def _delta_two_opt_star(routes, move, vertices, arcs, reserve_pct=0.0) -> float:
    r1, r2, p1, p2 = move.r1, move.r2, move.p1, move.p2
    s1, s2 = routes[r1], routes[r2]
    if p1 >= len(s1) - 1 or p2 >= len(s2) - 1:
        return 1e18
    a, b = s1[p1], s1[p1 + 1]
    c, d = s2[p2], s2[p2 + 1]
    load1 = load_leaving_position(s1, p1, vertices)
    load2 = load_leaving_position(s2, p2, vertices)
    delta = 0.0
    delta -= arc_energy_kwh(arcs, a, b, load1)
    delta -= arc_energy_kwh(arcs, c, d, load2)
    delta += arc_energy_kwh(arcs, a, d, load1)
    delta += arc_energy_kwh(arcs, c, b, load2)
    trial = apply_move(copy.deepcopy(routes), move)
    cap_d = sum(capacity_violation_kg(r, vertices) for r in trial) - sum(
        capacity_violation_kg(r, vertices) for r in routes
    )
    batt_d = surrogate_batt_violation_delta(routes, move, vertices, arcs, reserve_pct)
    return delta + GAMMA_CAP * cap_d + GAMMA_BATT * batt_d


def _delta_station_inre(routes, move, vertices, arcs, reserve_pct=0.0) -> float:
    r, ins, st = move.r1, move.p2, move.station
    seq = routes[r]
    if ins <= 0 or ins >= len(seq):
        return 1e18
    u, v = seq[ins - 1], seq[ins]
    load = load_leaving_position(seq, ins - 1, vertices)
    d = -arc_energy_kwh(arcs, u, v, load)
    d += arc_energy_kwh(arcs, u, st, load)
    d += arc_energy_kwh(arcs, st, v, 0.0)
    trial = apply_move(copy.deepcopy(routes), move)
    batt_d = surrogate_batt_violation_delta(routes, move, vertices, arcs, reserve_pct)
    return d + GAMMA_BATT * batt_d


def apply_move(routes: list[list[int]], move: Move) -> list[list[int]]:
    out = [list(r) for r in routes]
    if move.kind == MoveKind.RELOCATE:
        r_src, r_dst = move.r1, move.r2
        client = out[r_src].pop(move.p1)
        if len(out[r_src]) <= 2:
            out.pop(r_src)
            if r_dst > r_src:
                r_dst -= 1
        out[r_dst].insert(move.p2, client)
    elif move.kind == MoveKind.EXCHANGE:
        c1 = out[move.r1][move.p1]
        c2 = out[move.r2][move.p2]
        out[move.r1][move.p1] = c2
        out[move.r2][move.p2] = c1
    elif move.kind == MoveKind.TWO_OPT_STAR:
        s1, s2 = out[move.r1], out[move.r2]
        tail1 = s1[move.p1 + 1 :]
        tail2 = s2[move.p2 + 1 :]
        out[move.r1] = s1[: move.p1 + 1] + tail2
        out[move.r2] = s2[: move.p2 + 1] + tail1
    elif move.kind == MoveKind.STATION_INRE:
        out[move.r1].insert(move.p2, move.station)
    return out


def affected_routes(move: Move) -> set[int]:
    if move.kind in (MoveKind.RELOCATE, MoveKind.EXCHANGE, MoveKind.TWO_OPT_STAR):
        return {move.r1, move.r2}
    return {move.r1}


def affected_routes_after_apply(
    move: Move, client_routes: list[list[int]]
) -> list[int]:
    """Índices de rutas a re-optimizar con DP tras apply_move."""
    aff = set(affected_routes(move))
    if move.kind == MoveKind.RELOCATE:
        src, dst = move.r1, move.r2
        if len(client_routes[src]) <= 3:
            aff.discard(src)
            aff = {i - 1 if i > src else i for i in aff}
    return sorted(i for i in aff if i >= 0)


def apply_accepted_move_with_dp(
    client_routes: list[list[int]],
    move: Move,
    vertices: dict[int, Vertex],
    arcs: dict,
    station_idxs: list[int],
    depot_idxs: list[int],
    reserve_pct: float = 0.0,
) -> list[list[int]]:
    trial = apply_move(copy.deepcopy(client_routes), move)
    for ri in affected_routes_after_apply(move, client_routes):
        if ri < len(trial):
            trial[ri] = dp_station_insertion(
                trial[ri],
                vertices,
                arcs,
                station_idxs,
                depot_idxs,
                reserve_pct,
            )
    return trial


def optimize_routes_with_dp(
    routes: list[list[int]],
    route_indices: set[int],
    vertices: dict[int, Vertex],
    arcs: dict,
    station_idxs: list[int],
    depot_idxs: list[int],
    reserve_pct: float = 0.0,
) -> None:
    for ri in route_indices:
        if ri < len(routes):
            routes[ri] = dp_station_insertion(
                routes[ri],
                vertices,
                arcs,
                station_idxs,
                depot_idxs,
                reserve_pct,
            )


def strip_client_routes(
    routes: list[list[int]], vertices: dict[int, Vertex]
) -> list[list[int]]:
    return [
        [i for i in r if vertices[i].is_depot or vertices[i].is_customer]
        for r in routes
    ]


def greedy_build(
    vertices: dict[int, Vertex],
    arcs: dict,
    depot_start: int,
    depot_end: int,
    rng: random.Random,
) -> list[list[int]]:
    """Fase 1 diseño: greedy aleatorio (capacidad sí, batería no)."""
    customers = [v.idx for v in vertices.values() if v.is_customer]
    rng.shuffle(customers)
    routes: list[list[int]] = []
    unvisited = set(customers)

    while unvisited:
        cur = [depot_start]
        cur_load = 0.0
        while unvisited:
            feasible = [
                c
                for c in unvisited
                if cur_load + vertices[c].demand_tons <= VEHICLE_CAPACITY_TONS + 1e-9
            ]
            if not feasible:
                break
            best_c = rng.choice(feasible)
            cur.append(best_c)
            cur_load += vertices[best_c].demand_tons
            unvisited.remove(best_c)
        cur.append(depot_end)
        routes.append(cur)
    return routes


def greedy_build_distance(
    vertices: dict[int, Vertex],
    arcs: dict,
    depot_start: int,
    depot_end: int,
    rng: random.Random,
) -> list[list[int]]:
    customers = [v.idx for v in vertices.values() if v.is_customer]
    rng.shuffle(customers)
    routes: list[list[int]] = []
    unvisited = set(customers)

    while unvisited:
        cur = [depot_start]
        cur_load = 0.0
        current = depot_start
        while unvisited:
            best_c, best_dist = None, float("inf")
            for c in list(unvisited):
                d = vertices[c].demand_tons
                if cur_load + d > VEHICLE_CAPACITY_TONS + 1e-9:
                    continue
                dist = arc_dist(arcs, current, c)
                if dist < best_dist:
                    best_dist = dist
                    best_c = c
            if best_c is None:
                break
            cur.append(best_c)
            cur_load += vertices[best_c].demand_tons
            unvisited.remove(best_c)
            current = best_c
        cur.append(depot_end)
        routes.append(cur)
    return routes


def build_routes_with_dp(
    client_routes: list[list[int]],
    vertices: dict[int, Vertex],
    arcs: dict,
    station_idxs: list[int],
    depot_idxs: list[int],
    reserve_pct: float = 0.0,
) -> list[list[int]]:
    return [
        dp_station_insertion(r, vertices, arcs, station_idxs, depot_idxs, reserve_pct)
        for r in client_routes
    ]


def solve_distance_minimizing(
    vertices_list: list[Vertex],
    arcs: dict,
    seed: int = 42,
    reserve_pct: float = 0.0,
) -> tuple[list[list[int]], float]:
    vertices = {v.idx: v for v in vertices_list}
    station_idxs = [v.idx for v in vertices_list if v.is_station]
    depot_idxs = [v.idx for v in vertices_list if v.is_depot]
    d0, d1 = depot_idxs[0], depot_idxs[-1]
    rng = random.Random(seed)
    client_routes = greedy_build_distance(vertices, arcs, d0, d1, rng)
    full = build_routes_with_dp(
        client_routes, vertices, arcs, station_idxs, depot_idxs, reserve_pct
    )
    energy = exact_energy_solution(full, vertices, arcs, reserve_pct)
    return full, energy


def centroid(seq: list[int], vertices: dict[int, Vertex]) -> tuple[float, float]:
    custs = [vertices[i] for i in seq if vertices[i].is_customer]
    if not custs:
        return 0.0, 0.0
    return sum(v.x for v in custs) / len(custs), sum(v.y for v in custs) / len(custs)


def candidate_ratio(temperature: float) -> float:
    frac = (temperature - T_MIN) / max(T0 - T_MIN, 1e-9)
    return max(0.25, min(1.0, 0.25 + 0.75 * frac))


def candidate_clients(
    routes: list[list[int]], vertices: dict[int, Vertex], ratio: float
) -> list[tuple[int, int, int]]:
    items: list[tuple[float, int, int, int]] = []
    for ri, seq in enumerate(routes):
        cx, cy = centroid(seq, vertices)
        for pos, idx in enumerate(seq):
            v = vertices[idx]
            if not v.is_customer:
                continue
            dist_c = math.hypot(v.x - cx, v.y - cy)
            score = v.demand_tons * 1000.0 + dist_c
            items.append((score, ri, pos, idx))
    items.sort(reverse=True)
    keep = max(1, int(len(items) * ratio))
    return [(ri, pos, idx) for _, ri, pos, idx in items[:keep]]


def generate_neighborhood(
    routes: list[list[int]],
    vertices: dict[int, Vertex],
    station_idxs: list[int],
    ratio: float,
    rng: random.Random,
) -> list[Move]:
    moves: list[Move] = []
    cands = candidate_clients(routes, vertices, ratio)
    n_routes = len(routes)

    for r_src, pos_src, client_idx in cands:
        seq = routes[r_src]
        if len(seq) <= 3:
            continue
        for r_dst in range(n_routes):
            if r_dst == r_src:
                continue
            seq_d = routes[r_dst]
            for ins in range(1, len(seq_d)):
                moves.append(
                    Move(
                        MoveKind.RELOCATE,
                        r_src,
                        pos_src,
                        r_dst,
                        ins,
                        client=client_idx,
                        origin_route=r_src,
                    )
                )

    for r_src, pos_src, c1 in cands:
        for r_dst, pos_dst, c2 in cands:
            if r_src == r_dst and pos_src == pos_dst:
                continue
            s1, s2 = routes[r_src], routes[r_dst]
            if pos_src <= 0 or pos_src + 1 >= len(s1):
                continue
            if pos_dst <= 0 or pos_dst + 1 >= len(s2):
                continue
            if len(s1) <= 3 and r_src != r_dst:
                continue
            moves.append(
                Move(
                    MoveKind.EXCHANGE,
                    r_src,
                    pos_src,
                    r_dst,
                    pos_dst,
                    client=c1,
                    origin_route=r_src,
                    client2=c2,
                    origin_route2=r_dst,
                )
            )

    for r1 in range(n_routes):
        for r2 in range(r1 + 1, n_routes):
            s1, s2 = routes[r1], routes[r2]
            for p1 in range(1, len(s1) - 1):
                for p2 in range(1, len(s2) - 1):
                    moves.append(Move(MoveKind.TWO_OPT_STAR, r1, p1, r2, p2))

    top = cands[: max(1, len(cands) // 4)]
    for r, pos, _ in top:
        seq = routes[r]
        for st in station_idxs[: min(3, len(station_idxs))]:
            for ins in range(1, len(seq)):
                moves.append(Move(MoveKind.STATION_INRE, r, pos, r, ins, station=st))

    rng.shuffle(moves)
    cap = max(200, int(len(moves) * 0.15))
    return moves[:cap]


def evaluate_solution_with_dp(
    client_routes: list[list[int]],
    vertices: dict[int, Vertex],
    arcs: dict,
    station_idxs: list[int],
    depot_idxs: list[int],
    reserve_pct: float = 0.0,
) -> tuple[list[list[int]], float, float]:
    full = build_routes_with_dp(
        client_routes, vertices, arcs, station_idxs, depot_idxs, reserve_pct
    )
    fgen = f_gen_solution(full, vertices, arcs, reserve_pct)
    energy = exact_energy_solution(full, vertices, arcs, reserve_pct)
    return full, energy, fgen


def count_station_visits(routes: list[list[int]], vertices: dict[int, Vertex]) -> int:
    return sum(1 for seq in routes for i in seq if vertices[i].is_station)


def run_eh_sats(
    vertices_list: list[Vertex],
    arcs: dict,
    seed: int = 42,
    verbose: bool = False,
    battery_reserve_pct: float = 0.0,
) -> tuple[list[list[int]], float, float]:
    vertices = {v.idx: v for v in vertices_list}
    station_idxs = [v.idx for v in vertices_list if v.is_station]
    depot_idxs = [v.idx for v in vertices_list if v.is_depot]
    depot_start = depot_idxs[0]
    depot_end = depot_idxs[-1] if len(depot_idxs) > 1 else depot_idxs[0]

    rng = random.Random(seed)
    client_routes = greedy_build(vertices, arcs, depot_start, depot_end, rng)
    full_routes = build_routes_with_dp(
        client_routes, vertices, arcs, station_idxs, depot_idxs, battery_reserve_pct
    )
    client_routes = strip_client_routes(full_routes, vertices)
    current_f = f_gen_solution(full_routes, vertices, arcs, battery_reserve_pct)

    best_routes = [list(r) for r in full_routes]
    best_f = current_f
    tabu: dict[tuple[int, int], int] = {}
    tabu_iter = 0
    temperature = T0
    iter_sin_mejora = 0

    while temperature > T_MIN and iter_sin_mejora < UMBRAL:
        tabu_iter += 1
        expired = [k for k, v in tabu.items() if v <= tabu_iter]
        for k in expired:
            del tabu[k]

        ratio = candidate_ratio(temperature)
        neighborhood = generate_neighborhood(
            client_routes, vertices, station_idxs, ratio, rng
        )
        if not neighborhood:
            break

        best_delta = float("inf")
        chosen: Move | None = None

        for move in neighborhood:
            if any(tabu.get(k, 0) > tabu_iter for k in move.tabu_keys()):
                continue
            delta = delta_approx_move(
                client_routes, move, vertices, arcs, battery_reserve_pct
            )
            if delta < best_delta:
                best_delta = delta
                chosen = move

        if chosen is None:
            iter_sin_mejora += 1
            temperature *= ALPHA_COOLING
            continue

        trial_full = apply_accepted_move_with_dp(
            client_routes,
            chosen,
            vertices,
            arcs,
            station_idxs,
            depot_idxs,
            battery_reserve_pct,
        )
        trial_struct_f = f_gen_solution(trial_full, vertices, arcs, battery_reserve_pct)
        delta_e = trial_struct_f - current_f
        accept = delta_e <= 0 or rng.random() < math.exp(
            -delta_e / max(temperature, 1e-9)
        )

        if accept:
            full_routes = trial_full
            client_routes = strip_client_routes(full_routes, vertices)
            current_f = trial_struct_f
            for tkey in chosen.tabu_keys():
                tabu[tkey] = tabu_iter + K_TABU
            if current_f < best_f - 1e-6:
                best_f = current_f
                best_routes = [list(r) for r in full_routes]
                iter_sin_mejora = 0
            else:
                iter_sin_mejora += 1
        else:
            iter_sin_mejora += 1

        temperature *= ALPHA_COOLING
        if verbose and tabu_iter % 20 == 0:
            print(
                f"  iter={tabu_iter} T={temperature:.4f} f={current_f:.2f} best={best_f:.2f}"
            )

    final_energy = exact_energy_solution(
        best_routes, vertices, arcs, battery_reserve_pct
    )
    return best_routes, final_energy, best_f


def _run_eh_on_instance(
    name: str,
    seed: int,
    battery_reserve_pct: float = 0.0,
) -> dict:
    n_c, n_s = parse_instance_name(name)
    vertices, arcs = load_instance(name)
    t0 = time.time()
    routes, energy, fgen = run_eh_sats(
        vertices, arcs, seed=seed, battery_reserve_pct=battery_reserve_pct
    )
    t_s = time.time() - t0
    vmap = {v.idx: v for v in vertices}
    return dict(
        instance=name,
        customers=n_c,
        stations=n_s,
        energy=round(energy, 2),
        fgen=round(fgen, 2),
        routes=len(routes),
        station_visits=count_station_visits(routes, vmap),
        time_s=round(t_s, 2),
    )


def _run_eh_batch(
    names: list[str],
    seed: int = 42,
    title: str = "EH-SA/TS",
    battery_reserve_pct: float = 0.0,
    show_customers: bool = True,
) -> list[dict]:
    print("\n" + "=" * 80)
    print(title)
    print("=" * 80)
    if show_customers:
        hdr = (
            f"{'Inst':<14} {'Clientes':>8} {'Est':>4} {'Energía':>10} "
            f"{'Rutas':>6} {'VisEst':>7} {'t(s)':>6}"
        )
    else:
        hdr = f"{'Inst':<14} {'Energía':>10} {'Rutas':>6} {'VisEst':>7} {'t(s)':>6}"
    print(hdr)
    print("-" * 80)
    results = []
    for name in names:
        row = _run_eh_on_instance(name, seed, battery_reserve_pct)
        results.append(row)
        if show_customers:
            print(
                f"{row['instance']:<14} {row['customers']:>8} {row['stations']:>4} "
                f"{row['energy']:>10.2f} {row['routes']:>6} "
                f"{row['station_visits']:>7} {row['time_s']:>5.1f}s"
            )
        else:
            print(
                f"{row['instance']:<14} {row['energy']:>10.2f} {row['routes']:>6} "
                f"{row['station_visits']:>7} {row['time_s']:>5.1f}s"
            )
    avg_e = sum(r["energy"] for r in results) / len(results)
    avg_t = sum(r["time_s"] for r in results) / len(results)
    print("-" * 80)
    if show_customers:
        print(
            f"{'Promedio':<14} {'':>8} {'':>4} {avg_e:>10.2f} {'':>6} {'':>7} {avg_t:>5.1f}s"
        )
    else:
        print(f"{'Promedio':<14} {avg_e:>10.2f} {'':>6} {'':>7} {avg_t:>5.1f}s")
    return results


def run_small_instances(seed=42):
    return _run_eh_batch(
        SMALL_INSTANCES,
        seed,
        title="EH-SA/TS - instancias pequeñas (C12R2-C24R2)",
    )


def run_extended_small(seed=42):
    return _run_eh_batch(
        SMALL_INSTANCES_OWN,
        seed,
        title="EH-SA/TS - instancias extendidas (C10R2, C11R2)",
        show_customers=False,
    )


def run_own_bank(seed=42):
    return _run_eh_batch(
        ALL_INSTANCES,
        seed,
        title=f"EH-SA/TS - banco reproducible ({len(ALL_INSTANCES)} instancias)",
    )


def run_large_instances(seed=42):
    return _run_eh_batch(
        LARGE_INSTANCES,
        seed,
        title="EH-SA/TS - instancias grandes (C25-C150)",
    )


def run_recharge_stations(seed=42):
    print("\n" + "=" * 65)
    print("Estaciones de recarga - energía y visitas medias (EH-SA/TS)")
    print("=" * 65)
    print(f"{'Grupo':>6} {'Energía':>12} {'VisEst':>10} {'n':>4}")
    print("-" * 40)
    groups: dict[str, list] = {"R2": [], "R4": [], "R6": [], "R8": []}
    for name in LARGE_INSTANCES:
        _, n_s = parse_instance_name(name)
        key = f"R{n_s}"
        row = _run_eh_on_instance(name, seed)
        groups[key].append((row["energy"], row["station_visits"]))
    results = []
    for g, data in groups.items():
        avg_e = sum(d[0] for d in data) / len(data)
        avg_v = sum(d[1] for d in data) / len(data)
        print(f"{g:>6} {avg_e:>12.2f} {avg_v:>10.2f} {len(data):>4}")
        results.append(
            dict(group=g, energy=round(avg_e, 2), visits=round(avg_v, 2), n=len(data))
        )
    return results


def run_battery_reserve(seed=42):
    print("\n" + "=" * 65)
    print("Reserva de batería - energía y visitas medias (EH-SA/TS)")
    print("=" * 65)
    print("Reserva: 0% → 110 kWh usable; 10% → 99 kWh; 20% → 88 kWh.")
    print(f"{'Nivel':>6} {'Usable':>8} {'Energía':>12} {'VisEst':>10}")
    print("-" * 45)
    results = []
    for label, reserve_pct in BATTERY_RESERVE_LEVELS.items():
        usable = BATTERY_CAPACITY_KWH * (1.0 - reserve_pct / 100.0)
        energies, visits_list = [], []
        for name in LARGE_INSTANCES:
            row = _run_eh_on_instance(name, seed, battery_reserve_pct=reserve_pct)
            energies.append(row["energy"])
            visits_list.append(row["station_visits"])
        avg_e = sum(energies) / len(energies)
        avg_v = sum(visits_list) / len(visits_list)
        print(f"{label:>6} {usable:>7.0f} {avg_e:>12.2f} {avg_v:>10.2f}")
        results.append(
            dict(
                threshold=label,
                usable_kwh=round(usable, 1),
                reserve_pct=reserve_pct,
                energy=round(avg_e, 2),
                visits=round(avg_v, 2),
            )
        )
    return results


def run_energy_vs_distance(seed=42):
    print("\n" + "=" * 90)
    print("Energía vs distancia — consumo bajo cada objetivo (EH-SA/TS)")
    print("=" * 90)
    hdr = f"{'Inst':<14} {'E_min':>10} {'E_dist':>10} {'% inc':>10} {'t(s)':>6}"
    print(hdr)
    print("-" * 55)
    results = []
    for name in LARGE_INSTANCES:
        vertices, arcs = load_instance(name)
        t0 = time.time()
        _, energy_min, _ = run_eh_sats(vertices, arcs, seed=seed)
        _, energy_from_dist = solve_distance_minimizing(vertices, arcs, seed=seed + 1)
        t_s = time.time() - t0
        pct = relative_gap(energy_from_dist, energy_min)
        row = dict(
            instance=name,
            energy_min=round(energy_min, 2),
            energy_dist=round(energy_from_dist, 2),
            pct_inc=round(pct, 2),
            time_s=round(t_s, 2),
        )
        results.append(row)
        print(
            f"{name:<14} {energy_min:>10.2f} {energy_from_dist:>10.2f} "
            f"{pct:>9.2f}% {t_s:>5.1f}s"
        )
    avg_pct = sum(r["pct_inc"] for r in results) / len(results)
    print("-" * 55)
    print(f"{'Promedio':<14} {'':>10} {'':>10} {avg_pct:>9.2f}%")
    print("\n%E inc = (E_dist - E_min) / E_min × 100%")
    return results


def save_results(
    small,
    large,
    recharge_stations,
    battery_reserve,
    energy_vs_distance,
    path="benchmarks.txt",
    extended=None,
    bank=None,
):
    lines = [
        "RESULTADOS EH-SA/TS",
        "",
    ]

    def _batch_section(title: str, rows: list[dict]) -> None:
        if not rows:
            return
        lines.append(title)
        lines.append("-" * 60)
        for r in rows:
            lines.append(
                f"{r['instance']}: E={r['energy']} kWh rutas={r['routes']} "
                f"vis_est={r['station_visits']} t={r['time_s']}s"
            )
        lines.append("")

    _batch_section("INSTANCIAS PEQUEÑAS", small)
    _batch_section("INSTANCIAS GRANDES", large)
    if recharge_stations:
        lines.append("ESTACIONES DE RECARGA (Tabla 4 paper)")
        for r in recharge_stations:
            lines.append(f"{r['group']}: E={r['energy']} vis={r['visits']} n={r['n']}")
        lines.append("")
    if battery_reserve:
        lines.append("RESERVA DE BATERÍA (Tabla 5 paper)")
        for r in battery_reserve:
            lines.append(
                f"{r['threshold']}: E={r['energy']} vis={r['visits']} usable={r['usable_kwh']} kWh"
            )
        lines.append("")
    if energy_vs_distance:
        lines.append("ENERGÍA vs DISTANCIA (Tabla 6 paper)")
        for r in energy_vs_distance:
            lines.append(
                f"{r['instance']}: E_min={r['energy_min']} E_dist={r['energy_dist']} "
                f"inc={r['pct_inc']}%"
            )
        lines.append("")
    _batch_section("EXTENDIDAS", extended or [])
    _batch_section("BANCO COMPLETO", bank or [])

    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))
    print(f"\nResultados guardados en: {path}")


def print_usage():
    print("Uso: python main.py [modo] [--seed N] [--save]")
    print("")
    print("Modos:")
    print("  all          Ejecutar todos los benchmarks EH-SA/TS (default)")
    print("  small        Instancias pequeñas C12R2-C24R2")
    print("  large        Instancias grandes C25-C150")
    print("  recharge-stations    Estaciones de recarga")
    print("  battery-reserve      Reserva de batería")
    print("  energy-vs-distance   Objetivo energía vs distancia")
    print("  extended     C10R2, C11R2")
    print("  bank         Todas las instancias del banco (55)")
    print("  single NAME  Una instancia (ej: C25R2-1)")
    print("")
    print("Opciones:")
    print("  --seed N     Semilla aleatoria (default: 42)")
    print("  --save       Guardar benchmarks.txt con todos los resultados")
    print("")
    print("Ejemplos:")
    print("  python main.py single C12R2")
    print("  python main.py small --save")


def main():
    args = sys.argv[1:]
    mode = "all"
    seed = 42
    save = False
    single_name = None

    i = 0
    while i < len(args):
        if args[i] in ("--help", "-h"):
            print_usage()
            return
        if args[i] == "--seed" and i + 1 < len(args):
            seed = int(args[i + 1])
            i += 2
        elif args[i] == "--save":
            save = True
            i += 1
        elif args[i] == "single" and i + 1 < len(args):
            mode = "single"
            single_name = args[i + 1]
            i += 2
        else:
            mode = args[i]
            i += 1

    print("=" * 70)
    print("EH-SA/TS - Enhanced Hybrid Simulated Annealing / Tabu Search")
    print("EVRP - Electric Vehicle Routing Problem")
    print("=" * 70)
    print(f"Modo: {mode} | Semilla: {seed}")

    if mode == "single":
        if single_name is None:
            print("Error: especifique el nombre. Ej: python main.py single C25R2-1")
            return
        print(f"\nEjecutando instancia: {single_name}")
        row = _run_eh_on_instance(single_name, seed)
        print(f"Energía EH-SA/TS: {row['energy']:.4f} kWh")
        print(f"f_gen:            {row['fgen']:.4f}")
        print(f"Tiempo:           {row['time_s']:.2f}s")
        print(f"Rutas:            {row['routes']}")
        print(f"Visitas estación: {row['station_visits']}")
        return

    small, large, recharge_stations, battery_reserve, energy_vs_distance = (
        [],
        [],
        [],
        [],
        [],
    )
    extended, bank = [], []
    if mode in ("all", "small"):
        small = run_small_instances(seed)
    if mode in ("all", "extended"):
        extended = run_extended_small(seed)
    if mode == "bank":
        bank = run_own_bank(seed)
    if mode in ("all", "large"):
        large = run_large_instances(seed)
    if mode in ("all", "recharge-stations"):
        recharge_stations = run_recharge_stations(seed)
    if mode in ("all", "battery-reserve"):
        battery_reserve = run_battery_reserve(seed)
    if mode in ("all", "energy-vs-distance"):
        energy_vs_distance = run_energy_vs_distance(seed)
    if save and (
        small
        or large
        or recharge_stations
        or battery_reserve
        or energy_vs_distance
        or extended
        or bank
    ):
        save_results(
            small,
            large,
            recharge_stations,
            battery_reserve,
            energy_vs_distance,
            extended=extended,
            bank=bank,
        )
    print("\n Experimento completado :D")


if __name__ == "__main__":
    main()
