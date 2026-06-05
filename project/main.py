from __future__ import annotations

import copy
import heapq
import json
import math
import random
import re
import statistics
import sys
import time
from contextlib import contextmanager
from dataclasses import dataclass
from enum import Enum, auto
from pathlib import Path

INSTANCES_DIR = Path(__file__).parent / "instances"
LOGS_DIR = Path(__file__).parent / "logs"
STATS_DIR = Path(__file__).parent / "stats"

try:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    MATPLOTLIB_AVAILABLE = True
except ImportError:
    plt = None
    MATPLOTLIB_AVAILABLE = False

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
MAX_LABELS_DP = 20
MAX_NEIGHBORHOOD_MOVES = 2000
GAMMA_CAP = 200.0
GAMMA_BATT = 100.0
GAMMA_MISS = 5000.0
UMBRAL = 100

LARGE_NUM_RUNS_DEFAULT = 10
LARGE_RUN_TIME_LIMIT_S = 300.0
INIT_TIME_MAX_S = 90.0
INIT_TIME_FRACTION = 0.30

_DEADLINE: float | None = None

MAX_DIJKSTRA_POPS = 200_000


class _DeadlineReached(Exception):
    """Señal interna para abortar la búsqueda cuando se agota el tiempo por corrida."""


def _deadline_passed() -> bool:
    return _DEADLINE is not None and time.time() >= _DEADLINE


BATTERY_RESERVE_LEVELS = {"0%": 0.0, "10%": 10.0, "20%": 20.0}

SMALL_INSTANCES = [f"C{n}R2" for n in range(10, 25)]
SMALL_INSTANCES_EXTENDED = SMALL_INSTANCES
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
    """Gap relativo (Sol - ref) / ref x 100%"""
    return (solution - reference) / reference * 100.0 if reference > 0 else 0.0


def absolute_gap(solution: float, reference: float) -> float:
    return relative_gap(solution, reference)


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


def evaluate_route_strict(
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
        if batt + 1e-9 < e:
            batt_viol += e - batt
            total_e += e
            batt = 0.0
        else:
            total_e += e
            batt -= e
        if is_charge_node(vertices[j]):
            batt = BATTERY_CAPACITY_KWH
        elif batt + 1e-9 < reserve_kwh:
            batt_viol += reserve_kwh - batt
            batt = reserve_kwh
    return total_e, batt_viol


def battery_after_route_prefix(
    seq: list[int],
    vertices: dict[int, Vertex],
    arcs: dict,
    reserve_pct: float = 0.0,
    client_seq: list[int] | None = None,
) -> float:
    """Batería al final de seq bajo las mismas reglas que evaluate_route_strict."""
    if len(seq) < 2:
        return BATTERY_CAPACITY_KWH
    if client_seq is None:
        client_seq = seq
    reserve_kwh = battery_reserve_kwh(reserve_pct)
    load_kg = route_total_load_kg(client_seq, vertices)
    batt = BATTERY_CAPACITY_KWH
    for step in range(len(seq) - 1):
        i, j = seq[step], seq[step + 1]
        if vertices[i].is_customer:
            load_kg -= vertices[i].demand_tons * 1000.0
        e = arc_energy_kwh(arcs, i, j, max(0.0, load_kg))
        if batt + 1e-9 < e:
            batt = 0.0
        else:
            batt -= e
        if is_charge_node(vertices[j]):
            batt = BATTERY_CAPACITY_KWH
        elif batt + 1e-9 < reserve_kwh:
            batt = reserve_kwh
    return batt


def strict_segment_violation_kwh(
    seg_path: list[int],
    load_kg: float,
    start_batt: float,
    vertices: dict[int, Vertex],
    arcs: dict,
    reserve_pct: float = 0.0,
) -> tuple[float, float]:
    """Violación de batería y batería final al recorrer seg_path desde start_batt."""
    if len(seg_path) < 2:
        return 0.0, start_batt
    reserve_kwh = battery_reserve_kwh(reserve_pct)
    load = load_kg
    batt = start_batt
    viol = 0.0
    for step in range(len(seg_path) - 1):
        i, j = seg_path[step], seg_path[step + 1]
        if vertices[i].is_customer:
            load -= vertices[i].demand_tons * 1000.0
        e = arc_energy_kwh(arcs, i, j, max(0.0, load))
        if batt + 1e-9 < e:
            viol += e - batt
            batt = 0.0
        else:
            batt -= e
        if is_charge_node(vertices[j]):
            batt = BATTERY_CAPACITY_KWH
        elif batt + 1e-9 < reserve_kwh:
            viol += reserve_kwh - batt
            batt = reserve_kwh
    return viol, batt


def repair_route_battery(
    seq: list[int],
    vertices: dict[int, Vertex],
    arcs: dict,
    station_idxs: list[int],
    depot_idxs: list[int],
    reserve_pct: float = 0.0,
    max_passes: int = 64,
) -> list[int]:
    """Inserta recargas en la primera violación estricta detectada."""
    if len(seq) < 2:
        return list(seq)
    route = list(seq)
    all_charge = list(dict.fromkeys(station_idxs + depot_idxs))
    reserve_kwh = battery_reserve_kwh(reserve_pct)

    for _ in range(max_passes):
        _, viol = evaluate_route_strict(route, vertices, arcs, reserve_pct)
        if viol <= 1e-6:
            return route

        load_kg = route_total_load_kg(route, vertices)
        batt = BATTERY_CAPACITY_KWH
        fixed = False
        for step in range(len(route) - 1):
            i, j = route[step], route[step + 1]
            if vertices[i].is_customer:
                load_kg -= vertices[i].demand_tons * 1000.0
            e = arc_energy_kwh(arcs, i, j, max(0.0, load_kg))
            need = e if batt + 1e-9 < e else 0.0
            if need <= 1e-9:
                batt -= e
                if is_charge_node(vertices[j]):
                    batt = BATTERY_CAPACITY_KWH
                elif batt + 1e-9 < reserve_kwh:
                    need = reserve_kwh - batt
            if need <= 1e-9:
                continue

            load_before = load_kg
            repl = _charge_detour_to(
                i, j, load_before, batt, vertices, arcs, all_charge, reserve_kwh
            )
            if repl is None:
                via = _two_hop_via_charge_nodes(
                    i,
                    j,
                    load_before,
                    batt,
                    vertices,
                    arcs,
                    all_charge,
                    reserve_kwh,
                    min_end_batt=0.0,
                )
                if via is not None:
                    repl = (0.0, [i] + via[0], via[1])
            if repl is None:
                e_dep, p_dep, b_dep = min_energy_path(
                    i,
                    depot_idxs[0],
                    load_before,
                    batt,
                    vertices,
                    arcs,
                    all_charge,
                    0.0,
                    reserve_kwh,
                )
                if e_dep < 1e17:
                    e_fwd, p_fwd, _ = min_energy_path(
                        depot_idxs[0],
                        j,
                        load_before,
                        b_dep,
                        vertices,
                        arcs,
                        all_charge,
                        0.0,
                        reserve_kwh,
                    )
                    if e_fwd < 1e17:
                        repl = (e_dep + e_fwd, p_dep + p_fwd[1:], 0.0)
            if repl is None:
                batt = max(0.0, batt - e)
                if is_charge_node(vertices[j]):
                    batt = BATTERY_CAPACITY_KWH
                continue

            _, path, _ = repl
            route = route[: step + 1] + path[1:] + route[step + 2 :]
            fixed = True
            break
        if not fixed:
            break
    return route


def validate_solution(
    routes: list[list[int]],
    vertices: dict[int, Vertex],
    arcs: dict,
    reserve_pct: float = 0.0,
) -> dict:
    customer_ids = {v.idx for v in vertices.values() if v.is_customer}
    visited: list[int] = []
    battery_violation = 0.0
    capacity_violation = 0.0
    energy = 0.0
    for route in routes:
        visited.extend(i for i in route if vertices[i].is_customer)
        route_energy, route_battery_violation = evaluate_route_strict(
            route, vertices, arcs, reserve_pct
        )
        energy += route_energy
        battery_violation += route_battery_violation
        capacity_violation += capacity_violation_kg(route, vertices)
    missing = sorted(customer_ids - set(visited))
    duplicate_visits = len(visited) - len(set(visited))
    feasible = (
        not missing
        and duplicate_visits == 0
        and battery_violation <= 1e-6
        and capacity_violation <= 1e-6
    )
    return {
        "feasible": feasible,
        "energy_kwh": energy,
        "missing_customers": missing,
        "duplicate_visits": duplicate_visits,
        "battery_violation_kwh": battery_violation,
        "capacity_violation_kg": capacity_violation,
        "customers_served": len(set(visited)),
        "customers_total": len(customer_ids),
    }


def _infeasibility_reasons(report: dict) -> list[str]:
    reasons: list[str] = []
    if report["missing_customers"]:
        reasons.append(f"clientes faltantes ({len(report['missing_customers'])})")
    if report["duplicate_visits"] > 0:
        reasons.append(f"visitas duplicadas ({report['duplicate_visits']})")
    if report["battery_violation_kwh"] > 1e-6:
        reasons.append(f"violación batería ({report['battery_violation_kwh']:.2f} kWh)")
    if report["capacity_violation_kg"] > 1e-6:
        reasons.append(
            f"violación capacidad ({report['capacity_violation_kg']:.2f} kg)"
        )
    return reasons or ["desconocida"]


def _format_missing_customers(missing: list[int], limit: int = 12) -> str:
    if not missing:
        return "ninguno"
    if len(missing) <= limit:
        return str(missing)
    head = ", ".join(str(i) for i in missing[:limit])
    return f"[{head}, ... +{len(missing) - limit} más]"


def log_infeasible_candidate(
    report: dict,
    *,
    fgen: float,
    instance: str | None = None,
    routes_count: int | None = None,
    station_visits: int | None = None,
    feasible_alternative: dict | None = None,
    indent: str = "",
) -> None:
    label = "[MEJOR CANDIDATO INFACTIBLE]"
    inst_part = f" {instance}" if instance else ""
    reasons = ", ".join(_infeasibility_reasons(report))
    print(f"{indent}{label}{inst_part} (no reportada como energía válida)")
    print(
        f"{indent}  f_gen={fgen:.2f} | energía simulada={report['energy_kwh']:.2f} kWh"
    )
    print(
        f"{indent}  clientes={report['customers_served']}/{report['customers_total']} | "
        f"faltantes={_format_missing_customers(report['missing_customers'])} | "
        f"duplicados={report['duplicate_visits']}"
    )
    print(
        f"{indent}  viol_batería={report['battery_violation_kwh']:.4f} kWh | "
        f"viol_capacidad={report['capacity_violation_kg']:.4f} kg"
    )
    if routes_count is not None or station_visits is not None:
        parts = []
        if routes_count is not None:
            parts.append(f"rutas={routes_count}")
        if station_visits is not None:
            parts.append(f"visitas_estaciones={station_visits}")
        print(f"{indent}  {' | '.join(parts)}")
    print(f"{indent}  causas: {reasons}")
    if feasible_alternative is not None:
        alt = feasible_alternative["validation"]
        print(
            f"{indent}[MEJOR CANDIDATO FACTIBLE EN BÚSQUEDA] "
            f"f_gen={feasible_alternative['f']:.2f} | "
            f"energía={alt['energy_kwh']:.2f} kWh | "
            f"clientes={alt['customers_served']}/{alt['customers_total']}"
        )


def _update_feasibility_trackers(
    routes: list[list[int]],
    f_val: float,
    vertices: dict[int, Vertex],
    arcs: dict,
    reserve_pct: float,
    best_feasible: dict | None,
    best_infeasible: dict | None,
) -> tuple[dict | None, dict | None]:
    report = validate_solution(routes, vertices, arcs, reserve_pct)
    entry = {"routes": routes, "f": f_val, "validation": report}
    if report["feasible"]:
        if best_feasible is None or f_val < best_feasible["f"] - 1e-6:
            best_feasible = entry
    elif best_infeasible is None or f_val < best_infeasible["f"] - 1e-6:
        best_infeasible = entry
    return best_feasible, best_infeasible


def _reconstruct_path(
    predecessor: dict[tuple[int, float], tuple[int, float]], key: tuple[int, float]
) -> list[int]:
    path = [key[0]]
    state = key
    while state in predecessor:
        state = predecessor[state]
        path.append(state[0])
    path.reverse()
    return path


def _pareto_segment_arrivals(
    arrivals: list[tuple[float, float, list[int]]],
) -> list[tuple[float, list[int], float]]:
    if not arrivals:
        return []
    kept: list[tuple[float, list[int], float]] = []
    for cost, end_batt, path in arrivals:
        dominated = False
        for other_cost, other_batt, other_path in arrivals:
            if other_cost == cost and other_batt == end_batt and other_path is path:
                continue
            if other_cost <= cost + 1e-6 and other_batt >= end_batt - 1e-6:
                if other_cost < cost - 1e-6 or other_batt > end_batt + 1e-6:
                    dominated = True
                    break
        if not dominated:
            kept.append((cost, path, end_batt))
    return kept


def segment_path_options(
    from_i: int,
    to_j: int,
    load_kg: float,
    start_batt: float,
    vertices: dict[int, Vertex],
    arcs: dict,
    charge_nodes: list[int],
    min_end_batt: float = 0.0,
    reserve_kwh: float = 0.0,
) -> list[tuple[float, list[int], float]]:
    if _DEADLINE is not None and time.time() >= _DEADLINE:
        raise _DeadlineReached()
    direct = _segment_path_search_core(
        from_i,
        to_j,
        load_kg,
        start_batt,
        vertices,
        arcs,
        charge_nodes,
        min_end_batt,
        reserve_kwh,
    )
    if direct:
        return direct

    if min_end_batt > 0.0:
        ordered = [s for s in charge_nodes if vertices[s].is_station] + [
            s for s in charge_nodes if vertices[s].is_depot
        ]
        detours: list[tuple[float, float, list[int]]] = []
        for st in ordered:
            if st == from_i:
                continue
            first_legs = _segment_path_search_core(
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
            for cost1, path1, batt1 in first_legs:
                second_legs = _segment_path_search_core(
                    st,
                    to_j,
                    load_kg,
                    batt1,
                    vertices,
                    arcs,
                    charge_nodes,
                    min_end_batt,
                    reserve_kwh,
                )
                for cost2, path2, batt2 in second_legs:
                    detours.append((cost1 + cost2, batt2, path1 + path2[1:]))
        return _pareto_segment_arrivals(detours)

    return []


def _segment_path_search_core(
    from_i: int,
    to_j: int,
    load_kg: float,
    start_batt: float,
    vertices: dict[int, Vertex],
    arcs: dict,
    charge_nodes: list[int],
    min_end_batt: float,
    reserve_kwh: float,
) -> list[tuple[float, list[int], float]]:
    if from_i == to_j:
        need = (
            max(min_end_batt, reserve_kwh)
            if not is_charge_node(vertices[from_i])
            else 0.0
        )
        if start_batt + 1e-9 >= need:
            return _pareto_segment_arrivals([(0.0, start_batt, [from_i])])
        return []

    allowed = set(charge_nodes) | {from_i, to_j}
    best_cost: dict[tuple[int, float], float] = {}
    predecessor: dict[tuple[int, float], tuple[int, float]] = {}
    pq: list[tuple[float, int, float]] = []
    start_key = (from_i, round(start_batt, 4))
    best_cost[start_key] = 0.0
    heapq.heappush(pq, (0.0, from_i, start_batt))
    arrivals: list[tuple[float, float, list[int]]] = []
    need_at_dest = (
        max(min_end_batt, reserve_kwh) if not is_charge_node(vertices[to_j]) else 0.0
    )

    pops = 0
    while pq:
        pops += 1
        if pops > MAX_DIJKSTRA_POPS:
            break
        cost, u, batt = heapq.heappop(pq)
        key = (u, round(batt, 4))
        if best_cost.get(key, 1e18) < cost - 1e-9:
            continue
        if u == to_j:
            if batt + 1e-9 >= need_at_dest:
                arrivals.append((cost, batt, _reconstruct_path(predecessor, key)))
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

    return _pareto_segment_arrivals(arrivals)


def min_start_battery_for_segment(
    from_i: int,
    to_j: int,
    load_kg: float,
    vertices: dict[int, Vertex],
    arcs: dict,
    charge_nodes: list[int],
    min_end_batt: float = 0.0,
    reserve_kwh: float = 0.0,
) -> float:
    lo, hi = 0.0, BATTERY_CAPACITY_KWH
    for _ in range(24):
        mid = (lo + hi) * 0.5
        if segment_path_options(
            from_i,
            to_j,
            load_kg,
            mid,
            vertices,
            arcs,
            charge_nodes,
            min_end_batt,
            reserve_kwh,
        ):
            hi = mid
        else:
            lo = mid
    return hi


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
    options = segment_path_options(
        from_i,
        to_j,
        load_kg,
        start_batt,
        vertices,
        arcs,
        charge_nodes,
        min_end_batt,
        reserve_kwh,
    )
    if not options:
        return 1e18, [], 0.0
    cost, path, end_batt = min(options, key=lambda item: item[0])
    return cost, path, end_batt


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


def _segment_need_end_battery(
    client_seq: list[int],
    target_idx: int,
    vertices: dict[int, Vertex],
    arcs: dict,
    station_idxs: list[int],
    depot_idxs: list[int],
    reserve_kwh: float,
) -> float:
    if target_idx + 1 >= len(client_seq):
        return 0.0
    target_node = client_seq[target_idx]
    next_node = client_seq[target_idx + 1]
    if not vertices[target_node].is_customer:
        return 0.0
    next_load = load_leaving_position(client_seq, target_idx, vertices)
    next_seg_nodes = charge_nodes_for_segment(next_node, station_idxs, depot_idxs)
    return min_start_battery_for_segment(
        target_node,
        next_node,
        next_load,
        vertices,
        arcs,
        next_seg_nodes,
        min_end_batt=0.0,
        reserve_kwh=reserve_kwh,
    )


def _two_hop_via_charge_nodes(
    current: int,
    next_cust: int,
    load: float,
    batt: float,
    vertices: dict[int, Vertex],
    arcs: dict,
    charge_nodes: list[int],
    reserve_kwh: float,
    min_end_batt: float = 0.0,
) -> tuple[list[int], float] | None:
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
            charge_nodes,
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
            charge_nodes,
            min_end_batt=min_end_batt,
            reserve_kwh=reserve_kwh,
        )
        if e2 >= 1e17:
            continue
        if e1 + e2 < best_cost:
            best_cost = e1 + e2
            best_seg = p1[1:] + p2[1:]
            best_batt = b2
    if best_seg is None:
        return None
    return best_seg, best_batt


def greedy_station_insert_route(
    client_seq: list[int],
    vertices: dict[int, Vertex],
    arcs: dict,
    station_idxs: list[int],
    depot_idxs: list[int],
    reserve_pct: float = 0.0,
    _depth: int = 0,
) -> list[int]:
    """Paper §4.1.1 step 3: insert stations when battery cannot reach next customer."""
    if len(client_seq) < 2:
        return list(client_seq)
    if _depth > 12:
        return list(client_seq)
    reserve_kwh = battery_reserve_kwh(reserve_pct)
    charge_nodes = charge_nodes_for_segment(client_seq[-1], station_idxs, depot_idxs)
    all_charge_nodes = list(dict.fromkeys(station_idxs + depot_idxs))
    full = [client_seq[0]]
    batt = BATTERY_CAPACITY_KWH
    cust_idx = 0
    depot_resets = 0
    max_depot_resets = max(32, len(client_seq) * 4)

    def advance_to_next(next_cust: int, seg_path: list[int], end_batt: float) -> bool:
        nonlocal full, batt, cust_idx
        if not isinstance(seg_path, list) or len(seg_path) < 2:
            return False
        if seg_path[-1] != next_cust:
            return False
        if seg_path[0] == full[-1]:
            full.extend(seg_path[1:])
        elif seg_path[0] in (full[-1], depot_idxs[0]):
            full.extend(seg_path[1:] if seg_path[0] == full[-1] else seg_path)
        else:
            return False
        batt = end_batt
        if full[-1] != next_cust:
            return False
        cust_idx += 1
        return True

    def try_return_to_depot() -> bool:
        nonlocal full, batt, depot_resets
        depot_start = depot_idxs[0]
        if full[-1] == depot_start:
            batt = BATTERY_CAPACITY_KWH
            return True
        load = load_leaving_position(client_seq, cust_idx, vertices)
        e, path, return_batt = min_energy_path(
            full[-1],
            depot_start,
            load,
            batt,
            vertices,
            arcs,
            all_charge_nodes,
            reserve_kwh=reserve_kwh,
        )
        if e >= 1e17 or not path:
            return False
        full.extend(path[1:])
        batt = (
            BATTERY_CAPACITY_KWH
            if is_charge_node(vertices[depot_start])
            else return_batt
        )
        depot_resets += 1
        return True

    def finish_via_depot_suffix() -> list[int]:
        depot_start = depot_idxs[0]
        suffix_seq = [depot_start] + client_seq[cust_idx + 1 :]
        if len(suffix_seq) < 2:
            return full
        tail = greedy_station_insert_route(
            suffix_seq,
            vertices,
            arcs,
            station_idxs,
            depot_idxs,
            reserve_pct,
            _depth=_depth + 1,
        )
        full.extend(tail[1:])
        return full

    while cust_idx < len(client_seq) - 1:
        current = full[-1]
        next_cust = client_seq[cust_idx + 1]
        load = load_leaving_position(client_seq, cust_idx, vertices)
        seg_nodes = charge_nodes_for_segment(next_cust, station_idxs, depot_idxs)
        need_end = _segment_need_end_battery(
            client_seq,
            cust_idx + 1,
            vertices,
            arcs,
            station_idxs,
            depot_idxs,
            reserve_kwh,
        )
        segment_options = segment_path_options(
            current,
            next_cust,
            load,
            batt,
            vertices,
            arcs,
            seg_nodes,
            min_end_batt=need_end,
            reserve_kwh=reserve_kwh,
        )
        if segment_options:
            advanced = False
            for seg_e, seg_path, end_batt in sorted(
                segment_options, key=lambda item: item[0]
            ):
                if advance_to_next(next_cust, seg_path, end_batt):
                    advanced = True
                    break
            if advanced:
                continue

        via_charge = _two_hop_via_charge_nodes(
            current,
            next_cust,
            load,
            batt,
            vertices,
            arcs,
            seg_nodes,
            reserve_kwh,
            min_end_batt=need_end,
        )
        if via_charge is not None:
            seg_path = [current] + via_charge[0]
            if advance_to_next(next_cust, seg_path, via_charge[1]):
                continue

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
            _, path, end_batt = detour
            if advance_to_next(next_cust, path, end_batt):
                continue

        depot_start = depot_idxs[0]
        return_e, return_path, return_batt = min_energy_path(
            current,
            depot_start,
            load,
            batt,
            vertices,
            arcs,
            seg_nodes,
            reserve_kwh=reserve_kwh,
        )
        if return_e < 1e17:
            forward_e, forward_path, forward_batt = min_energy_path(
                depot_start,
                next_cust,
                load,
                return_batt,
                vertices,
                arcs,
                seg_nodes,
                min_end_batt=need_end,
                reserve_kwh=reserve_kwh,
            )
            if forward_e < 1e17:
                full.extend(return_path[1:])
                if advance_to_next(next_cust, forward_path, forward_batt):
                    continue

        via_all = _two_hop_via_charge_nodes(
            current,
            next_cust,
            load,
            batt,
            vertices,
            arcs,
            all_charge_nodes,
            reserve_kwh,
            min_end_batt=need_end,
        )
        if via_all is not None:
            seg_path = [current] + via_all[0]
            if advance_to_next(next_cust, seg_path, via_all[1]):
                continue

        relaxed = segment_path_options(
            current,
            next_cust,
            load,
            batt,
            vertices,
            arcs,
            seg_nodes,
            min_end_batt=0.0,
            reserve_kwh=reserve_kwh,
        )
        if relaxed:
            advanced = False
            for seg_e, seg_path, end_batt in sorted(relaxed, key=lambda item: item[0]):
                if advance_to_next(next_cust, seg_path, end_batt):
                    advanced = True
                    break
            if advanced:
                continue

        if depot_resets < max_depot_resets and try_return_to_depot():
            continue

        e_force, path_force, b_force = min_energy_path(
            current,
            next_cust,
            load,
            batt,
            vertices,
            arcs,
            all_charge_nodes,
            min_end_batt=0.0,
            reserve_kwh=reserve_kwh,
        )
        if e_force < 1e17 and advance_to_next(next_cust, path_force, b_force):
            continue
        if e_force < 1e17:
            if path_force[0] == full[-1]:
                full.extend(path_force[1:])
            else:
                full.extend(path_force)
            batt = b_force
            if full[-1] == next_cust:
                cust_idx += 1
                continue

        return finish_via_depot_suffix()
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
    max_labels: int = MAX_LABELS_DP,
    t_deadline: float | None = None,
) -> list[int]:
    if len(client_seq) < 2:
        return list(client_seq)
    reserve_kwh = battery_reserve_kwh(reserve_pct)
    n = len(client_seq)

    precomp: list[tuple[list[int], float, float]] = []
    for step in range(n - 1):
        if t_deadline is not None and time.time() >= t_deadline:
            raise _DeadlineReached()
        node_j = client_seq[step + 1]
        seg_nodes = charge_nodes_for_segment(node_j, station_idxs, depot_idxs)
        need_end = _segment_need_end_battery(
            client_seq, step + 1, vertices, arcs, station_idxs, depot_idxs, reserve_kwh
        )
        load = load_leaving_position(client_seq, step, vertices)
        precomp.append((seg_nodes, need_end, load))

    labels: dict[int, list[tuple[float, float, list[int]]]] = {
        0: [(BATTERY_CAPACITY_KWH, 0.0, [client_seq[0]])]
    }

    for step in range(n - 1):
        if t_deadline is not None and time.time() >= t_deadline:
            raise _DeadlineReached()
        node_i = client_seq[step]
        node_j = client_seq[step + 1]
        seg_nodes, need_end, load = precomp[step]
        new_labels: list[tuple[float, float, list[int]]] = []
        for batt, cost, path in labels[step]:
            for seg_e, seg_path, end_batt in segment_path_options(
                node_i,
                node_j,
                load,
                batt,
                vertices,
                arcs,
                seg_nodes,
                min_end_batt=need_end,
                reserve_kwh=reserve_kwh,
            ):
                merged = path + seg_path[1:]
                new_labels.append((end_batt, cost + seg_e, merged))
        if not new_labels:
            return greedy_station_insert_route(
                client_seq, vertices, arcs, station_idxs, depot_idxs, reserve_pct
            )
        if len(new_labels) > max_labels * 4:
            new_labels.sort(key=lambda t: (-t[0], t[1]))
            new_labels = new_labels[: max_labels * 2]
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
        if len(pareto) > max_labels:
            pareto.sort(key=lambda t: t[1])
            pareto = pareto[:max_labels]
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


def missing_customers_penalty(
    routes: list[list[int]], vertices: dict[int, Vertex]
) -> float:
    customer_ids = {v.idx for v in vertices.values() if v.is_customer}
    visited: list[int] = []
    for route in routes:
        visited.extend(i for i in route if vertices[i].is_customer)
    missing = len(customer_ids - set(visited))
    duplicates = max(0, len(visited) - len(set(visited)))
    return GAMMA_MISS * (missing + duplicates)


def f_gen_solution(
    routes: list[list[int]],
    vertices: dict[int, Vertex],
    arcs: dict,
    reserve_pct: float = 0.0,
) -> float:
    energy_penalties = sum(f_gen_route(r, vertices, arcs, reserve_pct) for r in routes)
    return energy_penalties + missing_customers_penalty(routes, vertices)


def exact_energy_solution(
    routes: list[list[int]],
    vertices: dict[int, Vertex],
    arcs: dict,
    reserve_pct: float = 0.0,
) -> float:
    report = validate_solution(routes, vertices, arcs, reserve_pct)
    if not report["feasible"]:
        return float("inf")
    return report["energy_kwh"]


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
    t_deadline: float | None = None,
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
                t_deadline=t_deadline,
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
    t_deadline: float | None = None,
) -> list[list[int]]:
    full_routes = []
    for route in client_routes:
        if _deadline_passed():
            full_routes.append(list(route))
            continue
        try:
            inserted = dp_station_insertion(
                route,
                vertices,
                arcs,
                station_idxs,
                depot_idxs,
                reserve_pct,
                t_deadline=t_deadline,
            )
            report = validate_solution([inserted], vertices, arcs, reserve_pct)
            if not report["feasible"]:
                inserted = greedy_station_insert_route(
                    route, vertices, arcs, station_idxs, depot_idxs, reserve_pct
                )
            inserted = repair_route_battery(
                inserted, vertices, arcs, station_idxs, depot_idxs, reserve_pct
            )
        except _DeadlineReached:
            inserted = list(route)
        full_routes.append(inserted)
    return full_routes


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
    cap = min(MAX_NEIGHBORHOOD_MOVES, max(200, int(len(moves) * 0.15)))
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
    instance_name: str | None = None,
    time_limit_s: float | None = None,
    collect_history: bool = False,
) -> tuple[list[list[int]], float, float, dict]:
    vertices = {v.idx: v for v in vertices_list}
    station_idxs = [v.idx for v in vertices_list if v.is_station]
    depot_idxs = [v.idx for v in vertices_list if v.is_depot]
    depot_start = depot_idxs[0]
    depot_end = depot_idxs[-1] if len(depot_idxs) > 1 else depot_idxs[0]

    global _DEADLINE
    t_start = time.time()
    t_deadline = (t_start + time_limit_s) if time_limit_s is not None else None
    if t_deadline is not None:
        init_budget = min(INIT_TIME_MAX_S, time_limit_s * INIT_TIME_FRACTION)
        t_deadline_init = t_start + init_budget
    else:
        t_deadline_init = None
    _DEADLINE = t_deadline_init

    rng = random.Random(seed)
    client_routes = greedy_build(vertices, arcs, depot_start, depot_end, rng)
    full_routes = build_routes_with_dp(
        client_routes,
        vertices,
        arcs,
        station_idxs,
        depot_idxs,
        battery_reserve_pct,
        t_deadline=t_deadline_init,
    )
    _DEADLINE = t_deadline
    client_routes = strip_client_routes(full_routes, vertices)
    current_f = f_gen_solution(full_routes, vertices, arcs, battery_reserve_pct)

    best_routes = [list(r) for r in full_routes]
    best_f = current_f
    best_feasible: dict | None = None
    best_infeasible: dict | None = None
    best_feasible, best_infeasible = _update_feasibility_trackers(
        best_routes,
        best_f,
        vertices,
        arcs,
        battery_reserve_pct,
        best_feasible,
        best_infeasible,
    )
    tabu: dict[tuple[int, int], int] = {}
    tabu_iter = 0
    temperature = T0
    iter_sin_mejora = 0
    history: list[dict] = []

    while temperature > T_MIN and iter_sin_mejora < UMBRAL:
        if t_deadline is not None and time.time() >= t_deadline:
            break
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
            if t_deadline is not None and time.time() >= t_deadline:
                break
            if any(tabu.get(k, 0) > tabu_iter for k in move.tabu_keys()):
                continue
            delta = delta_approx_move(
                client_routes, move, vertices, arcs, battery_reserve_pct
            )
            if delta < best_delta:
                best_delta = delta
                chosen = move

        if t_deadline is not None and time.time() >= t_deadline:
            break

        if chosen is None:
            iter_sin_mejora += 1
            temperature *= ALPHA_COOLING
            continue

        try:
            trial_full = apply_accepted_move_with_dp(
                client_routes,
                chosen,
                vertices,
                arcs,
                station_idxs,
                depot_idxs,
                battery_reserve_pct,
                t_deadline=t_deadline,
            )
        except _DeadlineReached:
            break
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
            best_feasible, best_infeasible = _update_feasibility_trackers(
                full_routes,
                current_f,
                vertices,
                arcs,
                battery_reserve_pct,
                best_feasible,
                best_infeasible,
            )
            if current_f < best_f - 1e-6:
                best_f = current_f
                best_routes = [list(r) for r in full_routes]
                iter_sin_mejora = 0
            else:
                iter_sin_mejora += 1
        else:
            iter_sin_mejora += 1

        temperature *= ALPHA_COOLING
        if collect_history:
            history.append(
                {
                    "iter": tabu_iter,
                    "t": round(time.time() - t_start, 3),
                    "current_f": round(current_f, 4),
                    "best_f": round(best_f, 4),
                }
            )
        if verbose and tabu_iter % 20 == 0:
            print(
                f"  iter={tabu_iter} T={temperature:.4f} f={current_f:.2f} best={best_f:.2f}"
            )

    _DEADLINE = None

    final_energy = exact_energy_solution(
        best_routes, vertices, arcs, battery_reserve_pct
    )
    validation = validate_solution(best_routes, vertices, arcs, battery_reserve_pct)
    validation["fgen"] = best_f
    validation["iterations"] = tabu_iter
    validation["history"] = history
    validation["infeasible_candidate"] = None
    validation["feasible_alternative"] = None
    if not validation["feasible"]:
        candidate = best_infeasible or {
            "routes": best_routes,
            "f": best_f,
            "validation": validation,
        }
        validation["infeasible_candidate"] = {
            "fgen": candidate["f"],
            "energy_kwh": candidate["validation"]["energy_kwh"],
            "report": candidate["validation"],
        }
        if best_feasible is not None:
            validation["feasible_alternative"] = {
                "fgen": best_feasible["f"],
                "energy_kwh": best_feasible["validation"]["energy_kwh"],
                "report": best_feasible["validation"],
            }
        log_infeasible_candidate(
            validation["infeasible_candidate"]["report"],
            fgen=validation["infeasible_candidate"]["fgen"],
            instance=instance_name,
            routes_count=len(best_routes),
            station_visits=count_station_visits(best_routes, vertices),
            feasible_alternative=best_feasible,
            indent="  ",
        )
    return best_routes, final_energy, best_f, validation


def _run_eh_on_instance(
    name: str,
    seed: int,
    battery_reserve_pct: float = 0.0,
    time_limit_s: float | None = None,
    collect_history: bool = False,
) -> dict:
    n_c, n_s = parse_instance_name(name)
    vertices, arcs = load_instance(name)
    t0 = time.time()
    routes, energy, fgen, validation = run_eh_sats(
        vertices,
        arcs,
        seed=seed,
        battery_reserve_pct=battery_reserve_pct,
        instance_name=name,
        time_limit_s=time_limit_s,
        collect_history=collect_history,
    )
    t_s = time.time() - t0
    vmap = {v.idx: v for v in vertices}
    return dict(
        instance=name,
        customers=n_c,
        stations=n_s,
        energy=round(energy, 2) if math.isfinite(energy) else None,
        fgen=round(fgen, 2),
        routes=len(routes),
        eh_paths=[list(r) for r in routes],
        station_visits=count_station_visits(routes, vmap),
        time_s=round(t_s, 2),
        feasible=validation["feasible"],
        customers_served=validation["customers_served"],
        missing_customers=len(validation["missing_customers"]),
        battery_violation_kwh=round(validation["battery_violation_kwh"], 4),
        capacity_violation_kg=round(validation["capacity_violation_kg"], 4),
        iterations=validation.get("iterations", 0),
        history=validation.get("history", []),
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
            energy_text = (
                f"{row['energy']:>10.2f}"
                if row["energy"] is not None
                else f"{'INFACTIBLE':>10}"
            )
            print(
                f"{row['instance']:<14} {row['customers']:>8} {row['stations']:>4} "
                f"{energy_text} {row['routes']:>6} "
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


def _large_run_seeds(base_seed: int, num_runs: int) -> list[int]:
    return [base_seed + i * 9973 for i in range(num_runs)]


def run_small_benchmark(seed: int = 42) -> list[dict]:
    print("\n" + "=" * 80)
    print("MODO -small | INSTANCIAS PEQUEÑAS (C10R2-C24R2)")
    print(
        "EH-SA/TS (referencia solver MIP: solve_cplex.py → logs/run_NNN_small_cplex.txt)"
    )
    print("=" * 80)
    print(f"Semilla EH: {seed}")
    hdr = (
        f"{'Inst':<14} {'Clientes':>8} {'Est':>4} {'Energía':>10} "
        f"{'Rutas':>6} {'VisEst':>7} {'t(s)':>6}"
    )
    print(hdr)
    print("-" * 80)

    results = []
    for name in SMALL_INSTANCES:
        row = _run_eh_on_instance(name, seed, collect_history=True)
        row["eh_routes"] = row.get("eh_paths", [])
        results.append(row)
        energy_text = (
            f"{row['energy']:>10.2f}"
            if row.get("feasible") and row["energy"] is not None
            else f"{'INFACTIBLE':>10}"
        )
        print(
            f"{row['instance']:<14} {row['customers']:>8} {row['stations']:>4} "
            f"{energy_text} {row['routes']:>6} "
            f"{row['station_visits']:>7} {row['time_s']:>5.1f}s"
        )

    feasible = [r for r in results if r.get("feasible") and r["energy"] is not None]
    print("-" * 80)
    if feasible:
        avg_e = sum(r["energy"] for r in feasible) / len(feasible)
        avg_t = sum(r["time_s"] for r in results) / len(results)
        print(
            f"{'Promedio':<14} {'':>8} {'':>4} {avg_e:>10.2f} {'':>6} {'':>7} {avg_t:>5.1f}s"
        )
    print(f"EH factibles: {len(feasible)}/{len(results)}")

    print("\n--- Resumen (modo -small) ---")
    for r in results:
        if r.get("feasible") and r["energy"] is not None:
            print(f"  {r['instance']}: EH-SA/TS={r['energy']:.2f} kWh")
        else:
            print(
                f"  {r['instance']}: EH-SA/TS=INFACTIBLE "
                f"(faltantes={r['missing_customers']}, "
                f"viol_bat={r['battery_violation_kwh']:.2f} kWh)"
            )

    return results


def run_large_multi_run(
    base_seed: int = 42,
    num_runs: int = LARGE_NUM_RUNS_DEFAULT,
    time_limit_s_per_run: float | None = LARGE_RUN_TIME_LIMIT_S,
) -> list[dict]:
    """Grandes: multi-run EH-SA/TS; RPD relativo vs B* (mejor corrida), estilo Tabla 3."""
    seeds = _large_run_seeds(base_seed, num_runs)
    print("\n" + "=" * 105)
    print("MODO -large | INSTANCIAS GRANDES (C25-C150)")
    print(f"EH-SA/TS: {num_runs} runs, semillas distintas")
    if time_limit_s_per_run is not None:
        print(
            f"Límite por corrida: {time_limit_s_per_run:.0f}s (ajustable con --time-limit-run)"
        )
    print("=" * 105)
    print("Métricas (RPD entre soluciones; solo EH-SA/TS):")
    print("  B*       = min(E_run)  mejor energía entre los runs")
    print("  Media, Desviación Estándar = estadísticas de energía por instancia")
    print("  RPD_run  = (E_run - B*) / B* x 100%")
    print(f"  Semillas: {seeds}")
    hdr = (
        f"{'Inst':<14} {'B*':>10} {'Media':>10} {'Std':>8} "
        f"{'RPD media':>10} {'RPD max':>9} {'t total':>8}"
    )
    print(hdr)
    print("-" * 105)

    summary = []
    for name in LARGE_INSTANCES:
        all_rows: list[dict] = []
        for run_idx, run_seed in enumerate(seeds, start=1):
            print(f"  {name} run {run_idx}/{num_runs}...", end=" ", flush=True)
            row = _run_eh_on_instance(
                name,
                run_seed,
                time_limit_s=time_limit_s_per_run,
                collect_history=True,
            )
            tl = (
                " (TL)"
                if time_limit_s_per_run and row["time_s"] >= time_limit_s_per_run - 1
                else ""
            )
            print(f"{row['time_s']:.1f}s{tl}")
            all_rows.append(row)

        energies = [r["energy"] for r in all_rows]
        times = [r["time_s"] for r in all_rows]
        finite_rows = [
            r
            for r in all_rows
            if r["energy"] is not None and math.isfinite(r["energy"])
        ]
        finite_energies = [r["energy"] for r in finite_rows]
        total_t = sum(times)
        if finite_energies:
            best = min(finite_energies)
            worst_val = max(finite_energies)
            mean_e = statistics.mean(finite_energies)
            std_e = (
                statistics.stdev(finite_energies) if len(finite_energies) > 1 else 0.0
            )
            rpds = [absolute_gap(e, best) for e in finite_energies]
            mean_rpd = statistics.mean(rpds)
            max_rpd = max(rpds)
            best_disp = f"{best:>10.2f}"
            mean_disp = f"{mean_e:>10.2f}"
            std_disp = f"{std_e:>8.2f}"
            rpd_mean_disp = f"{mean_rpd:>9.2f}%"
            rpd_max_disp = f"{max_rpd:>8.2f}%"
            best_row = min(finite_rows, key=lambda r: r["energy"])
            worst_row = max(finite_rows, key=lambda r: r["energy"])
            avg_row = min(finite_rows, key=lambda r: abs(r["energy"] - mean_e))
        else:
            best = worst_val = mean_e = std_e = mean_rpd = max_rpd = float("nan")
            rpds = []
            best_disp = f"{'INFACT':>10}"
            mean_disp = f"{'N/A':>10}"
            std_disp = f"{'N/A':>8}"
            rpd_mean_disp = f"{'N/A':>10}"
            rpd_max_disp = f"{'N/A':>9}"
            best_row = worst_row = avg_row = None

        entry = dict(
            instance=name,
            seeds=seeds,
            energies=energies,
            best=round(best, 2) if finite_energies else None,
            worst=round(worst_val, 2) if finite_energies else None,
            mean=round(mean_e, 2) if finite_energies else None,
            std=round(std_e, 2) if finite_energies else None,
            mean_rpd=round(mean_rpd, 2) if finite_energies else None,
            max_rpd=round(max_rpd, 2) if finite_energies else None,
            total_time_s=round(total_t, 2),
            all_infeasible=not finite_energies,
            best_routes=best_row["eh_paths"] if best_row else None,
            worst_routes=worst_row["eh_paths"] if worst_row else None,
            avg_routes=avg_row["eh_paths"] if avg_row else None,
            best_energy=best_row["energy"] if best_row else None,
            worst_energy=worst_row["energy"] if worst_row else None,
            avg_energy=avg_row["energy"] if avg_row else None,
            best_history=best_row["history"] if best_row else [],
            worst_history=worst_row["history"] if worst_row else [],
        )
        if best_row:
            entry["eh_routes"] = best_row["eh_paths"]
        summary.append(entry)
        print(
            f"{name:<14} {best_disp} {mean_disp} {std_disp} "
            f"{rpd_mean_disp} {rpd_max_disp} {total_t:>7.1f}s"
        )
        run_detail = "  ".join(
            f"R{j+1}={e:.1f}" if e is not None else f"R{j+1}=INF"
            for j, e in enumerate(energies)
        )
        rpd_detail = (
            "  ".join(f"{r:.1f}%" for r in rpds) if rpds else "N/A (sin runs factibles)"
        )
        print(f"  Energias: {run_detail}")
        print(f"  RPD:      {rpd_detail}")

    feasible_summary = [s for s in summary if not s.get("all_infeasible")]
    if feasible_summary:
        avg_best = statistics.mean(s["best"] for s in feasible_summary)
        avg_mean = statistics.mean(s["mean"] for s in feasible_summary)
        avg_std = statistics.mean(s["std"] for s in feasible_summary)
        avg_rpd = statistics.mean(s["mean_rpd"] for s in feasible_summary)
        print("-" * 105)
        print(
            f"{'Promedio':<14} {avg_best:>10.2f} {avg_mean:>10.2f} {avg_std:>8.2f} "
            f"{avg_rpd:>9.2f}%"
        )

    print("\n--- Resumen comparativo (modo -large) ---")
    for s in summary:
        if s.get("all_infeasible"):
            print(f"  {s['instance']}: INFACTIBLE en todos los runs")
        else:
            print(
                f"  {s['instance']}: B*={s['best']:.2f} Media={s['mean']:.2f} "
                f"Std={s['std']:.2f} RPD_media={s['mean_rpd']:.2f}%"
            )

    return summary


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


def run_recharge_stations(seed=42, time_limit_s: float | None = LARGE_RUN_TIME_LIMIT_S):
    print("\n" + "=" * 65)
    print("MODO recharge-stations | Análisis por número de estaciones (EH-SA/TS)")
    if time_limit_s:
        print(f"  Límite por instancia: {time_limit_s:.0f}s")
    print("=" * 65)
    print(f"{'Grupo':>6} {'Energía':>12} {'VisEst':>10} {'n':>4}")
    print("-" * 40)
    groups: dict[str, list] = {"R2": [], "R4": [], "R6": [], "R8": []}
    total = len(LARGE_INSTANCES)
    for idx, name in enumerate(LARGE_INSTANCES, start=1):
        _, n_s = parse_instance_name(name)
        key = f"R{n_s}"
        print(f"  {name} ({idx}/{total})...", end=" ", flush=True)
        row = _run_eh_on_instance(name, seed, time_limit_s=time_limit_s)
        tl_tag = " (TL)" if time_limit_s and row["time_s"] >= time_limit_s - 1 else ""
        print(f"{row['time_s']:.1f}s{tl_tag}")
        groups[key].append((row["energy"] or float("inf"), row["station_visits"]))
    results = []
    for g, data in groups.items():
        avg_e = sum(d[0] for d in data) / len(data)
        avg_v = sum(d[1] for d in data) / len(data)
        print(f"{g:>6} {avg_e:>12.2f} {avg_v:>10.2f} {len(data):>4}")
        results.append(
            dict(group=g, energy=round(avg_e, 2), visits=round(avg_v, 2), n=len(data))
        )
    return results


def run_battery_reserve(seed=42, time_limit_s: float | None = LARGE_RUN_TIME_LIMIT_S):
    print("\n" + "=" * 65)
    print("MODO battery-reserve | Reserva mínima de batería (EH-SA/TS)")
    if time_limit_s:
        print(f"  Límite por instancia: {time_limit_s:.0f}s")
    print("=" * 65)
    print("Reserva: 0% → 110 kWh usable; 10% → 99 kWh; 20% → 88 kWh.")
    print(f"{'Nivel':>6} {'Usable':>8} {'Energía':>12} {'VisEst':>10}")
    print("-" * 45)
    results = []
    total = len(LARGE_INSTANCES)
    for label, reserve_pct in BATTERY_RESERVE_LEVELS.items():
        usable = BATTERY_CAPACITY_KWH * (1.0 - reserve_pct / 100.0)
        energies, visits_list = [], []
        for idx, name in enumerate(LARGE_INSTANCES, start=1):
            print(f"  [{label}] {name} ({idx}/{total})...", end=" ", flush=True)
            row = _run_eh_on_instance(
                name, seed, battery_reserve_pct=reserve_pct, time_limit_s=time_limit_s
            )
            tl_tag = (
                " (TL)" if time_limit_s and row["time_s"] >= time_limit_s - 1 else ""
            )
            print(f"{row['time_s']:.1f}s{tl_tag}")
            energies.append(
                row["energy"] if row["energy"] is not None else float("inf")
            )
            visits_list.append(row["station_visits"])
        finite_e = [e for e in energies if math.isfinite(e)]
        avg_e = sum(finite_e) / len(finite_e) if finite_e else float("nan")
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


def run_energy_vs_distance(
    seed=42, time_limit_s: float | None = LARGE_RUN_TIME_LIMIT_S
):
    print("\n" + "=" * 90)
    print("MODO energy-vs-distance | Minimización energía vs distancia (EH-SA/TS)")
    if time_limit_s:
        print(f"  Límite por instancia (EH): {time_limit_s:.0f}s")
    print("=" * 90)
    hdr = f"{'Inst':<14} {'E_min':>10} {'E_dist':>10} {'% inc':>10} {'t(s)':>6}"
    print(hdr)
    print("-" * 55)
    results = []
    total = len(LARGE_INSTANCES)
    for idx, name in enumerate(LARGE_INSTANCES, start=1):
        print(f"  {name} ({idx}/{total})...", end=" ", flush=True)
        vertices, arcs = load_instance(name)
        t0 = time.time()
        _, energy_min, _, _ = run_eh_sats(
            vertices, arcs, seed=seed, time_limit_s=time_limit_s
        )
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
        tl_tag = " (TL)" if time_limit_s and t_s >= time_limit_s - 1 else ""
        print(f"listo ({t_s:.1f}s{tl_tag})")
        print(
            f"{name:<14} {energy_min:>10.2f} {energy_from_dist:>10.2f} "
            f"{pct:>9.2f}% {t_s:>5.1f}s"
        )
    avg_pct = sum(r["pct_inc"] for r in results) / len(results)
    print("-" * 55)
    print(f"{'Promedio':<14} {'':>10} {'':>10} {avg_pct:>9.2f}%")
    print("\n%E inc = (E_dist - E_min) / E_min x 100%")
    return results


def stats_png_path(prefix: str, suffix: str, category: str) -> Path:
    """category: 'small' | 'large' | 'extras' → stats/<category>/"""
    subdir = STATS_DIR / category
    subdir.mkdir(parents=True, exist_ok=True)
    return subdir / f"{prefix}_{suffix}.png"


def _draw_nodes(ax, vertices: list[Vertex]) -> None:
    legend_depot = legend_station = legend_customer = True
    for v in vertices:
        if v.is_depot:
            ax.scatter(
                v.x,
                v.y,
                marker="s",
                s=120,
                c="#d62728",
                edgecolors="black",
                linewidths=0.6,
                zorder=5,
                label="Depósito" if legend_depot else None,
            )
            legend_depot = False
        elif v.is_station:
            ax.scatter(
                v.x,
                v.y,
                marker="^",
                s=90,
                c="#2ca02c",
                edgecolors="black",
                linewidths=0.5,
                zorder=4,
                label="Estaciones" if legend_station else None,
            )
            legend_station = False
        else:
            ax.scatter(
                v.x,
                v.y,
                marker="o",
                s=45,
                c="#1f77b4",
                edgecolors="white",
                linewidths=0.4,
                zorder=3,
                label="Clientes" if legend_customer else None,
            )
            legend_customer = False


def _draw_routes_on_ax(
    ax,
    vertices: list[Vertex],
    routes: list[list[int]],
    *,
    color: str,
    linestyle: str,
    linewidth: float,
    alpha: float,
    label: str,
) -> None:
    vmap = {v.idx: v for v in vertices}
    drawn_label = False
    for route in routes:
        if len(route) < 2:
            continue
        xs = [vmap[n].x for n in route if n in vmap]
        ys = [vmap[n].y for n in route if n in vmap]
        if len(xs) < 2:
            continue
        ax.plot(
            xs,
            ys,
            color=color,
            linestyle=linestyle,
            linewidth=linewidth,
            alpha=alpha,
            label=label if not drawn_label else None,
            zorder=2,
        )
        drawn_label = True


def _save_route_comparison_map(
    prefix: str,
    instance: str,
    vertices: list[Vertex],
    eh_routes: list[list[int]],
    ref_routes: list[list[int]] | None,
    *,
    category: str = "small",
    ref_label: str = "Referencia",
    eh_label: str = "EH-SA/TS",
    eh_energy: float | None = None,
    ref_energy: float | None = None,
) -> Path | None:
    if not MATPLOTLIB_AVAILABLE:
        return None
    fig, ax = plt.subplots(figsize=(10, 8))
    _draw_nodes(ax, vertices)
    if ref_routes:
        _draw_routes_on_ax(
            ax,
            vertices,
            ref_routes,
            color="#9467bd",
            linestyle="-",
            linewidth=2.0,
            alpha=0.85,
            label=ref_label,
        )
    _draw_routes_on_ax(
        ax,
        vertices,
        eh_routes,
        color="#ff7f0e",
        linestyle="--",
        linewidth=2.0,
        alpha=0.9,
        label=eh_label,
    )
    title = f"Rutas {instance}"
    if eh_energy is not None and ref_energy is not None:
        title += (
            f"\n{ref_label}: {ref_energy:.1f} kWh | {eh_label}: {eh_energy:.1f} kWh"
        )
    elif eh_energy is not None:
        title += f"\n{eh_label}: {eh_energy:.1f} kWh"
    ax.set_title(title)
    ax.set_xlabel("X (millas)")
    ax.set_ylabel("Y (millas)")
    ax.grid(True, alpha=0.25)
    ax.set_aspect("equal", adjustable="box")
    handles, labels = ax.get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    legend_order = [
        "Depósito",
        "Estaciones",
        "Clientes",
        ref_label,
        eh_label,
    ]
    ordered_handles = [by_label[key] for key in legend_order if key in by_label]
    ordered_labels = [key for key in legend_order if key in by_label]
    ax.legend(
        ordered_handles,
        ordered_labels,
        loc="lower left",
        fontsize=9,
        framealpha=0.92,
    )
    fig.tight_layout()
    out = stats_png_path(prefix, f"map_{instance}", category)
    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)
    return out


MAP_GROUP_SIZE = 5


def _subplot_grid(n: int) -> tuple[int, int]:
    """Disposición filasxcolumnas para agrupar mapas (hasta 6 paneles en 2x3)."""
    if n <= 1:
        return 1, 1
    if n <= 2:
        return 1, 2
    if n <= 4:
        return 2, 2
    return 2, 3


def _flat_axes(axes) -> list:
    import numpy as np

    arr = np.atleast_1d(axes)
    return arr.flatten().tolist()


def _draw_small_map_on_ax(
    ax,
    vertices: list[Vertex],
    eh_routes: list[list[int]],
    mip_routes: list[list[int]] | None,
    instance: str,
    *,
    eh_energy: float | None = None,
    ref_energy: float | None = None,
    show_legend: bool = False,
) -> None:
    _draw_nodes(ax, vertices)
    if mip_routes:
        _draw_routes_on_ax(
            ax,
            vertices,
            mip_routes,
            color="#9467bd",
            linestyle="-",
            linewidth=1.4,
            alpha=0.85,
            label="OR-Tools",
        )
    _draw_routes_on_ax(
        ax,
        vertices,
        eh_routes,
        color="#ff7f0e",
        linestyle="--",
        linewidth=1.4,
        alpha=0.9,
        label="EH-SA/TS",
    )
    title = instance
    if eh_energy is not None and ref_energy is not None:
        title += f"\nOT {ref_energy:.0f} | EH {eh_energy:.0f} kWh"
    elif eh_energy is not None:
        title += f"\nEH {eh_energy:.0f} kWh"
    ax.set_title(title, fontsize=9)
    ax.tick_params(labelsize=7)
    ax.grid(True, alpha=0.2)
    ax.set_aspect("equal", adjustable="box")
    if show_legend:
        ax.legend(loc="upper right", fontsize=6, framealpha=0.9)


def _draw_large_3runs_on_ax(
    ax,
    vertices: list[Vertex],
    best_routes: list[list[int]] | None,
    worst_routes: list[list[int]] | None,
    avg_routes: list[list[int]] | None,
    instance: str,
    *,
    best_energy: float | None = None,
    worst_energy: float | None = None,
    avg_energy: float | None = None,
    show_legend: bool = False,
) -> None:
    _draw_nodes(ax, vertices)
    if best_routes:
        _draw_routes_on_ax(
            ax,
            vertices,
            best_routes,
            color="#1f77b4",
            linestyle="-",
            linewidth=1.5,
            alpha=0.9,
            label="Mejor",
        )
    if avg_routes:
        _draw_routes_on_ax(
            ax,
            vertices,
            avg_routes,
            color="#ff7f0e",
            linestyle="-.",
            linewidth=1.3,
            alpha=0.85,
            label="Rep.",
        )
    if worst_routes:
        _draw_routes_on_ax(
            ax,
            vertices,
            worst_routes,
            color="#d62728",
            linestyle="--",
            linewidth=1.3,
            alpha=0.8,
            label="Peor",
        )
    energy_bits: list[str] = []
    if best_energy is not None:
        energy_bits.append(f"B* {best_energy:.0f}")
    if avg_energy is not None:
        energy_bits.append(f"μ {avg_energy:.0f}")
    if worst_energy is not None:
        energy_bits.append(f"W {worst_energy:.0f}")
    sub = " | ".join(energy_bits)
    ax.set_title(f"{instance}\n{sub}" if sub else instance, fontsize=8)
    ax.tick_params(labelsize=7)
    ax.grid(True, alpha=0.2)
    ax.set_aspect("equal", adjustable="box")
    if show_legend:
        ax.legend(loc="upper right", fontsize=6, framealpha=0.9)


def _group_range_label_from_names(names: list[str]) -> str:
    return names[0] if len(names) == 1 else f"{names[0]} – {names[-1]}"


def save_grouped_small_maps(
    prefix: str,
    rows: list[dict],
    *,
    category: str = "small",
    group_size: int = MAP_GROUP_SIZE,
) -> list[Path]:
    """Mapas pequeños: 5 instancias por PNG (EH vs OR-Tools en cada panel)."""
    if not MATPLOTLIB_AVAILABLE:
        return []
    eligible = [r for r in rows if r.get("eh_routes") or r.get("eh_paths")]
    if not eligible:
        return []

    def sort_key(r: dict) -> tuple[int, int, int]:
        m = re.match(r"C(\d+)R(\d+)(?:-(\d+))?", r["instance"])
        if m:
            return (int(m.group(1)), int(m.group(2)), int(m.group(3) or 0))
        return (0, 0, 0)

    ordered = sorted(eligible, key=sort_key)
    saved: list[Path] = []
    for g_idx in range(0, len(ordered), group_size):
        chunk = ordered[g_idx : g_idx + group_size]
        g_num = g_idx // group_size + 1
        n = len(chunk)
        nrows, ncols = _subplot_grid(n)
        fig, axes = plt.subplots(nrows, ncols, figsize=(4.8 * ncols, 4.2 * nrows))
        for i, (ax, row) in enumerate(zip(_flat_axes(axes), chunk)):
            inst = row["instance"]
            eh_routes = row.get("eh_routes") or row.get("eh_paths")
            vertices, _ = load_instance(inst)
            _draw_small_map_on_ax(
                ax,
                vertices,
                eh_routes,
                row.get("mip_routes"),
                inst,
                eh_energy=row.get("eh_energy"),
                ref_energy=row.get("mip_energy"),
                show_legend=(i == 0),
            )
        for ax in _flat_axes(axes)[n:]:
            ax.set_visible(False)
        names = [r["instance"] for r in chunk]
        fig.suptitle(
            f"Mapas EH-SA/TS vs OR-Tools — {_group_range_label_from_names(names)}",
            fontsize=12,
            fontweight="bold",
        )
        fig.tight_layout()
        out = stats_png_path(prefix, f"map_g{g_num}", category)
        fig.savefig(out, dpi=130, bbox_inches="tight")
        plt.close(fig)
        saved.append(out)
    return saved


def save_grouped_large_maps(
    prefix: str,
    rows: list[dict],
    *,
    category: str = "large",
    group_size: int = MAP_GROUP_SIZE,
) -> list[Path]:
    """Mapas grandes: 5 instancias por PNG (mejor / rep. / peor corrida superpuestas)."""
    if not MATPLOTLIB_AVAILABLE:
        return []
    eligible = [
        r
        for r in rows
        if any([r.get("best_routes"), r.get("worst_routes"), r.get("avg_routes")])
    ]
    if not eligible:
        return []

    def sort_key(r: dict) -> tuple[int, int, int]:
        m = re.match(r"C(\d+)R(\d+)(?:-(\d+))?", r["instance"])
        if m:
            return (int(m.group(1)), int(m.group(2)), int(m.group(3) or 0))
        return (0, 0, 0)

    ordered = sorted(eligible, key=sort_key)
    saved: list[Path] = []
    for g_idx in range(0, len(ordered), group_size):
        chunk = ordered[g_idx : g_idx + group_size]
        g_num = g_idx // group_size + 1
        n = len(chunk)
        nrows, ncols = _subplot_grid(n)
        fig, axes = plt.subplots(nrows, ncols, figsize=(4.8 * ncols, 4.2 * nrows))
        for i, (ax, row) in enumerate(zip(_flat_axes(axes), chunk)):
            inst = row["instance"]
            vertices, _ = load_instance(inst)
            _draw_large_3runs_on_ax(
                ax,
                vertices,
                row.get("best_routes"),
                row.get("worst_routes"),
                row.get("avg_routes"),
                inst,
                best_energy=row.get("best_energy"),
                worst_energy=row.get("worst_energy"),
                avg_energy=row.get("avg_energy"),
                show_legend=(i == 0),
            )
        for ax in _flat_axes(axes)[n:]:
            ax.set_visible(False)
        names = [r["instance"] for r in chunk]
        fig.suptitle(
            f"Mapas EH-SA/TS (B* / rep. / peor) — {_group_range_label_from_names(names)}",
            fontsize=12,
            fontweight="bold",
        )
        fig.tight_layout()
        out = stats_png_path(prefix, f"map_3runs_g{g_num}", category)
        fig.savefig(out, dpi=130, bbox_inches="tight")
        plt.close(fig)
        saved.append(out)
    return saved


def _save_bar_chart(
    prefix: str,
    suffix: str,
    labels: list[str],
    series: dict[str, list[float]],
    *,
    category: str,
    title: str,
    ylabel: str,
    rotate: int = 45,
) -> Path | None:
    if not MATPLOTLIB_AVAILABLE or not labels:
        return None
    import math as _math

    def _safe(v):
        """Convierte None/inf/-inf a nan para que matplotlib los omita limpiamente."""
        if v is None:
            return float("nan")
        try:
            f = float(v)
            return float("nan") if not _math.isfinite(f) else f
        except (TypeError, ValueError):
            return float("nan")

    fig, ax = plt.subplots(figsize=(max(8, len(labels) * 0.45), 6))
    x = range(len(labels))
    n_series = len(series)
    width = 0.8 / max(n_series, 1)
    for i, (name, values) in enumerate(series.items()):
        offset = (i - (n_series - 1) / 2) * width
        ax.bar(
            [xi + offset for xi in x],
            [_safe(v) for v in values],
            width=width,
            label=name,
            alpha=0.9,
        )
    ax.set_xticks(list(x))
    ax.set_xticklabels(labels, rotation=rotate, ha="right", fontsize=8)
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    ax.legend()
    ax.grid(True, axis="y", alpha=0.3)
    fig.tight_layout()
    out = stats_png_path(prefix, suffix, category)
    fig.savefig(out, dpi=150, bbox_inches="tight")
    plt.close(fig)
    return out


def _save_large_3run_map(
    prefix: str,
    instance: str,
    vertices_list: list,
    best_routes: list[list[int]] | None,
    worst_routes: list[list[int]] | None,
    avg_routes: list[list[int]] | None,
    best_energy: float | None,
    worst_energy: float | None,
    avg_energy: float | None,
) -> Path | None:
    """Mapa de -large con 3 subplots: mejor, promedio y peor corrida."""
    if not MATPLOTLIB_AVAILABLE:
        return None
    if best_routes is None and worst_routes is None and avg_routes is None:
        return None

    vmap = {v.idx: v for v in vertices_list}
    fig, axes = plt.subplots(1, 3, figsize=(21, 7))
    specs = [
        (
            best_routes,
            best_energy,
            "#1f77b4",
            f"Mejor\n{best_energy:.2f} kWh" if best_energy else "Mejor (INF)",
        ),
        (
            avg_routes,
            avg_energy,
            "#ff7f0e",
            f"Representativo\n{avg_energy:.2f} kWh" if avg_energy else "Rep. (INF)",
        ),
        (
            worst_routes,
            worst_energy,
            "#d62728",
            f"Peor\n{worst_energy:.2f} kWh" if worst_energy else "Peor (INF)",
        ),
    ]
    for ax, (routes, energy, color, subtitle) in zip(axes, specs):
        _draw_nodes(ax, vertices_list)
        if routes:
            for seg in routes:
                xs = [vmap[n].x for n in seg if n in vmap]
                ys = [vmap[n].y for n in seg if n in vmap]
                ax.plot(xs, ys, color=color, linewidth=1.5, alpha=0.85)
        ax.set_title(subtitle, fontsize=10)
        ax.set_xlabel("X (miles)")
        ax.set_ylabel("Y (miles)")
        ax.grid(True, alpha=0.2)
        ax.legend(loc="upper right", fontsize=7)
    fig.suptitle(f"EH-SA/TS — {instance}", fontsize=13, fontweight="bold")
    fig.tight_layout()
    out = stats_png_path(prefix, f"map_{instance}_3runs", "large")
    fig.savefig(out, dpi=130, bbox_inches="tight")
    plt.close(fig)
    return out


def generate_visual_reports(
    prefix: str,
    mode: str,
    results: dict,
    *,
    seed: int = 42,
    single_name: str | None = None,
) -> list[Path]:
    """Genera gráficos PNG en stats/ sincronizados con el prefijo del log (run_NNN_modo)."""
    if not MATPLOTLIB_AVAILABLE:
        print(
            "\n[stats] matplotlib no instalado; omitiendo gráficos "
            "(pip install matplotlib)."
        )
        return []

    saved: list[Path] = []
    STATS_DIR.mkdir(parents=True, exist_ok=True)

    def add(path: Path | None) -> None:
        if path is not None:
            saved.append(path)

    large_rows = results.get("large")
    if large_rows and mode in ("large", "all"):
        labels = [s["instance"] for s in large_rows]
        add(
            _save_bar_chart(
                prefix,
                "rpd_bars",
                labels,
                {"RPD medio %": [s["mean_rpd"] for s in large_rows]},
                category="large",
                title="Grandes: RPD medio por instancia (vs B*)",
                ylabel="RPD (%)",
                rotate=90,
            )
        )
        add(
            _save_bar_chart(
                prefix,
                "energy_bars",
                labels,
                {
                    "B* (kWh)": [s["best"] for s in large_rows],
                    "Media (kWh)": [s["mean"] for s in large_rows],
                },
                category="large",
                title="Grandes: mejor y energía media EH-SA/TS",
                ylabel="Energía (kWh)",
                rotate=90,
            )
        )
        add(save_grouped_large_maps(prefix, large_rows, category="large"))

    if saved:
        print(
            f"\n[stats] {len(saved)} gráfico(s) PNG (prefijo {prefix}) en {STATS_DIR}/"
            f"\n        Regenerar offline: python stats.py -large {prefix.split('_')[1]}"
        )
        for path in saved:
            print(f"  - {path.name}")
    return saved


class _TeeWriter:
    def __init__(self, *streams):
        self._streams = streams

    def write(self, data):
        for stream in self._streams:
            stream.write(data)
            stream.flush()

    def flush(self):
        for stream in self._streams:
            stream.flush()


def log_mode_slug(mode: str, single_name: str | None = None) -> str:
    if mode == "single" and single_name:
        safe = re.sub(r"[^\w\-]", "_", single_name)
        return f"single_{safe}"
    return mode


def next_run_log_path(slug: str) -> Path:
    LOGS_DIR.mkdir(parents=True, exist_ok=True)
    pattern = re.compile(rf"^run_(\d{{3}})_{re.escape(slug)}\.txt$")
    max_run = 0
    for path in LOGS_DIR.iterdir():
        match = pattern.match(path.name)
        if match:
            max_run = max(max_run, int(match.group(1)))
    return LOGS_DIR / f"run_{max_run + 1:03d}_{slug}.txt"


@contextmanager
def capture_run_log(log_path: Path):
    log_file = open(log_path, "w", encoding="utf-8")
    stdout_prev, stderr_prev = sys.stdout, sys.stderr
    sys.stdout = _TeeWriter(stdout_prev, log_file)
    sys.stderr = _TeeWriter(stderr_prev, log_file)
    try:
        yield
    finally:
        sys.stdout = stdout_prev
        sys.stderr = stderr_prev
        log_file.close()


def normalize_mode(arg: str) -> str:
    aliases = {
        "-all": "all",
        "-small": "small",
        "-large": "large",
        "-recharge-stations": "recharge-stations",
        "-battery-reserve": "battery-reserve",
        "-energy-vs-distance": "energy-vs-distance",
    }
    return aliases.get(arg, arg)


def print_execution_modes_summary() -> None:
    """Resumen de modos (también en docstring del módulo)."""
    print("Modos de ejecución:")
    print("  python main.py -small")
    print("    EH-SA/TS en C10R2-C24R2 (15 inst.). Log: logs/run_NNN_small.txt")
    print("    Referencia CPLEX: solve_cplex.py → logs/run_NNN_small_cplex.txt")
    print(
        "    Gráficos + comparativa CPLEX: python stats.py -small [NNN] → stats/small/"
    )
    print("")
    print("  python main.py -large")
    print(
        "    10 runs EH-SA/TS, semillas distintas. B*, media, desviación estándar, RPD vs B*."
    )
    print("    Log: logs/run_NNN_large.txt")
    print("")
    print("  python main.py -all")
    print(
        "    -small + -large + recharge-stations + battery-reserve + energy-vs-distance"
    )
    print("    Log: logs/run_NNN_all.txt")
    print("")
    print("  python main.py recharge-stations")
    print("  python main.py battery-reserve")
    print("  python main.py energy-vs-distance")
    print("")
    print("  python main.py single <nombre>   (ej. C14R2, C25R2-1)")


def print_usage():
    print("Uso: python main.py <modo> [--seed N] [--runs N] [--time-limit-run N]")
    print("")
    print_execution_modes_summary()
    print("")
    print("Opciones:")
    print("  --seed N              Semilla base EH-SA/TS (default: 42)")
    print("  --runs N              Corridas en -large (default: 10)")
    print(
        f"  --time-limit-run N    Límite en segundos por corrida EH en -large y extras"
    )
    print(
        f"                        (default: {LARGE_RUN_TIME_LIMIT_S:.0f}s; usar 0 para sin límite)"
    )
    print("")
    print("Otros modos: extended (C10-C11), bank (55 instancias)")


def _execute_mode(
    mode: str,
    seed: int,
    single_name: str | None,
    num_runs: int = LARGE_NUM_RUNS_DEFAULT,
    time_limit_run_s: float | None = LARGE_RUN_TIME_LIMIT_S,
) -> dict:
    print("=" * 70)
    print("EH-SA/TS - Enhanced Hybrid Simulated Annealing / Tabu Search")
    print("EVRP - Electric Vehicle Routing Problem")
    print("=" * 70)
    print(f"Modo: {mode} | Semilla base: {seed}")
    collected: dict = {"mode": mode, "seed": seed}

    if mode == "all":
        print("\nPipeline -all:")
        print("  1) -small          EH-SA/TS (C10R2-C24R2)")
        print("  2) -large          Multi-run EH-SA/TS + RPD vs B*")
        print("  3) recharge-stations")
        print("  4) battery-reserve")
        print("  5) energy-vs-distance")

    if mode == "single":
        if single_name is None:
            print("Error: especifique el nombre. Ej: python main.py single C25R2-1")
            return collected
        print(f"\nEjecutando instancia: {single_name}")
        row = _run_eh_on_instance(single_name, seed)
        collected["single"] = row
        if row["feasible"] and row["energy"] is not None:
            print(f"Energía EH-SA/TS: {row['energy']:.4f} kWh")
        else:
            print(
                "Energía EH-SA/TS: INFACTIBLE (ver bloque [MEJOR CANDIDATO INFACTIBLE] arriba)"
            )
        print(f"f_gen:            {row['fgen']:.4f}")
        print(f"Tiempo:           {row['time_s']:.2f}s")
        print(f"Rutas:            {row['routes']}")
        print(f"Visitas estación: {row['station_visits']}")
        print("\n Experimento completado :D")
        return collected

    if mode in ("all", "small"):
        collected["small"] = run_small_benchmark(seed=seed)
    if mode in ("all", "large"):
        collected["large"] = run_large_multi_run(
            base_seed=seed, num_runs=num_runs, time_limit_s_per_run=time_limit_run_s
        )
    if mode == "extended":
        collected["extended"] = run_extended_small(seed)
    if mode == "bank":
        collected["bank"] = run_own_bank(seed)
    if mode in ("all", "recharge-stations"):
        collected["recharge_stations"] = run_recharge_stations(
            seed, time_limit_s=time_limit_run_s
        )
    if mode in ("all", "battery-reserve"):
        collected["battery_reserve"] = run_battery_reserve(
            seed, time_limit_s=time_limit_run_s
        )
    if mode in ("all", "energy-vs-distance"):
        collected["energy_vs_distance"] = run_energy_vs_distance(
            seed, time_limit_s=time_limit_run_s
        )
    print("\n Experimento completado :D")
    return collected


def _json_serializable(obj):
    """Convierte recursivamente a tipos JSON-serializables."""
    if isinstance(obj, dict):
        return {k: _json_serializable(v) for k, v in obj.items()}
    if isinstance(obj, list):
        return [_json_serializable(v) for v in obj]
    if isinstance(obj, float):
        if math.isnan(obj) or math.isinf(obj):
            return None
        return obj
    if isinstance(obj, (int, str, bool)) or obj is None:
        return obj
    return str(obj)


def save_run_data(log_path: Path, data: dict) -> Path:
    """Guarda datos estructurados en JSON junto al log (para stats.py offline)."""
    json_path = log_path.with_suffix(".json")
    try:
        json_path.write_text(
            json.dumps(_json_serializable(data), ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        print(f"[data] JSON guardado en: {json_path.name}")
    except Exception as exc:
        print(f"[data] Error guardando JSON: {exc}")
    return json_path


def main():
    args = sys.argv[1:]
    mode = "all"
    seed = 42
    single_name = None
    num_runs = LARGE_NUM_RUNS_DEFAULT
    time_limit_run_s: float | None = LARGE_RUN_TIME_LIMIT_S

    i = 0
    while i < len(args):
        if args[i] in ("--help", "-h"):
            print_usage()
            return
        if args[i] == "--seed" and i + 1 < len(args):
            seed = int(args[i + 1])
            i += 2
        elif args[i] == "--runs" and i + 1 < len(args):
            num_runs = int(args[i + 1])
            i += 2
        elif args[i] == "--time-limit-run" and i + 1 < len(args):
            val = float(args[i + 1])
            time_limit_run_s = None if val <= 0 else val
            i += 2
        elif args[i] == "single" and i + 1 < len(args):
            mode = "single"
            single_name = args[i + 1]
            i += 2
        else:
            mode = normalize_mode(args[i])
            i += 1

    slug = log_mode_slug(mode, single_name)
    log_path = next_run_log_path(slug)
    prefix = log_path.stem
    with capture_run_log(log_path):
        run_results = _execute_mode(
            mode,
            seed,
            single_name,
            num_runs=num_runs,
            time_limit_run_s=time_limit_run_s,
        )
    print(f"\nLog guardado en: {log_path}")
    save_run_data(log_path, run_results)
    generate_visual_reports(
        prefix,
        mode,
        run_results,
        seed=seed,
        single_name=single_name,
    )


if __name__ == "__main__":
    main()
