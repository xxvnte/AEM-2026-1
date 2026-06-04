from __future__ import annotations

import ast
import re
import sys
import time
from contextlib import contextmanager
from pathlib import Path

from amplpy import AMPL

from main import (
    ALPHA_ARC,
    BATTERY_CAPACITY_KWH,
    EFF_D,
    EFF_M,
    JOULES_TO_KWH,
    VEHICLE_CAPACITY_TONS,
    load_instance,
)

PROJECT_DIR = Path(__file__).parent
LOGS_DIR = PROJECT_DIR / "logs"
MODEL_PATH = PROJECT_DIR / "model.mod"
INSTANCES_DIR = PROJECT_DIR / "instances"
SMALL_DIR = INSTANCES_DIR / "small"
SMALL_DAT_DIR = INSTANCES_DIR / "small_dat"
LARGE_DIR = INSTANCES_DIR / "large"
LARGE_DAT_DIR = INSTANCES_DIR / "large_dat"
LOG_SLUG = "small_cplex"
MAX_STATION_COPIES = 3

DEFAULT_INSTANCES = [f"C{n}R2" for n in range(10, 25)]
CPLEX_MIP_GAP = 0.0001


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


def default_dat_path(name: str) -> Path:
    if (SMALL_DIR / f"{name}.txt").exists():
        return SMALL_DAT_DIR / f"{name}.dat"
    if (LARGE_DIR / f"{name}.txt").exists():
        return LARGE_DAT_DIR / f"{name}.dat"
    return INSTANCES_DIR / f"{name}.dat"


def resolve_dat_path(name: str) -> Path:
    for path in (
        SMALL_DAT_DIR / f"{name}.dat",
        LARGE_DAT_DIR / f"{name}.dat",
        INSTANCES_DIR / f"{name}.dat",
    ):
        if path.exists():
            return path
    raise FileNotFoundError(
        f"No se encontró '{name}.dat' en small_dat/large_dat. "
        f"Ejecute: python solve_cplex.py --build-small-dat"
    )


def list_small_instance_names() -> list[str]:
    return sorted(p.stem for p in SMALL_DIR.glob("*.txt"))


def build_expanded_graph(
    vertices: list,
    arcs: dict,
    max_station_copies: int = MAX_STATION_COPIES,
) -> tuple[
    list[int],
    set[int],
    list[int],
    list[int],
    int,
    int,
    dict[int, float],
    dict[tuple[int, int], float],
    int,
]:
    vmap = {v.idx: v for v in vertices}
    depot_idxs = [v.idx for v in vertices if v.is_depot]
    station_idxs = [v.idx for v in vertices if v.is_station]
    customer_idxs = [v.idx for v in vertices if v.is_customer]

    depot_start_vid = depot_idxs[0]
    depot_end_vid = depot_idxs[-1] if len(depot_idxs) > 1 else depot_idxs[0]
    n_customers = len(customer_idxs)
    max_vehicles = n_customers

    station_copies: list[int] = []
    for station_vid in station_idxs:
        for _ in range(max_station_copies * max_vehicles):
            station_copies.append(station_vid)

    all_nodes = [depot_start_vid] + customer_idxs + station_copies + [depot_end_vid]
    n_nodes = len(all_nodes)
    depot_start_idx = 0
    depot_end_idx = n_nodes - 1

    demands_kg = {c: vmap[c].demand_tons * 1000.0 for c in customer_idxs}
    customer_set = set(range(1, 1 + n_customers))
    station_local: set[int] = set()
    recharge_local: set[int] = set()
    for idx in range(n_nodes):
        vid = all_nodes[idx]
        if idx == depot_end_idx:
            recharge_local.add(idx)
        elif vid in vmap and vmap[vid].is_station:
            station_local.add(idx)
            recharge_local.add(idx)

    arc_energy: dict[tuple[int, int], float] = {}
    for i_idx in range(n_nodes):
        for j_idx in range(n_nodes):
            if i_idx == j_idx or i_idx == depot_end_idx or j_idx == depot_start_idx:
                continue
            i_vid = all_nodes[i_idx]
            j_vid = all_nodes[j_idx]
            load_approx = demands_kg.get(i_vid, 0.0) * 0.5
            arc = arcs.get((i_vid, j_vid))
            if arc is None:
                continue
            e_load = (
                ALPHA_ARC * load_approx * arc.dist_m * EFF_D * EFF_M * JOULES_TO_KWH
            )
            arc_energy[(i_idx, j_idx)] = arc.energy_kwh_empty + e_load

    return (
        all_nodes,
        customer_set,
        list(station_local),
        list(recharge_local),
        depot_start_idx,
        depot_end_idx,
        demands_kg,
        arc_energy,
        max_vehicles,
    )


def write_dat_file(name: str, output_path: Path | None = None) -> Path:
    vertices, arcs = load_instance(name)
    (
        all_nodes,
        customer_set,
        station_idxs,
        recharge_idxs,
        depot_start_idx,
        depot_end_idx,
        demands_kg,
        arc_energy,
        max_vehicles,
    ) = build_expanded_graph(vertices, arcs)

    n_nodes = len(all_nodes)
    cap_kg = VEHICLE_CAPACITY_TONS * 1000.0
    bat_cap = BATTERY_CAPACITY_KWH
    big_m_load = cap_kg + 1.0
    big_m_batt = bat_cap + 1.0

    if output_path is None:
        output_path = default_dat_path(name)

    node_list = " ".join(str(i) for i in range(n_nodes))
    lines: list[str] = [
        f"set NODES := {node_list};",
        f"set CUSTOMERS := {' '.join(str(i) for i in sorted(customer_set))};",
        f"set STATIONS := {' '.join(str(i) for i in sorted(station_idxs))};",
        f"set RECHARGE := {' '.join(str(i) for i in sorted(recharge_idxs))};",
        f"param depot_start := {depot_start_idx};",
        f"param depot_end := {depot_end_idx};",
        f"param cap_kg := {cap_kg};",
        f"param bat_cap := {bat_cap};",
        f"param big_m_load := {big_m_load};",
        f"param big_m_batt := {big_m_batt};",
        f"param max_vehicles := {max_vehicles};",
        "param demand :=",
    ]
    for idx in sorted(customer_set):
        vid = all_nodes[idx]
        lines.append(f"  {idx}  {demands_kg[vid]}")
    lines.append(";")
    lines.append("set ARCS :=")
    arc_lines = [f" ({i},{j})" for (i, j) in sorted(arc_energy)]
    for chunk_start in range(0, len(arc_lines), 12):
        lines.append("".join(arc_lines[chunk_start : chunk_start + 12]))
    lines.append(";")
    lines.append("param energy :=")
    for (i, j), e_val in sorted(arc_energy.items()):
        lines.append(f"  {i} {j}  {e_val:.8f}")
    lines.append(";")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8", newline="\n")
    return output_path


def build_all_small_dat() -> list[Path]:
    SMALL_DAT_DIR.mkdir(parents=True, exist_ok=True)
    return [
        write_dat_file(name, SMALL_DAT_DIR / f"{name}.dat")
        for name in list_small_instance_names()
    ]


def extract_mip_routes_ampl(
    ampl: AMPL,
    all_nodes: list[int],
    depot_start_idx: int,
    depot_end_idx: int,
) -> list[list[int]]:
    """Arcos activos x[i,j]=1; índices vía set ARCS (amplpy no expone Variable.keys)."""
    remaining: set[tuple[int, int]] = set()
    x_var = ampl.get_variable("x")
    arcs_set = ampl.get_set("ARCS")

    for arc in arcs_set:
        if isinstance(arc, tuple):
            i_idx, j_idx = int(arc[0]), int(arc[1])
        else:
            parts = re.findall(r"-?\d+", str(arc))
            if len(parts) < 2:
                continue
            i_idx, j_idx = int(parts[0]), int(parts[1])
        try:
            val = float(x_var[i_idx, j_idx].value())
        except (KeyError, TypeError, AttributeError):
            val = float(x_var[arc].value())
        if val > 0.5:
            remaining.add((i_idx, j_idx))

    routes: list[list[int]] = []
    while remaining:
        path_idx = [depot_start_idx]
        cur = depot_start_idx
        stuck = False
        while cur != depot_end_idx:
            next_candidates = [j for (i, j) in remaining if i == cur]
            if not next_candidates:
                stuck = True
                break
            nxt = next_candidates[0]
            remaining.discard((cur, nxt))
            path_idx.append(nxt)
            cur = nxt
        if len(path_idx) >= 2:
            routes.append([all_nodes[i] for i in path_idx])
        if stuck or path_idx[-1] != depot_end_idx:
            break
    return routes


def map_solver_status(
    solve_result: str, solve_message: str, solve_result_num: int
) -> str:
    text = f"{solve_result} {solve_message}".lower()
    if solve_result_num == 0:
        return "OPTIMO"
    if solve_result_num == 1:
        return "FACTIBLE (violaciones)"
    if solve_result_num == 2 or "infeasible" in text:
        return "INFACTIBLE"
    if solve_result_num == 3 or "unbounded" in text:
        return "NO ACOTADO"
    if solve_result_num in (4, 5, 6) or "limit" in text or "time" in text:
        return "FACTIBLE (lim. tiempo)"
    if solve_result == "solved":
        return "OPTIMO"
    if "feasible" in text:
        return "FACTIBLE (lim. tiempo)"
    if "failure" in text or "failed" in text:
        return "FALLO"
    return "DESCONOCIDO"


def create_ampl() -> AMPL:
    ampl = AMPL()
    ampl.set_option("solver", "cplex")
    ampl.set_option("cplex_options", f"mipgap={CPLEX_MIP_GAP}")
    ampl.read(str(MODEL_PATH))
    return ampl


def solve_instance(instance_name: str) -> dict:
    dat_path = resolve_dat_path(instance_name)
    vertices, arcs = load_instance(instance_name)
    (
        all_nodes,
        _customer_set,
        _station_idxs,
        _recharge_idxs,
        depot_start_idx,
        depot_end_idx,
        _demands_kg,
        _arc_energy,
        _max_vehicles,
    ) = build_expanded_graph(vertices, arcs)

    ampl = create_ampl()
    try:
        ampl.read_data(str(dat_path))
        t0 = time.perf_counter()
        ampl.solve()
        elapsed = time.perf_counter() - t0

        solve_result = str(ampl.get_value("solve_result") or "")
        solve_message = str(ampl.get_value("solve_message") or "")
        solve_result_num = int(ampl.get_value("solve_result_num") or -1)
        status = map_solver_status(solve_result, solve_message, solve_result_num)

        energy = float("nan")
        routes: list[list[int]] = []
        if status.startswith("OPTIMO") or status.startswith("FACTIBLE"):
            try:
                raw_obj = float(ampl.get_objective("total_energy").value())
                if raw_obj > -1e15:
                    energy = raw_obj
            except Exception:
                try:
                    raw_obj = float(ampl.get_value("total_energy"))
                    if raw_obj > -1e15:
                        energy = raw_obj
                except Exception:
                    energy = float("nan")
            if energy == energy:
                routes = extract_mip_routes_ampl(
                    ampl, all_nodes, depot_start_idx, depot_end_idx
                )

        return {
            "instance": instance_name,
            "energy_kwh": energy,
            "solve_time_s": round(elapsed, 2),
            "status": status,
            "solve_result": solve_result,
            "dat_file": str(dat_path),
            "routes": routes,
        }
    finally:
        ampl.close()


def parse_cplex_routes_from_log(text: str, instance: str) -> list[list[int]] | None:
    """Lee 'Rutas CPLEX C12R2: [[...], ...]' del log de solve_cplex."""
    marker = f"Rutas CPLEX {instance}:"
    for line in text.splitlines():
        if line.strip().startswith(marker):
            payload = line.split(":", 1)[1].strip()
            try:
                routes = ast.literal_eval(payload)
                if isinstance(routes, list):
                    return routes
            except (SyntaxError, ValueError):
                return None
    return None


def print_results_table(rows: list[dict]) -> None:
    print("\n" + "=" * 72)
    print("RESUMEN CPLEX (instancias pequeñas)")
    print("=" * 72)
    print(f"{'Instancia':<12} {'Energía':>12} {'Tiempo':>8} {'Estado':<22}")
    print("-" * 72)
    for row in rows:
        energy = row["energy_kwh"]
        energy_txt = f"{energy:>12.4f}" if energy == energy else f"{'nan':>12}"
        time_txt = (
            f"{row['solve_time_s']:>7.1f}s"
            if row["solve_time_s"] == row["solve_time_s"]
            else "    nan"
        )
        print(
            f"{row['instance']:<12} {energy_txt} {time_txt:>8} " f"{row['status']:<22}"
        )
    print("-" * 72)
    ok = sum(1 for r in rows if str(r["status"]).startswith(("OPTIMO", "FACTIBLE")))
    print(f"Resueltas: {ok}/{len(rows)}")


def run_batch(instance_names: list[str]) -> list[dict]:
    if not MODEL_PATH.exists():
        raise FileNotFoundError(f"No se encontró el modelo: {MODEL_PATH}")

    rows: list[dict] = []
    for name in instance_names:
        print(f"\n{'=' * 60}")
        print(f"Instancia: {name}")
        try:
            dat_path = resolve_dat_path(name)
            print(f"Datos: {dat_path}")
            row = solve_instance(name)
            energy_txt = (
                f"{row['energy_kwh']:.4f}"
                if row["energy_kwh"] == row["energy_kwh"]
                else "nan"
            )
            print(
                f"Estado: {row['status']} | Energía: {energy_txt} kWh "
                f"| Tiempo: {row['solve_time_s']} s"
            )
            if row.get("routes"):
                print(f"Rutas CPLEX {name}: {row['routes']}")
        except Exception as exc:
            row = {
                "instance": name,
                "energy_kwh": float("nan"),
                "solve_time_s": float("nan"),
                "status": "ERROR",
                "solve_result": str(exc),
                "dat_file": "",
                "routes": [],
            }
            print(f"Error en {name}: {exc}")
        rows.append(row)

    print_results_table(rows)
    return rows


def main() -> None:
    args = sys.argv[1:]
    if not args:
        instances = DEFAULT_INSTANCES
    elif args[0] == "--build-small-dat":
        paths = build_all_small_dat()
        for path in paths:
            print(f"Generado: {path}")
        return
    elif args[0] == "--help" or args[0] == "-h":
        print("Uso: python solve_cplex.py [instancia ...]")
        print("     python solve_cplex.py --build-small-dat")
        print(f"Logs: logs/run_NNN_{LOG_SLUG}.txt")
        return
    else:
        instances = args

    log_path = next_run_log_path(LOG_SLUG)
    print(f"Log: {log_path}")

    with capture_run_log(log_path):
        print("EVRP — batch CPLEX (AMPL)")
        print(f"Instancias: {len(instances)}")
        print(f"CPLEX: sin límite de tiempo, mipgap={CPLEX_MIP_GAP}")
        run_batch(instances)
        print("\n Experimento CPLEX completado")

    print(f"\nLog guardado en: {log_path}")


if __name__ == "__main__":
    main()
