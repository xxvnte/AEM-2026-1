from __future__ import annotations

import ast
import re
import sys
import time
from contextlib import contextmanager
from pathlib import Path

from amplpy import AMPL

from main import parse_instance_name

PROJECT_DIR = Path(__file__).parent
SOLVER_DIR = PROJECT_DIR / "solver"
LOGS_DIR = PROJECT_DIR / "logs"
MODEL_PATH = SOLVER_DIR / "evrp.mod"
DAT_DIR = SOLVER_DIR / "dat"
SMALL_DIR = PROJECT_DIR / "instances" / "small"
LOG_SLUG = "small_cplex"

DEFAULT_INSTANCES = [f"C{n}R2" for n in range(10, 25)]
CPLEX_TIME_LIMIT_S = 300
CPLEX_MIP_GAP = 0.01
CPLEX_INSTANCE_TIME_LIMITS: dict[str, int] = {"C23R2": 900}
CPLEX_INSTANCE_MIP_GAP: dict[str, float] = {"C23R2": 0.05}


def cplex_time_limit_s(instance_name: str) -> int:
    return CPLEX_INSTANCE_TIME_LIMITS.get(instance_name, CPLEX_TIME_LIMIT_S)


def cplex_mip_gap(instance_name: str) -> float:
    return CPLEX_INSTANCE_MIP_GAP.get(instance_name, CPLEX_MIP_GAP)


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


def list_small_instance_names() -> list[str]:
    return sorted(p.stem for p in SMALL_DIR.glob("*.txt"))


def resolve_dat_path(name: str) -> Path:
    path = DAT_DIR / f"{name}.dat"
    if path.exists():
        return path
    raise FileNotFoundError(
        f"No se encontro '{name}.dat' en solver/dat/. "
        f"Ejecute: python solve_cplex.py --build-dat"
    )


def build_all_small_dat() -> list[Path]:
    """Regenera solver/dat/*.dat desde instances/small/ via export_to_dat.py."""
    sys.path.insert(0, str(SOLVER_DIR))
    from export_to_dat import process_file

    DAT_DIR.mkdir(parents=True, exist_ok=True)
    paths: list[Path] = []
    for name in list_small_instance_names():
        txt = SMALL_DIR / f"{name}.txt"
        paths.append(Path(process_file(str(txt), str(DAT_DIR))))
    return paths


def orig_id_to_instance(orig: int, n_customers: int) -> int:
    """Convierte orig_id del .dat (esquema export_to_dat) a ID de instancia .txt."""
    if orig <= 1:
        return 0
    if orig <= n_customers + 1:
        return orig - 1
    return n_customers + 1 + (orig - n_customers - 2)


def _ampl_param_scalar(ampl: AMPL, name: str) -> int:
    p = ampl.get_parameter(name)
    val = p.value() if hasattr(p, "value") else p
    if hasattr(val, "value"):
        val = val.value()
    return int(val)


def _ampl_indexed_param(ampl: AMPL, name: str, idx: int) -> int:
    p = ampl.get_parameter(name)
    try:
        val = p[idx]
    except (KeyError, TypeError):
        val = p[idx].value() if hasattr(p[idx], "value") else p[idx]
    if hasattr(val, "value"):
        val = val.value()
    return int(val)


def extract_mip_routes_evrp(ampl: AMPL, instance_name: str) -> list[list[int]]:
    """Extrae rutas por vehiculo desde evrp.mod (x[i,j,k] + orig_id)."""
    n_customers, _n_stations = parse_instance_name(instance_name)
    depot_start = _ampl_param_scalar(ampl, "depot_start")
    depot_end = _ampl_param_scalar(ampl, "depot_end")
    x_var = ampl.get_variable("x")

    def orig(ampl_idx: int) -> int:
        return _ampl_indexed_param(ampl, "orig_id", ampl_idx)

    def x_val(i: int, j: int, k) -> float:
        try:
            return float(x_var[i, j, k].value())
        except (KeyError, TypeError, AttributeError):
            try:
                return float(x_var[(i, j, k)].value())
            except (KeyError, TypeError, AttributeError):
                return 0.0

    vehicles = list(ampl.get_set("VEHICLES"))
    active: dict = {k: [] for k in vehicles}
    for arc in ampl.get_set("ARCS"):
        if isinstance(arc, tuple):
            i_idx, j_idx = int(arc[0]), int(arc[1])
        else:
            parts = re.findall(r"-?\d+", str(arc))
            if len(parts) < 2:
                continue
            i_idx, j_idx = int(parts[0]), int(parts[1])
        for k in vehicles:
            if x_val(i_idx, j_idx, k) > 0.5:
                active[k].append((i_idx, j_idx))

    routes: list[list[int]] = []
    for k in vehicles:
        if not any(i == depot_start for i, _ in active[k]):
            continue
        path_inst: list[int] = [orig_id_to_instance(orig(depot_start), n_customers)]
        cur = depot_start
        visited = 0
        max_steps = len(list(ampl.get_set("NODES"))) + 5
        while cur != depot_end and visited < max_steps:
            visited += 1
            next_candidates = [j for i, j in active[k] if i == cur]
            if not next_candidates:
                break
            nxt = next_candidates[0]
            active[k] = [(i, j) for i, j in active[k] if not (i == cur and j == nxt)]
            if nxt != depot_end:
                path_inst.append(orig_id_to_instance(orig(nxt), n_customers))
            cur = nxt
        if len(path_inst) >= 2:
            if path_inst[-1] != 0:
                path_inst.append(0)
            routes.append(path_inst)

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


def create_ampl(*, time_limit_s: int | None = None, mip_gap: float | None = None) -> AMPL:
    if not MODEL_PATH.exists():
        raise FileNotFoundError(f"No se encontro el modelo: {MODEL_PATH}")
    limit = time_limit_s if time_limit_s is not None else CPLEX_TIME_LIMIT_S
    gap = mip_gap if mip_gap is not None else CPLEX_MIP_GAP
    ampl = AMPL()
    ampl.set_option("solver", "cplex")
    ampl.set_option(
        "cplex_options",
        f"timelimit={limit} mipgap={gap} threads=0",
    )
    ampl.read(str(MODEL_PATH))
    return ampl


def solve_instance(instance_name: str) -> dict:
    dat_path = resolve_dat_path(instance_name)
    time_limit = cplex_time_limit_s(instance_name)
    mip_gap = cplex_mip_gap(instance_name)
    ampl = create_ampl(time_limit_s=time_limit, mip_gap=mip_gap)
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
                raw_obj = float(ampl.get_objective("TotalEnergy").value())
                if raw_obj > -1e15:
                    energy = raw_obj
            except Exception:
                try:
                    raw_obj = float(ampl.get_value("TotalEnergy"))
                    if raw_obj > -1e15:
                        energy = raw_obj
                except Exception:
                    energy = float("nan")
            if energy == energy:
                routes = extract_mip_routes_evrp(ampl, instance_name)

        return {
            "instance": instance_name,
            "energy_kwh": energy,
            "solve_time_s": round(elapsed, 2),
            "time_limit_s": time_limit,
            "mip_gap": mip_gap,
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
    print(f"{'Instancia':<12} {'Energia':>12} {'Tiempo':>8} {'Estado':<22}")
    print("-" * 72)
    for row in rows:
        energy = row["energy_kwh"]
        energy_txt = f"{energy:>12.4f}" if energy == energy else f"{'nan':>12}"
        time_txt = (
            f"{row['solve_time_s']:>7.1f}s"
            if row["solve_time_s"] == row["solve_time_s"]
            else "    nan"
        )
        print(f"{row['instance']:<12} {energy_txt} {time_txt:>8} {row['status']:<22}")
    print("-" * 72)
    ok = sum(1 for r in rows if str(r["status"]).startswith(("OPTIMO", "FACTIBLE")))
    print(f"Resueltas: {ok}/{len(rows)}")


def run_batch(instance_names: list[str]) -> list[dict]:
    rows: list[dict] = []
    for name in instance_names:
        print(f"\n{'=' * 60}")
        print(f"Instancia: {name}")
        try:
            dat_path = resolve_dat_path(name)
            print(f"Modelo: {MODEL_PATH}")
            print(f"Datos: {dat_path}")
            time_limit = cplex_time_limit_s(name)
            mip_gap = cplex_mip_gap(name)
            print(f"CPLEX timelimit: {time_limit}s, mipgap: {mip_gap}")
            row = solve_instance(name)
            energy_txt = (
                f"{row['energy_kwh']:.4f}"
                if row["energy_kwh"] == row["energy_kwh"]
                else "nan"
            )
            print(
                f"Estado: {row['status']} | Energia: {energy_txt} kWh "
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
    elif args[0] in ("--build-dat", "--build-small-dat"):
        paths = build_all_small_dat()
        for path in paths:
            print(f"Generado: {path}")
        return
    elif args[0] in ("--help", "-h"):
        print("Uso: python solve_cplex.py [instancia ...]")
        print("     python solve_cplex.py --build-dat")
        print(f"Modelo: solver/evrp.mod")
        print(f"Datos:  solver/dat/<instancia>.dat")
        print(f"Logs:   logs/run_NNN_{LOG_SLUG}.txt")
        print(
            f"CPLEX:  timelimit={CPLEX_TIME_LIMIT_S}s, mipgap={CPLEX_MIP_GAP} "
            f"(C23R2: {CPLEX_INSTANCE_TIME_LIMITS['C23R2']}s, mipgap={CPLEX_INSTANCE_MIP_GAP['C23R2']})"
        )
        return
    else:
        instances = args

    log_path = next_run_log_path(LOG_SLUG)
    print(f"Log: {log_path}")

    with capture_run_log(log_path):
        print("EVRP — batch CPLEX (AMPL)")
        print(f"Modelo: {MODEL_PATH}")
        print(f"Instancias: {len(instances)}")
        print(
            f"CPLEX: timelimit={CPLEX_TIME_LIMIT_S}s, mipgap={CPLEX_MIP_GAP} por defecto; "
            f"C23R2: {CPLEX_INSTANCE_TIME_LIMITS['C23R2']}s, mipgap={CPLEX_INSTANCE_MIP_GAP['C23R2']}"
        )
        run_batch(instances)
        print("\n Experimento CPLEX completado")

    print(f"\nLog guardado en: {log_path}")


if __name__ == "__main__":
    main()
