from __future__ import annotations

import argparse
import re
from dataclasses import dataclass
from pathlib import Path

PROJECT_DIR = Path(__file__).parent
LOGS_DIR = PROJECT_DIR / "logs"
STATS_DIR = PROJECT_DIR / "stats" / "small"

EH_SLUG = "small"
CPLEX_SLUG = "small_cplex"

from main import (
    MATPLOTLIB_AVAILABLE,
    _run_eh_on_instance,
    _save_bar_chart,
    _save_route_comparison_map,
    absolute_gap,
    load_instance,
)
from solve_cplex import parse_cplex_routes_from_log, solve_instance


@dataclass
class InstanceRow:
    instance: str
    eh_energy: float | None
    cplex_energy: float | None
    eh_time_s: float | None = None
    cplex_time_s: float | None = None
    cplex_status: str | None = None
    gap_pct: float | None = None
    eh_routes: list[list[int]] | None = None
    cplex_routes: list[list[int]] | None = None


def _run_number_from_arg(arg: str) -> int:
    digits = re.sub(r"\D", "", arg)
    if not digits:
        raise ValueError(f"Número de run inválido: {arg}")
    return int(digits)


def find_latest_run_pair() -> int:
    pattern = re.compile(rf"^run_(\d{{3}})_{re.escape(EH_SLUG)}\.txt$")
    runs: list[int] = []
    for path in LOGS_DIR.glob(f"run_*_{EH_SLUG}.txt"):
        match = pattern.match(path.name)
        if not match:
            continue
        run_n = int(match.group(1))
        cplex_log = LOGS_DIR / f"run_{run_n:03d}_{CPLEX_SLUG}.txt"
        if cplex_log.exists():
            runs.append(run_n)
    if not runs:
        raise FileNotFoundError(
            f"No hay par de logs run_NNN_{EH_SLUG}.txt + run_NNN_{CPLEX_SLUG}.txt en {LOGS_DIR}"
        )
    return max(runs)


def log_paths_for_run(run_n: int) -> tuple[Path, Path]:
    eh_log = LOGS_DIR / f"run_{run_n:03d}_{EH_SLUG}.txt"
    cplex_log = LOGS_DIR / f"run_{run_n:03d}_{CPLEX_SLUG}.txt"
    if not eh_log.exists():
        raise FileNotFoundError(f"No existe: {eh_log}")
    if not cplex_log.exists():
        raise FileNotFoundError(f"No existe: {cplex_log}")
    return eh_log, cplex_log


def parse_eh_seed(text: str, default: int = 42) -> int:
    for pattern in (
        r"Semilla\s+EH:\s*(\d+)",
        r"Semilla\s+base:\s*(\d+)",
    ):
        match = re.search(pattern, text)
        if match:
            return int(match.group(1))
    return default


def parse_eh_table(text: str) -> dict[str, dict[str, float | str]]:
    """Energía (kWh) y tiempo (s) por instancia desde la tabla del log EH."""
    rows: dict[str, dict[str, float | str]] = {}
    summary_re = re.compile(r"^\s+(C\d+R\d+):\s+EH-SA/TS=([\d.]+)\s+kWh")
    table_re = re.compile(
        r"^(C\d+R\d+)\s+\d+\s+\d+\s+([\d.]+|INFACTIBLE)\s+\d+\s+\d+\s+([\d.]+)s"
    )
    for line in text.splitlines():
        match = summary_re.match(line)
        if match:
            inst = match.group(1)
            rows.setdefault(inst, {})["energy"] = float(match.group(2))
            continue
        match = table_re.match(line.strip())
        if not match:
            continue
        inst = match.group(1)
        entry = rows.setdefault(inst, {})
        if match.group(2) != "INFACTIBLE":
            entry["energy"] = float(match.group(2))
        else:
            entry["energy"] = "INFACTIBLE"
        entry["time_s"] = float(match.group(3))
    return rows


def parse_eh_energies(text: str) -> dict[str, float]:
    energies: dict[str, float] = {}
    for inst, data in parse_eh_table(text).items():
        energy = data.get("energy")
        if isinstance(energy, float):
            energies[inst] = energy
    return energies


def parse_eh_times(text: str) -> dict[str, float]:
    times: dict[str, float] = {}
    for inst, data in parse_eh_table(text).items():
        time_s = data.get("time_s")
        if isinstance(time_s, float):
            times[inst] = time_s
    return times


def parse_cplex_energies(text: str) -> dict[str, float]:
    energies: dict[str, float] = {}
    block_re = re.compile(r"^Instancia:\s+(C\d+R\d+)", re.MULTILINE)
    energy_re = re.compile(
        r"Estado:.*\|\s+Energía:\s+([\d.]+)\s+kWh", re.MULTILINE
    )
    table_re = re.compile(r"^(C\d+R\d+)\s+([\d.]+)\s+[\d.]+s\s+\w+")
    for line in text.splitlines():
        match = table_re.match(line.strip())
        if match:
            energies[match.group(1)] = float(match.group(2))

    if energies:
        return energies

    blocks = list(block_re.finditer(text))
    energy_hits = list(energy_re.finditer(text))
    for idx, block in enumerate(blocks):
        inst = block.group(1)
        if idx < len(energy_hits):
            energies[inst] = float(energy_hits[idx].group(1))
    return energies


def parse_cplex_times_and_status(text: str) -> tuple[dict[str, float], dict[str, str]]:
    times: dict[str, float] = {}
    status: dict[str, str] = {}
    table_re = re.compile(r"^(C\d+R\d+)\s+[\d.]+\s+([\d.]+)s\s+(\S+)")
    for line in text.splitlines():
        match = table_re.match(line.strip())
        if match:
            times[match.group(1)] = float(match.group(2))
            status[match.group(1)] = match.group(3)

    if times:
        return times, status

    blocks = re.split(r"={10,}", text)
    for block in blocks:
        inst_m = re.search(r"Instancia:\s+(C\d+R\d+)", block)
        time_m = re.search(r"Tiempo:\s+([\d.]+)\s*s", block)
        state_m = re.search(r"Estado:\s+([^|]+)", block)
        if not inst_m:
            continue
        inst = inst_m.group(1)
        if time_m:
            times[inst] = float(time_m.group(1))
        if state_m:
            status[inst] = state_m.group(1).strip()
    return times, status


def build_rows(
    eh_text: str,
    cplex_text: str,
    *,
    seed: int,
    load_routes: bool,
) -> list[InstanceRow]:
    eh_e = parse_eh_energies(eh_text)
    eh_t = parse_eh_times(eh_text)
    cplex_e = parse_cplex_energies(cplex_text)
    cplex_t, cplex_st = parse_cplex_times_and_status(cplex_text)
    instances = sorted(set(eh_e) | set(cplex_e) | set(eh_t) | set(cplex_t))
    rows: list[InstanceRow] = []

    for inst in instances:
        eh_val = eh_e.get(inst)
        ref_val = cplex_e.get(inst)
        gap = None
        if eh_val is not None and ref_val is not None and ref_val > 0:
            gap = absolute_gap(eh_val, ref_val)

        eh_routes = None
        cplex_routes = None
        if load_routes:
            cplex_routes = parse_cplex_routes_from_log(cplex_text, inst)
            if cplex_routes is None and ref_val is not None:
                try:
                    solved = solve_instance(inst)
                    cplex_routes = solved.get("routes") or None
                except Exception as exc:
                    print(f"[stats] CPLEX rutas {inst}: {exc}")
            if eh_val is not None:
                try:
                    eh_row = _run_eh_on_instance(inst, seed)
                    if eh_row.get("feasible"):
                        eh_routes = eh_row.get("eh_paths")
                except Exception as exc:
                    print(f"[stats] EH rutas {inst}: {exc}")

        rows.append(
            InstanceRow(
                instance=inst,
                eh_energy=eh_val,
                cplex_energy=ref_val,
                eh_time_s=eh_t.get(inst),
                cplex_time_s=cplex_t.get(inst),
                cplex_status=cplex_st.get(inst),
                gap_pct=gap,
                eh_routes=eh_routes,
                cplex_routes=cplex_routes,
            )
        )
    return rows


def _fmt_num(value: float | None, width: int, decimals: int = 2) -> str:
    if value is None:
        return f"{'---':>{width}}"
    return f"{value:>{width}.{decimals}f}"


def _fmt_time(value: float | None, width: int) -> str:
    if value is None:
        return f"{'---':>{width}}"
    return f"{value:>{width}.1f}s"


def _fmt_gap(value: float | None, width: int) -> str:
    if value is None:
        return f"{'---':>{width}}"
    return f"{value:>{width}.2f}%"


def write_comparative_txt(
    run_n: int,
    eh_log: Path,
    cplex_log: Path,
    rows: list[InstanceRow],
) -> Path:
    STATS_DIR.mkdir(parents=True, exist_ok=True)
    out_path = STATS_DIR / f"comparative_{run_n:03d}_{EH_SLUG}.txt"

    comparable = [
        r
        for r in rows
        if r.gap_pct is not None
        and r.eh_energy is not None
        and r.cplex_energy is not None
    ]

    lines: list[str] = [
        "=" * 88,
        f"COMPARATIVA EH-SA/TS vs CPLEX — run {run_n:03d}",
        "=" * 88,
        f"Log EH:    logs/{eh_log.name}",
        f"Log CPLEX: logs/{cplex_log.name}",
        "Gap % = (E_EH - E_CPLEX) / E_CPLEX x 100%",
        "",
        f"{'Instancia':<10} {'E_CPLEX':>10} {'t_CPLEX':>8} {'E_EH':>10} "
        f"{'t_EH':>8} {'Gap %':>9} {'Estado CPLEX':<18}",
        "-" * 88,
    ]

    for row in rows:
        lines.append(
            f"{row.instance:<10} "
            f"{_fmt_num(row.cplex_energy, 10)} "
            f"{_fmt_time(row.cplex_time_s, 8)} "
            f"{_fmt_num(row.eh_energy, 10)} "
            f"{_fmt_time(row.eh_time_s, 8)} "
            f"{_fmt_gap(row.gap_pct, 9)} "
            f"{(row.cplex_status or '---'):<18}"
        )

    lines.append("-" * 88)
    if comparable:
        n = len(comparable)
        avg_cplex_e = sum(r.cplex_energy for r in comparable) / n
        avg_eh_e = sum(r.eh_energy for r in comparable) / n
        avg_gap = sum(r.gap_pct for r in comparable) / n
        cplex_times = [r.cplex_time_s for r in comparable if r.cplex_time_s is not None]
        eh_times = [r.eh_time_s for r in comparable if r.eh_time_s is not None]
        avg_cplex_t = sum(cplex_times) / len(cplex_times) if cplex_times else None
        avg_eh_t = sum(eh_times) / len(eh_times) if eh_times else None
        lines.append(
            f"{'Promedio':<10} "
            f"{_fmt_num(avg_cplex_e, 10)} "
            f"{_fmt_time(avg_cplex_t, 8)} "
            f"{_fmt_num(avg_eh_e, 10)} "
            f"{_fmt_time(avg_eh_t, 8)} "
            f"{_fmt_gap(avg_gap, 9)} "
            f"{f'({n} inst.)':<18}"
        )
    lines.extend(
        [
            "",
            f"Instancias con Gap: {len(comparable)}/{len(rows)}",
            "=" * 88,
        ]
    )

    out_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return out_path


def generate_charts(
    prefix: str,
    rows: list[InstanceRow],
    *,
    with_maps: bool = True,
) -> list[Path]:
    if not MATPLOTLIB_AVAILABLE:
        print("[stats_small] matplotlib no instalado (pip install matplotlib).")
        return []

    gap_rows = [
        r
        for r in rows
        if r.gap_pct is not None
        and r.eh_energy is not None
        and r.cplex_energy is not None
    ]
    saved: list[Path] = []

    def add(path: Path | None) -> None:
        if path is not None:
            saved.append(path)

    if gap_rows:
        labels = [r.instance for r in gap_rows]
        add(
            _save_bar_chart(
                prefix,
                "bars",
                labels,
                {
                    "CPLEX (kWh)": [r.cplex_energy for r in gap_rows],
                    "EH-SA/TS (kWh)": [r.eh_energy for r in gap_rows],
                },
                category="small",
                title="Pequeñas: energía CPLEX vs EH-SA/TS",
                ylabel="Energía (kWh)",
            )
        )
        add(
            _save_bar_chart(
                prefix,
                "gaps",
                labels,
                {"Gap % (EH)": [r.gap_pct for r in gap_rows]},
                category="small",
                title="Pequeñas: Absolute Gap EH-SA/TS vs CPLEX",
                ylabel="Gap (%)",
            )
        )

    if not with_maps:
        return saved

    for row in rows:
        if not row.eh_routes:
            continue
        vertices, _ = load_instance(row.instance)
        add(
            _save_route_comparison_map(
                prefix,
                row.instance,
                vertices,
                row.eh_routes,
                row.cplex_routes,
                category="small",
                ref_label="CPLEX",
                eh_energy=row.eh_energy,
                ref_energy=row.cplex_energy,
            )
        )
    return saved


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Gráficos pequeñas (EH vs CPLEX) desde logs emparejados."
    )
    parser.add_argument(
        "run",
        nargs="?",
        help="Número de run (ej. 001). Por defecto: último par de logs.",
    )
    parser.add_argument(
        "--no-maps",
        action="store_true",
        help="Solo barras y gaps; no genera mapas por instancia.",
    )
    args = parser.parse_args()

    run_n = _run_number_from_arg(args.run) if args.run else find_latest_run_pair()
    eh_log, cplex_log = log_paths_for_run(run_n)
    prefix = f"run_{run_n:03d}_{EH_SLUG}"

    eh_text = eh_log.read_text(encoding="utf-8")
    cplex_text = cplex_log.read_text(encoding="utf-8")
    seed = parse_eh_seed(eh_text)

    print(f"EH log:    {eh_log.name}")
    print(f"CPLEX log: {cplex_log.name}")
    print(f"Prefijo:   {prefix} | Semilla EH (mapas): {seed}")

    rows = build_rows(
        eh_text,
        cplex_text,
        seed=seed,
        load_routes=not args.no_maps,
    )
    compared = [r for r in rows if r.gap_pct is not None]
    print(f"Instancias comparables: {len(compared)}/{len(rows)}")

    comparative_path = write_comparative_txt(run_n, eh_log, cplex_log, rows)
    print(f"Comparativa: {comparative_path}")

    saved = generate_charts(prefix, rows, with_maps=not args.no_maps)
    if saved:
        print(f"\n[stats_small] {len(saved)} gráfico(s) en {STATS_DIR}/")
        for path in saved:
            print(f"  - {path.name}")
    else:
        print("\n[stats_small] No se generaron gráficos.")


if __name__ == "__main__":
    main()
