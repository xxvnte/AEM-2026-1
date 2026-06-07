from __future__ import annotations

import argparse
import ast
import json
import math
import re
import statistics
import sys
from dataclasses import dataclass
from pathlib import Path

PROJECT_DIR = Path(__file__).parent
LOGS_DIR = PROJECT_DIR / "logs"
STATS_DIR = PROJECT_DIR / "stats"
STATS_SMALL_DIR = STATS_DIR / "small"
SMALL_SLUG = "small"
CPLEX_SMALL_SLUG = "small_cplex"

try:
    import matplotlib.pyplot as plt
    import matplotlib.gridspec as gridspec

    MATPLOTLIB_AVAILABLE = True
except ImportError:
    MATPLOTLIB_AVAILABLE = False

try:
    from main import (
        _save_bar_chart,
        save_grouped_large_maps,
        save_grouped_small_maps,
        stats_png_path,
        MATPLOTLIB_AVAILABLE as _MAIN_MPL,
    )

    MAIN_AVAILABLE = True
except Exception:
    MAIN_AVAILABLE = False


MODE_WITH_LEADING_DASH = {
    "-small": "small",
    "-large": "large",
    "-recharge-stations": "recharge-stations",
    "-battery-reserve": "battery-reserve",
    "-energy-vs-distance": "energy-vs-distance",
}

VALID_MODE_SLUGS = frozenset(MODE_WITH_LEADING_DASH.values())


def _normalize_mode_slug(mode: str) -> str:
    if mode in MODE_WITH_LEADING_DASH:
        return MODE_WITH_LEADING_DASH[mode]
    slug = mode.lstrip("-") if mode.startswith("-") else mode
    return slug if slug in VALID_MODE_SLUGS else mode


def _prepare_cli_argv(argv: list[str]) -> list[str]:
    """Convierte -small → small en el primer argumento posicional (modo)."""
    prepared = list(argv)
    for i, token in enumerate(prepared):
        if token.startswith("--") or token in ("-h",):
            continue
        if token in MODE_WITH_LEADING_DASH:
            prepared[i] = MODE_WITH_LEADING_DASH[token]
        break
    return prepared


def _parse_eh_log_table(text: str) -> dict[str, dict[str, float | str]]:
    """Energía y tiempo por instancia desde logs/run_NNN_small.txt."""
    rows: dict[str, dict[str, float | str]] = {}
    summary_re = re.compile(r"^\s+(C\d+R\d+):\s+EH-SA/TS=([\d.]+)\s+kWh")
    table_re = re.compile(
        r"^(C\d+R\d+)\s+\d+\s+\d+\s+([\d.]+|INFACTIBLE)\s+\d+\s+\d+\s+([\d.]+)s"
    )
    for line in text.splitlines():
        if match := summary_re.match(line):
            rows.setdefault(match.group(1), {})["energy"] = float(match.group(2))
            continue
        if match := table_re.match(line.strip()):
            inst = match.group(1)
            entry = rows.setdefault(inst, {})
            if match.group(2) != "INFACTIBLE":
                entry["energy"] = float(match.group(2))
            else:
                entry["energy"] = "INFACTIBLE"
            entry["time_s"] = float(match.group(3))
    return rows


def _parse_eh_seed(text: str, default: int = 42) -> int:
    for pattern in (r"Semilla\s+EH:\s*(\d+)", r"Semilla\s+base:\s*(\d+)"):
        if match := re.search(pattern, text):
            return int(match.group(1))
    return default


def _parse_cplex_routes_from_log(text: str, instance: str) -> list[list[int]] | None:
    marker = f"Rutas CPLEX {instance}:"
    for line in text.splitlines():
        if line.strip().startswith(marker):
            try:
                routes = ast.literal_eval(line.split(":", 1)[1].strip())
                if isinstance(routes, list):
                    return routes
            except (SyntaxError, ValueError):
                return None
    return None


def _parse_cplex_log_times_and_status(
    text: str,
) -> tuple[dict[str, float], dict[str, str]]:
    times: dict[str, float] = {}
    status: dict[str, str] = {}
    table_re = re.compile(r"^(C\d+R\d+)\s+[\d.]+\s+([\d.]+)s\s+(\S+)")
    for line in text.splitlines():
        if match := table_re.match(line.strip()):
            times[match.group(1)] = float(match.group(2))
            status[match.group(1)] = match.group(3)
    if times:
        return times, status
    for block in re.split(r"={10,}", text):
        if inst_m := re.search(r"Instancia:\s+(C\d+R\d+)", block):
            inst = inst_m.group(1)
            if time_m := re.search(r"Tiempo:\s+([\d.]+)\s*s", block):
                times[inst] = float(time_m.group(1))
            if state_m := re.search(r"Estado:\s+([^|]+)", block):
                status[inst] = state_m.group(1).strip()
    return times, status


def _parse_cplex_log_energies(text: str) -> dict[str, float]:
    """Energía CPLEX por instancia desde logs/run_NNN_small_cplex.txt."""
    energies: dict[str, float] = {}
    table_re = re.compile(r"^(C\d+R\d+)\s+([\d.]+)\s+[\d.]+s\s+\w+")
    for line in text.splitlines():
        if match := table_re.match(line.strip()):
            energies[match.group(1)] = float(match.group(2))
    if energies:
        return energies
    block_re = re.compile(r"^Instancia:\s+(C\d+R\d+)", re.MULTILINE)
    energy_re = re.compile(r"Estado:.*\|\s+Energía:\s+([\d.]+)\s+kWh", re.MULTILINE)
    blocks = list(block_re.finditer(text))
    energy_hits = list(energy_re.finditer(text))
    for idx, block in enumerate(blocks):
        if idx < len(energy_hits):
            energies[block.group(1)] = float(energy_hits[idx].group(1))
    return energies


def _load_small_from_logs(run_n: int) -> dict:
    """Fallback: arma datos desde logs de texto (sin JSON ni rutas/historia)."""
    from main import absolute_gap

    eh_log = LOGS_DIR / f"run_{run_n:03d}_small.txt"
    if not eh_log.exists():
        raise FileNotFoundError(f"No existe: {eh_log}")

    eh_text = eh_log.read_text(encoding="utf-8")
    eh_table = _parse_eh_log_table(eh_text)
    cplex_log = LOGS_DIR / f"run_{run_n:03d}_small_cplex.txt"
    cplex_text = cplex_log.read_text(encoding="utf-8") if cplex_log.exists() else ""
    cplex_e = _parse_cplex_log_energies(cplex_text) if cplex_text else {}
    cplex_t, cplex_st = (
        _parse_cplex_log_times_and_status(cplex_text) if cplex_text else ({}, {})
    )

    small: list[dict] = []
    for inst in sorted(set(eh_table) | set(cplex_e)):
        entry = eh_table.get(inst, {})
        energy = entry.get("energy")
        eh_energy = energy if isinstance(energy, float) else None
        mip_energy = cplex_e.get(inst)
        gap_pct = None
        if eh_energy is not None and mip_energy is not None and mip_energy > 0:
            gap_pct = absolute_gap(eh_energy, mip_energy)
        small.append(
            {
                "instance": inst,
                "eh_energy": eh_energy,
                "energy": eh_energy,
                "mip_energy": mip_energy,
                "cplex_energy": mip_energy,
                "eh_time_s": entry.get("time_s"),
                "time_s": entry.get("time_s"),
                "cplex_time_s": cplex_t.get(inst),
                "cplex_status": cplex_st.get(inst),
                "gap_pct": gap_pct,
                "mip_routes": (
                    _parse_cplex_routes_from_log(cplex_text, inst)
                    if cplex_text
                    else None
                ),
            }
        )

    seed = _parse_eh_seed(eh_text)
    print(
        f"[stats] Sin JSON; usando log {eh_log.name}"
        + (f" + {cplex_log.name}" if cplex_log.exists() else "")
        + f" (semilla EH: {seed}; se re-ejecutará EH para evolución y mapas)."
    )
    return {
        "mode": "small",
        "seed": seed,
        "small": small,
        "_from_log": True,
        "_cplex_log_text": cplex_text,
        "_run_n": run_n,
    }


def load_run_data(mode: str, run_n: int) -> dict:
    slug = _normalize_mode_slug(mode)
    json_path = LOGS_DIR / f"run_{run_n:03d}_{slug}.json"
    if json_path.exists():
        return json.loads(json_path.read_text(encoding="utf-8"))

    log_path = LOGS_DIR / f"run_{run_n:03d}_{slug}.txt"
    if slug == "small" and log_path.exists():
        return _load_small_from_logs(run_n)

    raise FileNotFoundError(
        f"No hay datos para run {run_n:03d} modo {slug}.\n"
        f"  Esperado: {json_path.name} (generado al terminar python main.py -{slug})\n"
        f"  o para -small: {log_path.name} (+ opcional run_{run_n:03d}_small_cplex.txt)"
    )


def _run_number(arg: str) -> int:
    digits = re.sub(r"\D", "", arg)
    if not digits:
        raise ValueError(f"Número de run inválido: {arg!r}")
    return int(digits)


def _find_latest(slug: str) -> int:
    nums: list[int] = []
    for ext in ("json", "txt"):
        pattern = re.compile(rf"^run_(\d{{3}})_{re.escape(slug)}\.{ext}$")
        for p in LOGS_DIR.glob(f"run_*_{slug}.{ext}"):
            if m := pattern.match(p.name):
                nums.append(int(m.group(1)))
    if not nums:
        raise FileNotFoundError(
            f"No hay archivos run_NNN_{slug}.json ni .txt en {LOGS_DIR}"
        )
    return max(nums)


EVO_GROUP_SIZE = 5
EVO_INSTANCE_COLORS = [
    "#1f77b4",
    "#ff7f0e",
    "#2ca02c",
    "#d62728",
    "#9467bd",
    "#8c564b",
    "#e377c2",
    "#7f7f7f",
    "#bcbd22",
    "#17becf",
]


def _instance_sort_key(name: str) -> tuple[int, int, int]:
    match = re.match(r"C(\d+)R(\d+)(?:-(\d+))?", name)
    if match:
        return (int(match.group(1)), int(match.group(2)), int(match.group(3) or 0))
    return (0, 0, 0)


def _chunk_rows(rows: list[dict], group_size: int = EVO_GROUP_SIZE) -> list[list[dict]]:
    ordered = sorted(rows, key=lambda r: _instance_sort_key(r["instance"]))
    return [ordered[i : i + group_size] for i in range(0, len(ordered), group_size)]


def _group_range_label(rows: list[dict]) -> str:
    names = [r["instance"] for r in rows]
    return names[0] if len(names) == 1 else f"{names[0]} - {names[-1]}"


@dataclass
class _EvoSeries:
    label: str
    history: list[dict]
    color: str
    linestyle: str = "-"
    linewidth: float = 1.4
    alpha: float = 0.85


def _plot_series_on_axes(
    ax, series_list: list[_EvoSeries], x_key: str, y_key: str
) -> None:
    for s in series_list:
        if not s.history:
            continue
        xs = [h[x_key] for h in s.history]
        ys = [h[y_key] for h in s.history]
        ax.plot(
            xs,
            ys,
            color=s.color,
            linestyle=s.linestyle,
            linewidth=s.linewidth,
            alpha=s.alpha,
            label=s.label,
        )


def _save_grouped_evolution_charts(
    prefix: str,
    group_index: int,
    group_label: str,
    series_list: list[_EvoSeries],
    category: str,
    *,
    title_prefix: str = "Evolución FO",
) -> list[Path]:
    """Un par evo_iter + evo_time con varias series (instancias) por gráfico."""
    if not MATPLOTLIB_AVAILABLE or not series_list:
        return []

    saved: list[Path] = []
    gid = f"g{group_index}"
    suptitle = f"{title_prefix} - {group_label}"

    for x_key, x_label, suffix in [
        ("iter", "Iteración", "evo_iter"),
        ("t", "Tiempo (s)", "evo_time"),
    ]:
        fig, axes = plt.subplots(1, 2, figsize=(15, 5.5))
        _plot_series_on_axes(axes[0], series_list, x_key, "current_f")
        _plot_series_on_axes(axes[1], series_list, x_key, "best_f")

        axes[0].set_title("FO actual (exploración)")
        axes[0].set_xlabel(x_label)
        axes[0].set_ylabel("f_gen")
        axes[0].grid(True, alpha=0.3)
        axes[0].legend(fontsize=7, loc="best", ncol=1)

        axes[1].set_title("Mejor FO (convergencia)")
        axes[1].set_xlabel(x_label)
        axes[1].set_ylabel("f_gen best")
        axes[1].grid(True, alpha=0.3)
        axes[1].legend(fontsize=7, loc="best", ncol=1)

        fig.suptitle(suptitle, fontsize=12, fontweight="bold")
        fig.tight_layout()
        out = stats_png_path(prefix, f"{suffix}_{gid}", category)
        fig.savefig(out, dpi=130, bbox_inches="tight")
        plt.close(fig)
        saved.append(out)
    return saved


def _small_evolution_series(rows: list[dict]) -> list[_EvoSeries]:
    series: list[_EvoSeries] = []
    for i, row in enumerate(rows):
        hist = row.get("history") or []
        if not hist:
            continue
        inst = row["instance"]
        color = EVO_INSTANCE_COLORS[i % len(EVO_INSTANCE_COLORS)]
        series.append(_EvoSeries(label=inst, history=hist, color=color))
    return series


def _large_evolution_series(row: dict, color: str) -> list[_EvoSeries]:
    inst = row["instance"]
    out: list[_EvoSeries] = []
    best_hist = row.get("best_history") or []
    worst_hist = row.get("worst_history") or []
    if best_hist:
        be = row.get("best_energy") or row.get("best")
        lbl = f"{inst} mejor ({be:.0f} kWh)" if be is not None else f"{inst} mejor"
        out.append(
            _EvoSeries(
                label=lbl, history=best_hist, color=color, linestyle="-", linewidth=1.6
            )
        )
    if worst_hist:
        we = row.get("worst_energy") or row.get("worst")
        lbl = f"{inst} peor ({we:.0f} kWh)" if we is not None else f"{inst} peor"
        out.append(
            _EvoSeries(
                label=lbl,
                history=worst_hist,
                color=color,
                linestyle="--",
                linewidth=1.2,
                alpha=0.75,
            )
        )
    return out


def save_grouped_evolution_from_rows(
    prefix: str,
    rows: list[dict],
    category: str,
    *,
    mode: str = "small",
    title_prefix: str = "Evolución FO",
    group_size: int = EVO_GROUP_SIZE,
) -> list[Path]:
    """Genera evo_iter_g* y evo_time_g* agrupando instancias (group_size por gráfico)."""
    if mode == "small":
        eligible = [r for r in rows if r.get("history")]
    else:
        eligible = [r for r in rows if r.get("best_history") or r.get("worst_history")]
    if not eligible:
        return []

    saved: list[Path] = []
    for g_idx, chunk in enumerate(_chunk_rows(eligible, group_size), start=1):
        if mode == "small":
            series = _small_evolution_series(chunk)
        else:
            series = []
            for i, row in enumerate(chunk):
                color = EVO_INSTANCE_COLORS[i % len(EVO_INSTANCE_COLORS)]
                series.extend(_large_evolution_series(row, color))
        saved.extend(
            _save_grouped_evolution_charts(
                prefix,
                g_idx,
                _group_range_label(chunk),
                series,
                category,
                title_prefix=title_prefix,
            )
        )
    return saved


def _fmt_comparative_num(value: float | None, width: int, decimals: int = 2) -> str:
    if value is None:
        return f"{'---':>{width}}"
    return f"{value:>{width}.{decimals}f}"


def _fmt_comparative_time(value: float | None, width: int) -> str:
    if value is None:
        return f"{'---':>{width}}"
    return f"{value:>{width}.1f}s"


def _fmt_comparative_gap(value: float | None, width: int) -> str:
    if value is None:
        return f"{'---':>{width}}"
    return f"{value:>{width}.2f}%"


def _normalize_small_row(row: dict) -> dict:
    r = dict(row)
    if r.get("eh_energy") is None and r.get("energy") is not None:
        r["eh_energy"] = r["energy"]
    if r.get("eh_time_s") is None and r.get("time_s") is not None:
        r["eh_time_s"] = r["time_s"]
    if r.get("mip_energy") is None and r.get("cplex_energy") is not None:
        r["mip_energy"] = r["cplex_energy"]
    return r


def _merge_cplex_log_into_small_rows(run_n: int, rows: list[dict]) -> list[dict]:
    """Enriquece filas EH con energía/tiempo/estado/rutas CPLEX desde el log."""
    from main import absolute_gap

    cplex_log = LOGS_DIR / f"run_{run_n:03d}_{CPLEX_SMALL_SLUG}.txt"
    if not cplex_log.exists():
        return [_normalize_small_row(r) for r in rows]

    text = cplex_log.read_text(encoding="utf-8")
    cplex_e = _parse_cplex_log_energies(text)
    cplex_t, cplex_st = _parse_cplex_log_times_and_status(text)
    merged: list[dict] = []
    for row in rows:
        r = _normalize_small_row(row)
        inst = r["instance"]
        if inst in cplex_e:
            r["mip_energy"] = cplex_e[inst]
            r["cplex_energy"] = cplex_e[inst]
        if inst in cplex_t:
            r["cplex_time_s"] = cplex_t[inst]
        if inst in cplex_st:
            r["cplex_status"] = cplex_st[inst]
        eh = r.get("eh_energy")
        ref = r.get("mip_energy")
        if eh is not None and ref is not None and ref > 0:
            r["gap_pct"] = absolute_gap(eh, ref)
        if not r.get("mip_routes"):
            routes = _parse_cplex_routes_from_log(text, inst)
            if routes:
                r["mip_routes"] = routes
        merged.append(r)
    return merged


def write_comparative_txt(
    run_n: int,
    rows: list[dict],
    *,
    eh_log: Path | None = None,
    cplex_log: Path | None = None,
) -> Path | None:
    """Genera stats/small/comparative_NNN_small.txt (EH vs CPLEX)."""
    eh_log = eh_log or (LOGS_DIR / f"run_{run_n:03d}_{SMALL_SLUG}.txt")
    cplex_log = cplex_log or (LOGS_DIR / f"run_{run_n:03d}_{CPLEX_SMALL_SLUG}.txt")
    if not cplex_log.exists():
        print(
            f"[stats] Sin {cplex_log.name}; omitiendo comparative_{run_n:03d}_{SMALL_SLUG}.txt"
        )
        return None

    STATS_SMALL_DIR.mkdir(parents=True, exist_ok=True)
    out_path = STATS_SMALL_DIR / f"comparative_{run_n:03d}_{SMALL_SLUG}.txt"

    comparable = [
        r
        for r in rows
        if r.get("gap_pct") is not None
        and r.get("eh_energy") is not None
        and r.get("mip_energy") is not None
    ]

    lines: list[str] = [
        "=" * 88,
        f"COMPARATIVA EH-SA/TS vs CPLEX - run {run_n:03d}",
        "=" * 88,
        f"Log EH:    logs/{eh_log.name}",
        f"Log CPLEX: logs/{cplex_log.name}",
        "Gap % = (E_EH - E_CPLEX) / E_CPLEX x 100%",
        "",
        f"{'Instancia':<10} {'E_CPLEX':>10} {'t_CPLEX':>8} {'E_EH':>10} "
        f"{'t_EH':>8} {'Gap %':>9} {'Estado CPLEX':<18}",
        "-" * 88,
    ]

    for row in sorted(rows, key=lambda r: _instance_sort_key(r["instance"])):
        lines.append(
            f"{row['instance']:<10} "
            f"{_fmt_comparative_num(row.get('mip_energy'), 10)} "
            f"{_fmt_comparative_time(row.get('cplex_time_s'), 8)} "
            f"{_fmt_comparative_num(row.get('eh_energy'), 10)} "
            f"{_fmt_comparative_time(row.get('eh_time_s'), 8)} "
            f"{_fmt_comparative_gap(row.get('gap_pct'), 9)} "
            f"{(row.get('cplex_status') or '---'):<18}"
        )

    lines.append("-" * 88)
    if comparable:
        n = len(comparable)
        avg_cplex_e = sum(r["mip_energy"] for r in comparable) / n
        avg_eh_e = sum(r["eh_energy"] for r in comparable) / n
        avg_gap = sum(r["gap_pct"] for r in comparable) / n
        cplex_times = [
            r["cplex_time_s"] for r in comparable if r["cplex_time_s"] is not None
        ]
        eh_times = [r["eh_time_s"] for r in comparable if r["eh_time_s"] is not None]
        avg_cplex_t = sum(cplex_times) / len(cplex_times) if cplex_times else None
        avg_eh_t = sum(eh_times) / len(eh_times) if eh_times else None
        lines.append(
            f"{'Promedio':<10} "
            f"{_fmt_comparative_num(avg_cplex_e, 10)} "
            f"{_fmt_comparative_time(avg_cplex_t, 8)} "
            f"{_fmt_comparative_num(avg_eh_e, 10)} "
            f"{_fmt_comparative_time(avg_eh_t, 8)} "
            f"{_fmt_comparative_gap(avg_gap, 9)} "
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


def _enrich_small_rows(
    rows: list[dict],
    *,
    seed: int,
    cplex_log_text: str = "",
    with_history: bool = True,
    with_routes: bool = True,
) -> list[dict]:
    """Completa filas con history/iteraciones/rutas re-ejecutando EH-SA/TS."""
    from main import _run_eh_on_instance

    enriched: list[dict] = []
    for row in rows:
        inst = row["instance"]
        new_row = dict(row)
        need_hist = with_history and not new_row.get("history")
        need_routes = with_routes and not (
            new_row.get("eh_routes") or new_row.get("eh_paths")
        )
        if need_hist or need_routes:
            print(f"  [stats] EH {inst}...", end=" ", flush=True)
            try:
                eh = _run_eh_on_instance(inst, seed, collect_history=True)
                if need_hist:
                    new_row["history"] = eh.get("history") or []
                    new_row["iterations"] = eh.get("iterations", 0)
                if need_routes and eh.get("feasible"):
                    paths = eh.get("eh_paths")
                    new_row["eh_routes"] = paths
                    new_row["eh_paths"] = paths
                print(f"{eh.get('time_s', 0):.1f}s")
            except Exception as exc:
                print(f"error ({exc})")
        if with_routes and cplex_log_text and not new_row.get("mip_routes"):
            cplex_routes = _parse_cplex_routes_from_log(cplex_log_text, inst)
            if cplex_routes:
                new_row["mip_routes"] = cplex_routes
        enriched.append(new_row)
    return enriched


def generate_small_charts(
    prefix: str,
    data: dict,
    run_n: int,
    *,
    with_maps: bool = True,
    with_evolution: bool = True,
    write_comparative: bool = True,
) -> list[Path]:
    if not MATPLOTLIB_AVAILABLE or not MAIN_AVAILABLE:
        print("[stats] matplotlib o main.py no disponible.")
        return []

    small_rows = _merge_cplex_log_into_small_rows(run_n, list(data.get("small") or []))
    seed = int(data.get("seed", 42))
    cplex_text = data.get("_cplex_log_text") or ""
    if not cplex_text:
        cplex_log = LOGS_DIR / f"run_{run_n:03d}_{CPLEX_SMALL_SLUG}.txt"
        if cplex_log.exists():
            cplex_text = cplex_log.read_text(encoding="utf-8")

    saved: list[Path] = []
    if write_comparative:
        comp = write_comparative_txt(run_n, small_rows)
        if comp:
            saved.append(comp)
            print(f"Comparativa: {comp.relative_to(PROJECT_DIR)}")

    if with_maps or with_evolution:
        needs_enrich = any(
            not r.get("history") or not (r.get("eh_routes") or r.get("eh_paths"))
            for r in small_rows
        )
        if needs_enrich or data.get("_from_log"):
            print("[stats] Completando datos (evolución FO / mapas)...")
            small_rows = _enrich_small_rows(
                small_rows,
                seed=seed,
                cplex_log_text=cplex_text,
                with_history=with_evolution,
                with_routes=with_maps,
            )

    def add(p):
        if p:
            saved.append(p) if not isinstance(p, list) else saved.extend(p)

    gap_rows = [
        r
        for r in small_rows
        if r.get("mip_energy") is not None and r.get("eh_energy") is not None
    ]
    eh_rows = [r for r in small_rows if r.get("eh_energy") is not None]

    if gap_rows:
        labels = [r["instance"] for r in gap_rows]
        add(
            _save_bar_chart(
                prefix,
                "bars",
                labels,
                {
                    "CPLEX (kWh)": [r["mip_energy"] for r in gap_rows],
                    "EH-SA/TS (kWh)": [r["eh_energy"] for r in gap_rows],
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
                {"Gap % (EH)": [r["gap_pct"] for r in gap_rows]},
                category="small",
                title="Pequeñas: Absolute Gap EH-SA/TS vs CPLEX",
                ylabel="Gap (%)",
            )
        )

    if eh_rows:
        t_labels = [r["instance"] for r in small_rows]
        add(
            _save_bar_chart(
                prefix,
                "time_bars",
                t_labels,
                {
                    "t_EH (s)": [
                        r.get("eh_time_s") or r.get("time_s") or 0 for r in small_rows
                    ]
                },
                category="small",
                title="Pequeñas: tiempo de ejecución EH-SA/TS",
                ylabel="Tiempo (s)",
            )
        )
        iter_vals = [r.get("iterations") or 0 for r in small_rows]
        if any(v > 0 for v in iter_vals):
            add(
                _save_bar_chart(
                    prefix,
                    "iter_bars",
                    t_labels,
                    {"Iteraciones": iter_vals},
                    category="small",
                    title="Pequeñas: iteraciones SA/TS por instancia",
                    ylabel="Iteraciones",
                )
            )

    if with_evolution:
        add(
            save_grouped_evolution_from_rows(
                prefix,
                small_rows,
                "small",
                mode="small",
                title_prefix="Pequeñas: evolución FO",
            )
        )

    if with_maps:
        add(save_grouped_small_maps(prefix, small_rows, category="small"))

    return saved


def generate_large_charts(
    prefix: str,
    data: dict,
    *,
    with_maps: bool = True,
    with_evolution: bool = True,
) -> list[Path]:
    if not MATPLOTLIB_AVAILABLE or not MAIN_AVAILABLE:
        print("[stats] matplotlib o main.py no disponible.")
        return []

    large_rows = data.get("large") or []
    saved: list[Path] = []

    def add(p):
        if p:
            saved.append(p) if not isinstance(p, list) else saved.extend(p)

    feasible = [r for r in large_rows if r.get("best") is not None]
    if not feasible:
        print("[stats-large] Sin instancias factibles para graficar.")
        return []

    labels = [r["instance"] for r in feasible]

    add(
        _save_bar_chart(
            prefix,
            "rpd_bars",
            labels,
            {"RPD medio %": [r["mean_rpd"] for r in feasible]},
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
                "B* (kWh)": [r["best"] for r in feasible],
                "Media (kWh)": [r["mean"] for r in feasible],
            },
            category="large",
            title="Grandes: mejor y energía media EH-SA/TS",
            ylabel="Energía (kWh)",
            rotate=90,
        )
    )
    add(
        _save_bar_chart(
            prefix,
            "std_bars",
            labels,
            {"Desv. std. (kWh)": [r.get("std") or 0 for r in feasible]},
            category="large",
            title="Grandes: desviación estándar entre corridas",
            ylabel="σ Energía (kWh)",
            rotate=90,
        )
    )
    add(
        _save_bar_chart(
            prefix,
            "time_bars",
            labels,
            {"t total (s)": [r.get("total_time_s") or 0 for r in feasible]},
            category="large",
            title="Grandes: tiempo total (10 corridas)",
            ylabel="Tiempo (s)",
            rotate=90,
        )
    )

    if with_evolution:
        add(
            save_grouped_evolution_from_rows(
                prefix,
                feasible,
                "large",
                mode="large",
                title_prefix="Grandes: evolución FO",
            )
        )

    if with_maps:
        add(save_grouped_large_maps(prefix, large_rows, category="large"))

    return saved


def generate_extras_charts(prefix: str, mode: str, data: dict) -> list[Path]:
    """Gráficos para recharge-stations, battery-reserve y energy-vs-distance.

    Gráficos generados por modo
    ---------------------------
    recharge-stations:
      - recharge_energy_bars  → Energía media por grupo (R2/R4/R6/R8)
      - recharge_visits_bars  → Visitas a estación media por grupo
      - recharge_combo_bars   → Ambas series combinadas en el mismo gráfico

    battery-reserve:
      - battery_energy_bars   → Energía media por nivel de reserva (0%/10%/20%)
      - battery_visits_bars   → Visitas a estación media por nivel de reserva
      - battery_bars          → Ambas series combinadas (compatibilidad)

    energy-vs-distance:
      - evd_bars_g<N>         → Energía E_min vs E_dist por grupo de instancias
      - evd_pct_g<N>          → % incremento energía al minimizar distancia por grupo
    """
    if not MATPLOTLIB_AVAILABLE or not MAIN_AVAILABLE:
        print("[stats] matplotlib o main.py no disponible.")
        return []

    import math as _math

    def _safe(v):
        if v is None:
            return float("nan")
        try:
            f = float(v)
            return float("nan") if not _math.isfinite(f) else f
        except (TypeError, ValueError):
            return float("nan")

    saved: list[Path] = []

    def add(p):
        if p is None:
            return
        if isinstance(p, list):
            saved.extend(q for q in p if q is not None)
        else:
            saved.append(p)

    slug = _normalize_mode_slug(mode)

    if slug == "recharge-stations":
        recharge = data.get("recharge_stations") or []
        if not recharge:
            return saved
        groups = [r["group"] for r in recharge]
        energies = [_safe(r["energy"]) for r in recharge]
        visits = [_safe(r["visits"]) for r in recharge]

        add(
            _save_bar_chart(
                prefix,
                "recharge_energy_bars",
                groups,
                {"Energía media (kWh)": energies},
                category="extras",
                title="Efecto del número de estaciones - Energía media (EH-SA/TS)",
                ylabel="Energía (kWh)",
                rotate=0,
            )
        )
        add(
            _save_bar_chart(
                prefix,
                "recharge_visits_bars",
                groups,
                {"Visitas a estación (media)": visits},
                category="extras",
                title="Efecto del número de estaciones - Visitas media a estaciones",
                ylabel="Visitas",
                rotate=0,
            )
        )
        add(
            _save_bar_chart(
                prefix,
                "recharge_bars",
                groups,
                {"Energía media (kWh)": energies, "Visitas estación": visits},
                category="extras",
                title="Efecto del número de estaciones (R2-R8)",
                ylabel="Valor medio",
                rotate=0,
            )
        )

    elif slug == "battery-reserve":
        battery = data.get("battery_reserve") or []
        if not battery:
            return saved
        thresholds = [r["threshold"] for r in battery]
        energies = [_safe(r["energy"]) for r in battery]
        visits = [_safe(r["visits"]) for r in battery]
        usable = [_safe(r.get("usable_kwh")) for r in battery]

        add(
            _save_bar_chart(
                prefix,
                "battery_energy_bars",
                thresholds,
                {"Energía media (kWh)": energies},
                category="extras",
                title="Reserva mínima de batería - Energía media (EH-SA/TS)",
                ylabel="Energía (kWh)",
                rotate=0,
            )
        )

        add(
            _save_bar_chart(
                prefix,
                "battery_visits_bars",
                thresholds,
                {"Visitas a estación (media)": visits},
                category="extras",
                title="Reserva mínima de batería - Visitas a estaciones",
                ylabel="Visitas",
                rotate=0,
            )
        )

        add(
            _save_bar_chart(
                prefix,
                "battery_bars",
                thresholds,
                {"Energía media (kWh)": energies, "Visitas estación": visits},
                category="extras",
                title="Reserva mínima de batería (EH-SA/TS)",
                ylabel="Valor medio",
                rotate=0,
            )
        )

    elif slug == "energy-vs-distance":
        evd = data.get("energy_vs_distance") or []
        if not evd:
            return saved

        labels = [r["instance"] for r in evd]
        e_min = [_safe(r.get("energy_min")) for r in evd]
        e_dist = [_safe(r.get("energy_dist")) for r in evd]
        pct = [_safe(r.get("pct_inc")) for r in evd]
        times = [_safe(r.get("time_s")) for r in evd]

        GROUP_SIZE = 10
        chunks = [
            (
                labels[i : i + GROUP_SIZE],
                e_min[i : i + GROUP_SIZE],
                e_dist[i : i + GROUP_SIZE],
                pct[i : i + GROUP_SIZE],
                times[i : i + GROUP_SIZE],
            )
            for i in range(0, len(labels), GROUP_SIZE)
        ]
        for gi, (lbl, em, ed, pc, ts) in enumerate(chunks, start=1):
            tag = f"_g{gi}" if len(chunks) > 1 else ""
            range_str = f"{lbl[0]}-{lbl[-1]}" if len(lbl) > 1 else lbl[0]

            add(
                _save_bar_chart(
                    prefix,
                    f"evd_bars{tag}",
                    lbl,
                    {"E_min (kWh)": em, "E_dist (kWh)": ed},
                    category="extras",
                    title=f"Energía vs distancia - {range_str}",
                    ylabel="Energía (kWh)",
                    rotate=90,
                )
            )

            add(
                _save_bar_chart(
                    prefix,
                    f"evd_pct{tag}",
                    lbl,
                    {"% inc. energía": pc},
                    category="extras",
                    title=f"% incremento al minimizar distancia - {range_str}",
                    ylabel="%",
                    rotate=90,
                )
            )

            add(
                _save_bar_chart(
                    prefix,
                    f"evd_time{tag}",
                    lbl,
                    {"Tiempo EH (s)": ts},
                    category="extras",
                    title=f"Tiempo EH-SA/TS - {range_str}",
                    ylabel="Tiempo (s)",
                    rotate=90,
                )
            )

    return saved


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Genera gráficos EH-SA/TS desde datos de corridas.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "mode",
        help="Modo: -small | -large | recharge-stations | battery-reserve | energy-vs-distance",
    )
    parser.add_argument(
        "run",
        nargs="?",
        help="Número de run (ej. 001). Por defecto: el último disponible.",
    )
    parser.add_argument("--no-maps", action="store_true", help="Omitir mapas de rutas.")
    parser.add_argument(
        "--no-evolution", action="store_true", help="Omitir gráficos de evolución."
    )
    args = parser.parse_args(_prepare_cli_argv(sys.argv[1:]))

    slug = _normalize_mode_slug(args.mode)
    if slug not in VALID_MODE_SLUGS:
        parser.error(
            f"modo no reconocido: {args.mode!r}. "
            f"Use: -small, -large, recharge-stations, battery-reserve, energy-vs-distance"
        )

    run_n = _run_number(args.run) if args.run else _find_latest(slug)
    prefix = f"run_{run_n:03d}_{slug}"

    json_path = LOGS_DIR / f"run_{run_n:03d}_{slug}.json"
    if json_path.exists():
        print(f"Cargando datos: {json_path.name}")
    else:
        print(f"Cargando datos: run_{run_n:03d}_{slug} (log .txt)")
    data = load_run_data(slug, run_n)

    with_maps = not args.no_maps
    with_evo = not args.no_evolution

    if slug == "small":
        saved = generate_small_charts(
            prefix,
            data,
            run_n,
            with_maps=with_maps,
            with_evolution=with_evo,
        )
    elif slug == "large":
        saved = generate_large_charts(
            prefix, data, with_maps=with_maps, with_evolution=with_evo
        )
    elif slug in ("recharge-stations", "battery-reserve", "energy-vs-distance"):
        saved = generate_extras_charts(prefix, slug, data)
    else:
        print(f"Modo no reconocido: {args.mode!r}")
        return

    if saved:
        print(f"\n[stats] {len(saved)} gráfico(s) generados:")
        for p in saved:
            print(f"  - {p.relative_to(PROJECT_DIR)}")
    else:
        print(
            "\n[stats] No se generaron gráficos (sin datos o matplotlib no disponible)."
        )


if __name__ == "__main__":
    main()
