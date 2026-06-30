import csv
import re
from collections import defaultdict
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as mticker

# ─────────────────────────── Rutas ───────────────────────────
# Ajusta estas rutas según donde queden los archivos generados por main_P3.py
RESULTS_PATH = Path(__file__).parent.parent.parent / "P3" / "results.txt"
CONVERGENCE_PATH = Path(__file__).parent.parent.parent / "P3" / "convergence_data.csv"
SWEEP_PATH = Path(__file__).parent.parent.parent / "P3" / "param_sweep.csv"
OUTPUT_DIR = Path(__file__).parent

INSTANCE_ORDER = ["easy", "medium1", "medium2", "hard"]
PALETTE = ["#4C72B0", "#DD8452", "#55A868", "#C44E52"]


# ══════════════════════════════════════════════════════════════════
#  PARSERS
# ══════════════════════════════════════════════════════════════════

def parse_results(path: Path) -> dict:
    """Igual que la versión original: parsea resumen de cada instancia desde results.txt."""
    content = path.read_text(encoding="utf-8", errors="replace")
    instances = {}

    for name, block in re.findall(
        r"ACO - Instancia:\s*(\w+)(.*?)(?=ACO - Instancia|RESUMEN|\Z)",
        content,
        re.DOTALL,
    ):
        name = name.lower()

        det_section = re.search(r"\[Greedy Determinista\](.*?)(?=\[ACO)", block, re.DOTALL)
        det_text = det_section.group(1)
        det_resultado = int(re.search(r"Beneficio\s*:\s*(\d+)", det_text).group(1))
        det_time = float(re.search(r"Tiempo\s*:\s*([\d.]+)s", det_text).group(1))

        aco_section = re.search(r"\[ACO x\d+ corridas.*?\](.*?)(?=Estad[ií]sticas)", block, re.DOTALL)
        aco_runs = [
            {"run": int(m.group(1)), "benefit": int(m.group(2)), "time": float(m.group(3))}
            for m in re.finditer(
                r"Run\s+(\d+):\s*beneficio=\s*(\d+)\s+costo=\s*\d+\s+t=([\d.]+)s",
                aco_section.group(1),
            )
        ]

        stats_section = re.search(
            r"Estad[ií]sticas ACO.*?Media\s*:\s*([\d.]+).*?Desv\. Est\.\s*:\s*([\d.]+).*?Mejor\s*:\s*(\d+).*?Peor\s*:\s*(\d+).*?Mediana\s*:\s*([\d.]+)",
            block,
            re.DOTALL,
        )

        instances[name] = {
            "det_resultado": det_resultado,
            "det_time": det_time,
            "aco_runs": aco_runs,
            "aco_mean": float(stats_section.group(1)),
            "aco_std": float(stats_section.group(2)),
            "aco_best": int(stats_section.group(3)),
            "aco_worst": int(stats_section.group(4)),
            "aco_median": float(stats_section.group(5)),
        }

    return instances


def parse_convergence(path: Path) -> dict:
    """
    Lee convergence_data.csv y retorna:
        {instance: {run_id: [ {iteration, best_global, iter_best, iter_mean,
                                iter_worst, time_cum}, ... ]}}
    """
    data = defaultdict(lambda: defaultdict(list))
    if not path.exists():
        return data

    with open(path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            inst = row["instance"].lower()
            run = int(row["run"])
            data[inst][run].append({
                "iteration": int(row["iteration"]),
                "best_global": int(row["best_global"]),
                "iter_best": int(row["iter_best"]),
                "iter_mean": float(row["iter_mean"]),
                "iter_worst": float(row["iter_worst"]),
                "time_cum": float(row["time_cum"]),
            })

    for inst in data:
        for run in data[inst]:
            data[inst][run].sort(key=lambda r: r["iteration"])

    return data


def parse_sweep(path: Path) -> dict:
    """
    Lee param_sweep.csv y retorna:
        {instance: {rho_value: [benefit, benefit, ...]}}
    """
    data = defaultdict(lambda: defaultdict(list))
    if not path.exists():
        return data

    with open(path, "r", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            inst = row["instance"].lower()
            rho = float(row["rho"])
            data[inst][rho].append(int(row["benefit"]))

    return data


def best_run_id(conv_for_instance: dict) -> int:
    """Determina la corrida con mejor beneficio final (best_global de la última iteración)."""
    best_run, best_val = None, -1
    for run_id, rows in conv_for_instance.items():
        final_best = rows[-1]["best_global"]
        if final_best > best_val:
            best_val = final_best
            best_run = run_id
    return best_run


# ══════════════════════════════════════════════════════════════════
#  GRÁFICOS YA EXISTENTES (sin cambios respecto a la versión original)
# ══════════════════════════════════════════════════════════════════

def plot_convergence(instances: dict, output_dir: Path):
    fig, axes = plt.subplots(2, 2, figsize=(14, 9))
    fig.suptitle(
        "Evolución del Beneficio — Mejora de ACO vs Baseline Determinista",
        fontsize=14, fontweight="bold",
    )

    for idx, name in enumerate(INSTANCE_ORDER):
        if name not in instances:
            continue
        ax = axes[idx // 2][idx % 2]
        data = instances[name]

        run_nums = list(range(1, len(data["aco_runs"]) + 1))
        aco_results = [r["benefit"] for r in data["aco_runs"]]
        det_resultado = data["det_resultado"]
        baselines = [det_resultado] * len(run_nums)

        ax.plot(run_nums, baselines, marker="s", color="gray", linewidth=1.5,
                markersize=5, linestyle="--", label="Baseline (Greedy Det.)", alpha=0.7)
        ax.plot(run_nums, aco_results, marker="o", color=PALETTE[idx], linewidth=2,
                markersize=6, label="Resultado ACO")

        for i, (b, r) in enumerate(zip(baselines, aco_results)):
            ax.annotate("", xy=(i + 1, r), xytext=(i + 1, b),
                        arrowprops=dict(arrowstyle="->", color=PALETTE[idx], alpha=0.4, lw=1.0))

        ax.axhline(data["aco_mean"], color="red", linestyle=":", linewidth=1.5,
                  label=f"Media ACO ({data['aco_mean']:,.1f})")

        ax.set_title(f"Instancia: {name}", fontsize=11, fontweight="bold")
        ax.set_xlabel("Run")
        ax.set_ylabel("Beneficio")
        ax.set_xticks(run_nums)
        ax.legend(fontsize=7.5)
        ax.grid(True, alpha=0.3)
        ax.yaxis.set_major_formatter(mticker.FuncFormatter(lambda v, _: f"{int(v):,}"))

    plt.tight_layout()
    fig.savefig(output_dir / "convergencia_evolucion.png", dpi=150, bbox_inches="tight")
    plt.close(fig)


def plot_execution_times(instances: dict, output_dir: Path):
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    fig.suptitle("Tiempos de Ejecución — Ant Colony Optimization", fontsize=14, fontweight="bold")

    names = [n for n in INSTANCE_ORDER if n in instances]
    det_times = [instances[n]["det_time"] for n in names]
    aco_mean_times = [np.mean([r["time"] for r in instances[n]["aco_runs"]]) for n in names]
    aco_max_times = [np.max([r["time"] for r in instances[n]["aco_runs"]]) for n in names]
    aco_min_times = [np.min([r["time"] for r in instances[n]["aco_runs"]]) for n in names]

    x = np.arange(len(names))
    width = 0.35

    ax = axes[0]
    ax.bar(x - width / 2, det_times, width, label="Greedy Det.", color=PALETTE[0], alpha=0.85)
    ax.bar(x + width / 2, aco_mean_times, width, label="ACO (media)", color=PALETTE[1], alpha=0.85)
    ax.errorbar(
        x + width / 2, aco_mean_times,
        yerr=[[m - mn for m, mn in zip(aco_mean_times, aco_min_times)],
              [mx - m for mx, m in zip(aco_max_times, aco_mean_times)]],
        fmt="none", color="black", capsize=4, linewidth=1.2,
    )
    ax.set_title("Tiempo de ejecución promedio por instancia")
    ax.set_xticks(x)
    ax.set_xticklabels(names)
    ax.set_ylabel("Tiempo (s)")
    ax.legend()
    ax.grid(True, alpha=0.3, axis="y")

    ax2 = axes[1]
    for idx, name in enumerate(names):
        run_times = [r["time"] for r in instances[name]["aco_runs"]]
        ax2.boxplot(run_times, positions=[idx + 1], widths=0.5, patch_artist=True,
                   boxprops=dict(facecolor=PALETTE[idx], alpha=0.7),
                   medianprops=dict(color="black", linewidth=2),
                   whiskerprops=dict(linewidth=1.2), capprops=dict(linewidth=1.2))
    ax2.set_title("Distribución tiempos ACO (10 corridas)")
    ax2.set_xticks(range(1, len(names) + 1))
    ax2.set_xticklabels(names)
    ax2.set_ylabel("Tiempo (s)")
    ax2.grid(True, alpha=0.3, axis="y")

    plt.tight_layout()
    fig.savefig(output_dir / "tiempos_ejecucion.png", dpi=150, bbox_inches="tight")
    plt.close(fig)


def plot_solution_quality(instances: dict, output_dir: Path):
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    fig.suptitle("Calidad de la Solución — Ant Colony Optimization vs Greedy", fontsize=14, fontweight="bold")

    names = [n for n in INSTANCE_ORDER if n in instances]
    det_resultado = [instances[n]["det_resultado"] for n in names]
    aco_best = [instances[n]["aco_best"] for n in names]
    aco_mean = [instances[n]["aco_mean"] for n in names]

    x = np.arange(len(names))
    width = 0.25

    ax = axes[0]
    ax.bar(x - width, det_resultado, width, label="Greedy Det.", color=PALETTE[0], alpha=0.85)
    ax.bar(x, aco_mean, width, label="ACO Media", color=PALETTE[2], alpha=0.85)
    ax.bar(x + width, aco_best, width, label="ACO Mejor", color=PALETTE[1], alpha=1.0)

    ax.set_xticks(x)
    ax.set_xticklabels(names)
    ax.set_ylabel("Beneficio")
    ax.set_xlabel("Instancia")
    ax.set_title("Comparativa de beneficio máximo y promedio")
    ax.legend(fontsize=9)
    ax.grid(True, alpha=0.3, axis="y")
    ax.yaxis.set_major_formatter(mticker.FuncFormatter(lambda v, _: f"{int(v):,}"))

    ax2 = axes[1]
    improvement_best = [
        (instances[n]["aco_best"] - instances[n]["det_resultado"]) / instances[n]["det_resultado"] * 100
        for n in names
    ]
    improvement_mean = [
        (instances[n]["aco_mean"] - instances[n]["det_resultado"]) / instances[n]["det_resultado"] * 100
        for n in names
    ]

    x2 = np.arange(len(names))
    w2 = 0.35
    ax2.bar(x2 - w2 / 2, improvement_mean, w2, label="Mejora Media ACO", color=PALETTE[2], alpha=0.85)
    ax2.bar(x2 + w2 / 2, improvement_best, w2, label="Mejora Mejor ACO", color=PALETTE[1], alpha=0.85)

    ax2.axhline(0, color="gray", linewidth=1)
    ax2.set_xticks(x2)
    ax2.set_xticklabels(names)
    ax2.set_ylabel("Mejora (%)")
    ax2.set_xlabel("Instancia")
    ax2.set_title("Mejora porcentual de ACO sobre Greedy Determinista")
    ax2.legend()
    ax2.grid(True, alpha=0.3, axis="y")

    for i, (vmean, vbest) in enumerate(zip(improvement_mean, improvement_best)):
        ax2.text(i - w2 / 2, vmean + 0.5, f"{vmean:+.1f}%", ha="center", va="bottom", fontsize=8)
        ax2.text(i + w2 / 2, vbest + 0.5, f"{vbest:+.1f}%", ha="center", va="bottom", fontsize=8)

    plt.tight_layout()
    fig.savefig(output_dir / "calidad_solucion.png", dpi=150, bbox_inches="tight")
    plt.close(fig)


# ══════════════════════════════════════════════════════════════════
#  GRÁFICOS NUEVOS (requisitos pendientes)
# ══════════════════════════════════════════════════════════════════

def plot_population_convergence(convergence: dict, output_dir: Path):
    """
    Convergencia poblacional: mejor / promedio / peor de la colonia por
    iteración, para la corrida con mejor resultado de cada instancia
    (igual criterio que usaron en TS: 'Best FO (run X)' + resto en gris).
    """
    fig, axes = plt.subplots(2, 2, figsize=(14, 9))
    fig.suptitle(
        "Convergencia Poblacional — Mejor / Promedio / Peor por Iteración (ACO)",
        fontsize=14, fontweight="bold",
    )

    for idx, name in enumerate(INSTANCE_ORDER):
        ax = axes[idx // 2][idx % 2]
        if name not in convergence or not convergence[name]:
            ax.set_title(f"Instancia: {name} (sin datos)", fontsize=11)
            continue

        runs = convergence[name]
        rid = best_run_id(runs)
        rows = runs[rid]

        iters = [r["iteration"] for r in rows]
        best_g = [r["best_global"] for r in rows]
        iter_mean = [r["iter_mean"] for r in rows]
        iter_worst = [r["iter_worst"] for r in rows]

        ax.fill_between(iters, iter_worst, best_g, color=PALETTE[idx], alpha=0.12)
        ax.plot(iters, best_g, color=PALETTE[idx], linewidth=2.2, label=f"Mejor global (run {rid})")
        ax.plot(iters, iter_mean, color="black", linewidth=1.3, linestyle="-.",
                label="Promedio colonia (iteración)")
        ax.plot(iters, iter_worst, color="firebrick", linewidth=1.2, linestyle=":",
                label="Peor colonia (iteración)")

        ax.set_title(f"Instancia: {name}", fontsize=11, fontweight="bold")
        ax.set_xlabel("Iteración")
        ax.set_ylabel("Beneficio")
        ax.legend(fontsize=7.5)
        ax.grid(True, alpha=0.3)
        ax.yaxis.set_major_formatter(mticker.FuncFormatter(lambda v, _: f"{int(v):,}"))

    plt.tight_layout()
    fig.savefig(output_dir / "convergencia_poblacional.png", dpi=150, bbox_inches="tight")
    plt.close(fig)


def plot_best_fo_vs_time(convergence: dict, output_dir: Path):
    """
    Mejor FO acumulada en función del tiempo de ejecución acumulado, para la
    corrida con mejor resultado de cada instancia, con el resto de corridas
    en gris de fondo (análogo a la Figura 12 de Tabu Search en el informe).
    """
    fig, axes = plt.subplots(2, 2, figsize=(14, 9))
    fig.suptitle(
        "Evolución de la Mejor FO en función del Tiempo de Ejecución (ACO)",
        fontsize=14, fontweight="bold",
    )

    for idx, name in enumerate(INSTANCE_ORDER):
        ax = axes[idx // 2][idx % 2]
        if name not in convergence or not convergence[name]:
            ax.set_title(f"Instancia: {name} (sin datos)", fontsize=11)
            continue

        runs = convergence[name]
        rid = best_run_id(runs)

        # Todas las corridas en gris de fondo
        for run_id, rows in runs.items():
            if run_id == rid:
                continue
            t = [r["time_cum"] for r in rows]
            b = [r["best_global"] for r in rows]
            ax.plot(t, b, color="gray", alpha=0.25, linewidth=1.0)

        rows_best = runs[rid]
        t_best = [r["time_cum"] for r in rows_best]
        b_best = [r["best_global"] for r in rows_best]
        ax.plot(t_best, b_best, color=PALETTE[idx], linewidth=2.2, label=f"Best FO (run {rid})")

        ax.set_title(f"Instancia: {name}", fontsize=11, fontweight="bold")
        ax.set_xlabel("Tiempo (s)")
        ax.set_ylabel("Best FO (beneficio acumulado)")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.yaxis.set_major_formatter(mticker.FuncFormatter(lambda v, _: f"{int(v):,}"))

    plt.tight_layout()
    fig.savefig(output_dir / "mejor_fo_vs_tiempo.png", dpi=150, bbox_inches="tight")
    plt.close(fig)


def plot_parameter_sweep(sweep: dict, output_dir: Path):
    """
    Gráfico de calibración de parámetros: mejor y media del beneficio en
    función de rho, una curva por instancia (solo las instancias que tengan
    datos de barrido), análogo a la Figura 6 (tenure) de Tabu Search.
    """
    names = [n for n in INSTANCE_ORDER if n in sweep and sweep[n]]
    if not names:
        print("  [AVISO] No hay datos de param_sweep.csv; se omite el gráfico de calibración.")
        return

    fig, axes = plt.subplots(1, 2, figsize=(13, 5))
    fig.suptitle("Calibración de Parámetros — rho (tasa de evaporación) en ACO",
                fontsize=14, fontweight="bold")

    for idx, name in enumerate(names):
        rho_values = sorted(sweep[name].keys())
        best_vals = [max(sweep[name][r]) for r in rho_values]
        mean_vals = [np.mean(sweep[name][r]) for r in rho_values]

        color = PALETTE[INSTANCE_ORDER.index(name) % len(PALETTE)]
        axes[0].plot(rho_values, best_vals, marker="o", color=color, label=name)
        axes[1].plot(rho_values, mean_vals, marker="o", color=color, label=name)

    axes[0].set_title("Mejor solución vs rho")
    axes[0].set_xlabel("rho")
    axes[0].set_ylabel("Beneficio")
    axes[0].legend()
    axes[0].grid(True, alpha=0.3)

    axes[1].set_title("Media vs rho")
    axes[1].set_xlabel("rho")
    axes[1].set_ylabel("Beneficio")
    axes[1].legend()
    axes[1].grid(True, alpha=0.3)

    plt.tight_layout()
    fig.savefig(output_dir / "calibracion_parametros_rho.png", dpi=150, bbox_inches="tight")
    plt.close(fig)


# ══════════════════════════════════════════════════════════════════
#  MAIN
# ══════════════════════════════════════════════════════════════════

def main():
    if not RESULTS_PATH.exists():
        print(f"Error: no se encontró el archivo {RESULTS_PATH}")
        return

    instances = parse_results(RESULTS_PATH)
    if not instances:
        print("Error: No se lograron parsear instancias. Revisa el formato de results.txt")
        return

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    # Gráficos existentes
    plot_convergence(instances, OUTPUT_DIR)
    plot_execution_times(instances, OUTPUT_DIR)
    plot_solution_quality(instances, OUTPUT_DIR)

    # Gráficos nuevos (requieren los CSV generados por la versión extendida de main_P3.py)
    convergence = parse_convergence(CONVERGENCE_PATH)
    if convergence:
        plot_population_convergence(convergence, OUTPUT_DIR)
        plot_best_fo_vs_time(convergence, OUTPUT_DIR)
    else:
        print(f"  [AVISO] No se encontró o está vacío: {CONVERGENCE_PATH}. "
              f"Corre primero la versión extendida de main_P3.py.")

    sweep = parse_sweep(SWEEP_PATH)
    if sweep:
        plot_parameter_sweep(sweep, OUTPUT_DIR)
    else:
        print(f"  [AVISO] No se encontró o está vacío: {SWEEP_PATH}.")

    print(f"Gráficas generadas exitosamente en: {OUTPUT_DIR}")


if __name__ == "__main__":
    main()