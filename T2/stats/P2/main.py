import re
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as mticker
from pathlib import Path

RESULTS_PATH = Path(__file__).parent.parent.parent / "P2" / "results.txt"
OUTPUT_DIR = Path(__file__).parent

INSTANCE_ORDER = ["easy", "medium1", "medium2", "hard"]
PALETTE = ["#4C72B0", "#DD8452", "#55A868", "#C44E52"]


def parse_int_history(line_prefix, block):
    match = re.search(rf"{re.escape(line_prefix)}:\s*([\d\s]+)", block)
    if match:
        return list(map(int, match.group(1).split()))
    return []


def parse_float_history(line_prefix, block):
    match = re.search(rf"{re.escape(line_prefix)}:\s*([\d.\s]+)", block)
    if match:
        return list(map(float, match.group(1).split()))
    return []


def parse_global_tenure(content):
    match = re.search(r"Tenure usado:\s*(\d+)", content)
    if match:
        return int(match.group(1))
    return None


def parse_instance_tenure(block):
    match = re.search(r"Tenure usado:\s*(\d+)", block)
    if match:
        return int(match.group(1))
    match = re.search(r"tenure=(\d+)", block)
    if match:
        return int(match.group(1))
    return None


def instance_title(name, tenure):
    if tenure is not None:
        return f"Instancia: {name} (tenure={tenure})"
    return f"Instancia: {name}"


def parse_results(path: Path) -> tuple[dict, int | None]:
    content = path.read_text(encoding="utf-8", errors="replace")
    global_tenure = parse_global_tenure(content)
    instances = {}

    for name, block in re.findall(
        r"TABU SEARCH.*?Instancia:\s*(\w+)(.*?)(?=TABU SEARCH|RESUMEN|\Z)",
        content,
        re.DOTALL,
    ):
        name = name.lower()
        tenure = parse_instance_tenure(block)

        det_section = re.search(
            r"\[TS desde Greedy Determinista\](.*?)(?=\[TS desde Greedy)",
            block,
            re.DOTALL,
        )
        det_text = det_section.group(1) if det_section else ""
        det_inicio = int(re.search(r"Inicio\s*:.*?beneficio=(\d+)", det_text).group(1))
        det_resultado = int(
            re.search(r"Resultado:.*?beneficio=(\d+)", det_text).group(1)
        )
        det_time = float(re.search(r"Tiempo\s*:\s*([\d.]+)s", det_text).group(1))
        best_hist_det = parse_int_history("Best History Det", det_text)
        current_hist_det = parse_int_history("Current History Det", det_text)
        iter_times_det = parse_float_history("Iter Times Det", det_text)

        sto_section = re.search(
            r"\[TS desde Greedy Estoc[aá]stico.*?\](.*?)(?=Estad[ií]sticas)",
            block,
            re.DOTALL,
        )
        sto_text = sto_section.group(1) if sto_section else ""

        sto_runs = []
        for match in re.finditer(
            r"Run\s+(\d+):\s*partida=\s*(\d+)\s+TS=\s*(\d+)\s+costo=\s*\d+\s+t=([\d.]+)s",
            sto_text,
        ):
            run_num = int(match.group(1))
            sto_runs.append(
                {
                    "run": run_num,
                    "partida": int(match.group(2)),
                    "ts_result": int(match.group(3)),
                    "time": float(match.group(4)),
                    "best_hist": parse_int_history(
                        f"Best History Sto {run_num}", sto_text
                    ),
                    "curr_hist": parse_int_history(
                        f"Current History Sto {run_num}", sto_text
                    ),
                    "iter_times": parse_float_history(
                        f"Iter Times Sto {run_num}", sto_text
                    ),
                }
            )

        stats_match = re.search(
            r"Estad[ií]sticas TS.*?Media\s*:\s*([\d.]+).*?Desv\. Est\.\s*:\s*([\d.]+)"
            r".*?Mejor\s*:\s*(\d+).*?Peor\s*:\s*(\d+).*?Mediana\s*:\s*([\d.]+)",
            block,
            re.DOTALL,
        )
        best_global = int(re.search(r"Mejor global.*?:\s*(\d+)", block).group(1))

        instances[name] = {
            "tenure": tenure,
            "det_inicio": det_inicio,
            "det_resultado": det_resultado,
            "det_time": det_time,
            "best_hist_det": best_hist_det,
            "current_hist_det": current_hist_det,
            "iter_times_det": iter_times_det,
            "sto_runs": sto_runs,
            "sto_mean": float(stats_match.group(1)) if stats_match else 0.0,
            "sto_std": float(stats_match.group(2)) if stats_match else 0.0,
            "sto_best": int(stats_match.group(3)) if stats_match else 0,
            "sto_worst": int(stats_match.group(4)) if stats_match else 0,
            "sto_median": float(stats_match.group(5)) if stats_match else 0.0,
            "best_global": best_global,
        }

    return instances, global_tenure


def plot_convergence(instances, output_dir, global_tenure):
    tenure_label = global_tenure if global_tenure is not None else "N/A"
    fig, axes = plt.subplots(2, 2, figsize=(14, 9))
    fig.suptitle(
        f"Evolucion del Beneficio - Tabu Search (tenure={tenure_label})",
        fontsize=14,
        fontweight="bold",
    )

    for idx, name in enumerate(INSTANCE_ORDER):
        ax = axes[idx // 2][idx % 2]
        data = instances[name]
        runs = data["sto_runs"]
        run_nums = [run["run"] for run in runs]
        partidas = [run["partida"] for run in runs]
        ts_results = [run["ts_result"] for run in runs]

        ax.plot(
            run_nums,
            partidas,
            marker="s",
            color="gray",
            linewidth=1.5,
            markersize=5,
            linestyle="--",
            label="Partida (Greedy Sto.)",
            alpha=0.7,
        )
        ax.plot(
            run_nums,
            ts_results,
            marker="o",
            color=PALETTE[idx],
            linewidth=2,
            markersize=6,
            label="Resultado TS",
        )

        for i, (partida, ts_result) in enumerate(zip(partidas, ts_results)):
            ax.annotate(
                "",
                xy=(i + 1, ts_result),
                xytext=(i + 1, partida),
                arrowprops=dict(arrowstyle="->", color=PALETTE[idx], alpha=0.4, lw=1.0),
            )

        ax.axhline(
            data["det_resultado"],
            color="navy",
            linestyle="-.",
            linewidth=1.5,
            label=f"TS/Det. ({data['det_resultado']:,})",
        )
        ax.axhline(
            data["sto_mean"],
            color="red",
            linestyle=":",
            linewidth=1.5,
            label=f"Media TS ({data['sto_mean']:,.1f})",
        )

        ax.set_title(
            instance_title(name, data.get("tenure")), fontsize=11, fontweight="bold"
        )
        ax.set_xlabel("Run")
        ax.set_ylabel("Beneficio")
        ax.set_xticks(run_nums)
        ax.legend(fontsize=7.5)
        ax.grid(True, alpha=0.3)
        ax.yaxis.set_major_formatter(mticker.FuncFormatter(lambda v, _: f"{int(v):,}"))

    plt.tight_layout()
    fig.savefig(output_dir / "convergencia_evolucion.png", dpi=150, bbox_inches="tight")
    plt.close(fig)
    print("  convergencia_evolucion.png generada")


def plot_execution_times(instances, output_dir, global_tenure):
    tenure_label = global_tenure if global_tenure is not None else "N/A"
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    fig.suptitle(
        f"Tiempos de Ejecucion - Tabu Search (tenure={tenure_label})",
        fontsize=14,
        fontweight="bold",
    )

    names = INSTANCE_ORDER
    det_times = [instances[name]["det_time"] for name in names]
    sto_mean_times = [
        np.mean([run["time"] for run in instances[name]["sto_runs"]]) for name in names
    ]
    sto_max_times = [
        np.max([run["time"] for run in instances[name]["sto_runs"]]) for name in names
    ]
    sto_min_times = [
        np.min([run["time"] for run in instances[name]["sto_runs"]]) for name in names
    ]

    x = np.arange(len(names))
    width = 0.35

    ax = axes[0]
    ax.bar(
        x - width / 2, det_times, width, label="TS/Det.", color=PALETTE[0], alpha=0.85
    )
    ax.bar(
        x + width / 2,
        sto_mean_times,
        width,
        label="TS/Sto. (media)",
        color=PALETTE[1],
        alpha=0.85,
    )
    ax.errorbar(
        x + width / 2,
        sto_mean_times,
        yerr=[
            [mean - min_time for mean, min_time in zip(sto_mean_times, sto_min_times)],
            [max_time - mean for max_time, mean in zip(sto_max_times, sto_mean_times)],
        ],
        fmt="none",
        color="black",
        capsize=4,
        linewidth=1.2,
    )
    ax.set_title("Tiempo de ejecucion por instancia")
    ax.set_xticks(x)
    ax.set_xticklabels(names)
    ax.set_ylabel("Tiempo (s)")
    ax.legend()
    ax.grid(True, alpha=0.3, axis="y")

    ax2 = axes[1]
    for idx, name in enumerate(names):
        run_times = [run["time"] for run in instances[name]["sto_runs"]]
        ax2.boxplot(
            run_times,
            positions=[idx + 1],
            widths=0.5,
            patch_artist=True,
            boxprops=dict(facecolor=PALETTE[idx], alpha=0.7),
            medianprops=dict(color="black", linewidth=2),
            whiskerprops=dict(linewidth=1.2),
            capprops=dict(linewidth=1.2),
        )
    ax2.set_title("Distribucion tiempos TS estocastico")
    ax2.set_xticks(range(1, len(names) + 1))
    ax2.set_xticklabels(names)
    ax2.set_ylabel("Tiempo (s)")
    ax2.grid(True, alpha=0.3, axis="y")

    plt.tight_layout()
    fig.savefig(output_dir / "tiempos_ejecucion.png", dpi=150, bbox_inches="tight")
    plt.close(fig)
    print("  tiempos_ejecucion.png generada")


def plot_solution_quality(instances, output_dir, global_tenure):
    tenure_label = global_tenure if global_tenure is not None else "N/A"
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    fig.suptitle(
        f"Calidad de la Solucion - Tabu Search vs Greedy (tenure={tenure_label})",
        fontsize=14,
        fontweight="bold",
    )

    names = INSTANCE_ORDER
    det_ini = [instances[name]["det_inicio"] for name in names]
    ts_det = [instances[name]["det_resultado"] for name in names]
    sto_best_partida = [
        max(run["partida"] for run in instances[name]["sto_runs"]) for name in names
    ]
    ts_sto_best = [instances[name]["sto_best"] for name in names]
    ts_sto_mean = [instances[name]["sto_mean"] for name in names]

    x = np.arange(len(names))
    width = 0.15

    ax = axes[0]
    ax.bar(
        x - 2 * width, det_ini, width, label="Greedy Det.", color=PALETTE[0], alpha=0.75
    )
    ax.bar(x - 1 * width, ts_det, width, label="TS/Det.", color=PALETTE[0], alpha=1.0)
    ax.bar(
        x + 0 * width,
        sto_best_partida,
        width,
        label="Greedy Sto. Mejor",
        color=PALETTE[1],
        alpha=0.75,
    )
    ax.bar(
        x + 1 * width,
        ts_sto_best,
        width,
        label="TS/Sto. Mejor",
        color=PALETTE[1],
        alpha=1.0,
    )
    ax.bar(
        x + 2 * width,
        ts_sto_mean,
        width,
        label="TS/Sto. Media",
        color=PALETTE[2],
        alpha=0.85,
    )
    ax.set_xticks(x)
    ax.set_xticklabels(names)
    ax.set_ylabel("Beneficio")
    ax.set_xlabel("Instancia")
    ax.set_title("Comparativa de beneficio")
    ax.legend(fontsize=8)
    ax.grid(True, alpha=0.3, axis="y")
    ax.yaxis.set_major_formatter(mticker.FuncFormatter(lambda v, _: f"{int(v):,}"))

    ax2 = axes[1]
    improvement_det = [
        (instances[name]["det_resultado"] - instances[name]["det_inicio"])
        / instances[name]["det_inicio"]
        * 100
        for name in names
    ]
    improvement_sto = [
        (
            instances[name]["sto_best"]
            - max(run["partida"] for run in instances[name]["sto_runs"])
        )
        / max(run["partida"] for run in instances[name]["sto_runs"])
        * 100
        for name in names
    ]

    x2 = np.arange(len(names))
    w2 = 0.35
    ax2.bar(
        x2 - w2 / 2,
        improvement_det,
        w2,
        label="Mejora TS/Det.",
        color=PALETTE[0],
        alpha=0.85,
    )
    ax2.bar(
        x2 + w2 / 2,
        improvement_sto,
        w2,
        label="Mejora TS/Sto.",
        color=PALETTE[1],
        alpha=0.85,
    )
    ax2.axhline(0, color="gray", linewidth=1)
    ax2.set_xticks(x2)
    ax2.set_xticklabels(names)
    ax2.set_ylabel("Mejora (%)")
    ax2.set_xlabel("Instancia")
    ax2.set_title("Mejora de TS sobre su partida Greedy")
    ax2.legend()
    ax2.grid(True, alpha=0.3, axis="y")
    for i, (value_det, value_sto) in enumerate(zip(improvement_det, improvement_sto)):
        ax2.text(
            i - w2 / 2,
            value_det + 0.05,
            f"{value_det:+.2f}%",
            ha="center",
            va="bottom",
            fontsize=8,
        )
        ax2.text(
            i + w2 / 2,
            value_sto + 0.05,
            f"{value_sto:+.2f}%",
            ha="center",
            va="bottom",
            fontsize=8,
        )

    plt.tight_layout()
    fig.savefig(output_dir / "calidad_solucion.png", dpi=150, bbox_inches="tight")
    plt.close(fig)
    print("  calidad_solucion.png generada")


def plot_best_fo_vs_iterations(instances, output_dir, global_tenure):
    tenure_label = global_tenure if global_tenure is not None else "N/A"
    fig, axes = plt.subplots(2, 2, figsize=(14, 9))
    fig.suptitle(
        f"Evolucion Best FO vs Iteraciones - Tabu Search (tenure={tenure_label})",
        fontsize=14,
        fontweight="bold",
    )

    for idx, name in enumerate(INSTANCE_ORDER):
        ax = axes[idx // 2][idx % 2]
        data = instances[name]
        best_run = max(data["sto_runs"], key=lambda run: run["ts_result"])
        best_hist = best_run.get("best_hist", [])

        if best_hist:
            iterations = list(range(len(best_hist)))
            ax.plot(
                iterations,
                best_hist,
                color=PALETTE[idx],
                linewidth=2,
                label=f"Best FO (run {best_run['run']})",
            )
            ax.fill_between(
                iterations, best_hist[0], best_hist, alpha=0.15, color=PALETTE[idx]
            )

        for run in data["sto_runs"]:
            history = run.get("best_hist", [])
            if history:
                ax.plot(
                    range(len(history)), history, color="gray", linewidth=0.8, alpha=0.3
                )

        if data["best_hist_det"]:
            ax.plot(
                range(len(data["best_hist_det"])),
                data["best_hist_det"],
                color="navy",
                linewidth=1.5,
                linestyle="-.",
                label="Best FO Det.",
            )

        ax.set_title(
            instance_title(name, data.get("tenure")), fontsize=11, fontweight="bold"
        )
        ax.set_xlabel("Iteracion")
        ax.set_ylabel("Best FO (beneficio acumulado)")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.yaxis.set_major_formatter(mticker.FuncFormatter(lambda v, _: f"{int(v):,}"))

    plt.tight_layout()
    fig.savefig(output_dir / "best_fo_vs_iteraciones.png", dpi=150, bbox_inches="tight")
    plt.close(fig)
    print("  best_fo_vs_iteraciones.png generada")


def plot_current_fo_vs_iterations(instances, output_dir, global_tenure):
    tenure_label = global_tenure if global_tenure is not None else "N/A"
    fig, axes = plt.subplots(2, 2, figsize=(14, 9))
    fig.suptitle(
        f"Evolucion Current FO vs Iteraciones - Tabu Search (tenure={tenure_label})",
        fontsize=14,
        fontweight="bold",
    )

    for idx, name in enumerate(INSTANCE_ORDER):
        ax = axes[idx // 2][idx % 2]
        data = instances[name]
        best_run = max(data["sto_runs"], key=lambda run: run["ts_result"])
        curr_hist = best_run.get("curr_hist", [])
        best_hist = best_run.get("best_hist", [])

        if curr_hist:
            iterations = list(range(len(curr_hist)))
            ax.plot(
                iterations,
                curr_hist,
                color=PALETTE[idx],
                linewidth=1.5,
                alpha=0.85,
                label=f"Current FO (run {best_run['run']})",
            )
        if best_hist:
            ax.plot(
                range(len(best_hist)),
                best_hist,
                color="black",
                linewidth=2,
                linestyle="--",
                label="Best FO (referencia)",
            )

        ax.set_title(
            instance_title(name, data.get("tenure")), fontsize=11, fontweight="bold"
        )
        ax.set_xlabel("Iteracion")
        ax.set_ylabel("FO solucion actual")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.yaxis.set_major_formatter(mticker.FuncFormatter(lambda v, _: f"{int(v):,}"))

    plt.tight_layout()
    fig.savefig(
        output_dir / "current_fo_vs_iteraciones.png", dpi=150, bbox_inches="tight"
    )
    plt.close(fig)
    print("  current_fo_vs_iteraciones.png generada")


def plot_fo_vs_time(instances, output_dir, global_tenure):
    tenure_label = global_tenure if global_tenure is not None else "N/A"
    fig, axes = plt.subplots(2, 2, figsize=(14, 9))
    fig.suptitle(
        f"Best FO vs Tiempo de Ejecucion - Tabu Search (tenure={tenure_label})",
        fontsize=14,
        fontweight="bold",
    )

    for idx, name in enumerate(INSTANCE_ORDER):
        ax = axes[idx // 2][idx % 2]
        data = instances[name]
        best_run = max(data["sto_runs"], key=lambda run: run["ts_result"])
        best_hist = best_run.get("best_hist", [])
        iter_times = best_run.get("iter_times", [])

        if best_hist and iter_times and len(best_hist) == len(iter_times):
            ax.plot(
                iter_times,
                best_hist,
                color=PALETTE[idx],
                linewidth=2,
                label=f"Best FO (run {best_run['run']})",
            )
            ax.fill_between(
                iter_times, best_hist[0], best_hist, alpha=0.12, color=PALETTE[idx]
            )

        for run in data["sto_runs"]:
            history = run.get("best_hist", [])
            times = run.get("iter_times", [])
            if history and times and len(history) == len(times):
                ax.plot(times, history, color="gray", linewidth=0.7, alpha=0.3)

        if data["best_hist_det"] and data["iter_times_det"]:
            history = data["best_hist_det"]
            times = data["iter_times_det"]
            if len(history) == len(times):
                ax.plot(
                    times,
                    history,
                    color="navy",
                    linewidth=1.5,
                    linestyle="-.",
                    label="Best FO Det.",
                )

        ax.set_title(
            instance_title(name, data.get("tenure")), fontsize=11, fontweight="bold"
        )
        ax.set_xlabel("Tiempo (s)")
        ax.set_ylabel("Best FO (beneficio)")
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.yaxis.set_major_formatter(mticker.FuncFormatter(lambda v, _: f"{int(v):,}"))

    plt.tight_layout()
    fig.savefig(output_dir / "fo_vs_tiempo.png", dpi=150, bbox_inches="tight")
    plt.close(fig)
    print("  fo_vs_tiempo.png generada")


def main():
    if not RESULTS_PATH.exists():
        print(f"Error: no se encontro el archivo {RESULTS_PATH}")
        return

    print(f"Leyendo resultados de: {RESULTS_PATH}")
    instances, global_tenure = parse_results(RESULTS_PATH)

    if not instances:
        print("Error: no se pudieron parsear instancias del archivo.")
        return

    if global_tenure is not None:
        print(f"Tenure global detectado: {global_tenure}")

    print(f"Instancias encontradas: {list(instances.keys())}")
    print(f"Generando graficas en: {OUTPUT_DIR}\n")

    plot_convergence(instances, OUTPUT_DIR, global_tenure)
    plot_execution_times(instances, OUTPUT_DIR, global_tenure)
    plot_solution_quality(instances, OUTPUT_DIR, global_tenure)
    plot_best_fo_vs_iterations(instances, OUTPUT_DIR, global_tenure)
    plot_current_fo_vs_iterations(instances, OUTPUT_DIR, global_tenure)
    plot_fo_vs_time(instances, OUTPUT_DIR, global_tenure)

    print("\nTodas las graficas generadas correctamente.")


if __name__ == "__main__":
    main()
