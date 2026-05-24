import re
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as mticker
from pathlib import Path

RESULTS_PATH = Path(__file__).parent.parent.parent / "P2" / "results.txt"
OUTPUT_DIR = Path(__file__).parent

INSTANCE_ORDER = ["easy", "medium1", "medium2", "hard"]
PALETTE = ["#4C72B0", "#DD8452", "#55A868", "#C44E52"]


def parse_results(path: Path) -> dict:
    content = path.read_text(encoding="utf-8", errors="replace")
    instances = {}

    for name, block in re.findall(
        r"TABU SEARCH.*?Instancia:\s*(\w+)(.*?)(?=TABU SEARCH|RESUMEN|\Z)",
        content,
        re.DOTALL,
    ):
        name = name.lower()

        det_section = re.search(
            r"\[TS desde Greedy Determinista\](.*?)(?=\[TS desde Greedy)",
            block,
            re.DOTALL,
        )
        det_text = det_section.group(1)
        det_inicio = int(re.search(r"Inicio\s*:.*?beneficio=(\d+)", det_text).group(1))
        det_resultado = int(
            re.search(r"Resultado:.*?beneficio=(\d+)", det_text).group(1)
        )
        det_time = float(re.search(r"Tiempo\s*:\s*([\d.]+)s", det_text).group(1))

        sto_section = re.search(
            r"\[TS desde Greedy Estoc[aá]stico.*?\](.*?)(?=Estad[ií]sticas)",
            block,
            re.DOTALL,
        )
        sto_runs = [
            {
                "partida": int(m.group(1)),
                "ts_result": int(m.group(2)),
                "time": float(m.group(3)),
            }
            for m in re.finditer(
                r"Run\s+\d+:\s*partida=\s*(\d+)\s+TS=\s*(\d+)\s+costo=\s*\d+\s+t=([\d.]+)s",
                sto_section.group(1),
            )
        ]

        stats_section = re.search(
            r"Estad[ií]sticas TS.*?Media\s*:\s*([\d.]+).*?Desv\. Est\.\s*:\s*([\d.]+).*?Mejor\s*:\s*(\d+).*?Peor\s*:\s*(\d+).*?Mediana\s*:\s*([\d.]+)",
            block,
            re.DOTALL,
        )

        best_global = int(re.search(r"Mejor global.*?:\s*(\d+)", block).group(1))

        instances[name] = {
            "det_inicio": det_inicio,
            "det_resultado": det_resultado,
            "det_time": det_time,
            "sto_runs": sto_runs,
            "sto_mean": float(stats_section.group(1)),
            "sto_std": float(stats_section.group(2)),
            "sto_best": int(stats_section.group(3)),
            "sto_worst": int(stats_section.group(4)),
            "sto_median": float(stats_section.group(5)),
            "best_global": best_global,
        }

    return instances


def plot_convergence(instances: dict, output_dir: Path):
    fig, axes = plt.subplots(2, 2, figsize=(14, 9))
    fig.suptitle(
        "Evolución del Beneficio — Tabu Search (partida → resultado)",
        fontsize=14,
        fontweight="bold",
    )

    for idx, name in enumerate(INSTANCE_ORDER):
        ax = axes[idx // 2][idx % 2]
        data = instances[name]
        run_nums = list(range(1, len(data["sto_runs"]) + 1))
        partidas = [r["partida"] for r in data["sto_runs"]]
        ts_results = [r["ts_result"] for r in data["sto_runs"]]

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

        for i, (p, r) in enumerate(zip(partidas, ts_results)):
            ax.annotate(
                "",
                xy=(i + 1, r),
                xytext=(i + 1, p),
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
    fig.suptitle("Tiempos de Ejecución — Tabu Search", fontsize=14, fontweight="bold")

    names = INSTANCE_ORDER
    det_times = [instances[n]["det_time"] for n in names]
    sto_mean_times = [
        np.mean([r["time"] for r in instances[n]["sto_runs"]]) for n in names
    ]
    sto_max_times = [
        np.max([r["time"] for r in instances[n]["sto_runs"]]) for n in names
    ]
    sto_min_times = [
        np.min([r["time"] for r in instances[n]["sto_runs"]]) for n in names
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
            [m - mn for m, mn in zip(sto_mean_times, sto_min_times)],
            [mx - m for mx, m in zip(sto_max_times, sto_mean_times)],
        ],
        fmt="none",
        color="black",
        capsize=4,
        linewidth=1.2,
    )
    ax.set_title("Tiempo de ejecución por instancia")
    ax.set_xticks(x)
    ax.set_xticklabels(names)
    ax.set_ylabel("Tiempo (s)")
    ax.legend()
    ax.grid(True, alpha=0.3, axis="y")

    ax2 = axes[1]
    for idx, name in enumerate(names):
        run_times = [r["time"] for r in instances[name]["sto_runs"]]
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
    ax2.set_title("Distribución tiempos TS estocástico")
    ax2.set_xticks(range(1, len(names) + 1))
    ax2.set_xticklabels(names)
    ax2.set_ylabel("Tiempo (s)")
    ax2.grid(True, alpha=0.3, axis="y")

    plt.tight_layout()
    fig.savefig(output_dir / "tiempos_ejecucion.png", dpi=150, bbox_inches="tight")
    plt.close(fig)


def plot_solution_quality(instances: dict, output_dir: Path):
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    fig.suptitle(
        "Calidad de la Solución — Tabu Search vs Greedy", fontsize=14, fontweight="bold"
    )

    names = INSTANCE_ORDER
    det_ini = [instances[n]["det_inicio"] for n in names]
    ts_det = [instances[n]["det_resultado"] for n in names]
    sto_best_partida = [
        max(r["partida"] for r in instances[n]["sto_runs"]) for n in names
    ]
    ts_sto_best = [instances[n]["sto_best"] for n in names]
    ts_sto_mean = [instances[n]["sto_mean"] for n in names]

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
        (instances[n]["det_resultado"] - instances[n]["det_inicio"])
        / instances[n]["det_inicio"]
        * 100
        for n in names
    ]
    improvement_sto = [
        (instances[n]["sto_best"] - max(r["partida"] for r in instances[n]["sto_runs"]))
        / max(r["partida"] for r in instances[n]["sto_runs"])
        * 100
        for n in names
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
    for i, (vd, vs) in enumerate(zip(improvement_det, improvement_sto)):
        ax2.text(
            i - w2 / 2, vd + 0.05, f"{vd:+.2f}%", ha="center", va="bottom", fontsize=8
        )
        ax2.text(
            i + w2 / 2, vs + 0.05, f"{vs:+.2f}%", ha="center", va="bottom", fontsize=8
        )

    plt.tight_layout()
    fig.savefig(output_dir / "calidad_solucion.png", dpi=150, bbox_inches="tight")
    plt.close(fig)


def main():
    if not RESULTS_PATH.exists():
        print(f"Error: no se encontró el archivo {RESULTS_PATH}")
        return

    instances = parse_results(RESULTS_PATH)
    plot_convergence(instances, OUTPUT_DIR)
    plot_execution_times(instances, OUTPUT_DIR)
    plot_solution_quality(instances, OUTPUT_DIR)
    print(f"Gráficas generadas en: {OUTPUT_DIR}")


if __name__ == "__main__":
    main()
