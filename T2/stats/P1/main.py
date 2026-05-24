import re
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as mticker
from pathlib import Path

RESULTS_PATH = Path(__file__).parent.parent.parent / "P1" / "results.txt"
OUTPUT_DIR = Path(__file__).parent

INSTANCE_ORDER = ["easy", "medium1", "medium2", "hard"]
PALETTE = ["#4C72B0", "#DD8452", "#55A868", "#C44E52"]


def parse_results(path: Path) -> dict:
    content = path.read_text(encoding="utf-8", errors="replace")
    instances = {}

    for name, block in re.findall(
        r"Instance:\s*(\w+)(.*?)(?=Instance:|\Z)", content, re.DOTALL
    ):
        name = name.lower()

        det_benefit = int(
            re.search(
                r"\[Greedy Determinista\].*?Beneficio\s*:\s*(\d+)", block, re.DOTALL
            ).group(1)
        )
        det_time = float(
            re.search(
                r"\[Greedy Determinista\].*?Tiempo\s*:\s*([\d.]+)s", block, re.DOTALL
            ).group(1)
        )

        sto_runs = [
            {"benefit": int(m.group(1)), "time": float(m.group(2))}
            for m in re.finditer(
                r"Run\s+\d+:\s*beneficio=\s*(\d+)\s+costo=\s*\d+\s+t=([\d.]+)s", block
            )
        ]

        stats_match = re.search(
            r"Estad[ií]sticas.*?Media\s*:\s*([\d.]+).*?Desv\. Est\.\s*:\s*([\d.]+).*?Mejor\s*:\s*(\d+).*?Peor\s*:\s*(\d+).*?Mediana\s*:\s*([\d.]+)",
            block,
            re.DOTALL,
        )

        instances[name] = {
            "det_benefit": det_benefit,
            "det_time": det_time,
            "sto_runs": sto_runs,
            "sto_mean": float(stats_match.group(1)),
            "sto_std": float(stats_match.group(2)),
            "sto_best": int(stats_match.group(3)),
            "sto_worst": int(stats_match.group(4)),
            "sto_median": float(stats_match.group(5)),
        }

    return instances


def plot_convergence(instances: dict, output_dir: Path):
    fig, axes = plt.subplots(2, 2, figsize=(14, 9))
    fig.suptitle(
        "Evolución del Beneficio — Greedy Estocástico (10 runs)",
        fontsize=14,
        fontweight="bold",
    )

    for idx, name in enumerate(INSTANCE_ORDER):
        ax = axes[idx // 2][idx % 2]
        data = instances[name]
        run_nums = list(range(1, len(data["sto_runs"]) + 1))
        benefits = [r["benefit"] for r in data["sto_runs"]]

        ax.plot(
            run_nums,
            benefits,
            marker="o",
            color=PALETTE[idx],
            linewidth=2,
            markersize=6,
            label="Estocástico",
        )
        ax.axhline(
            data["det_benefit"],
            color="gray",
            linestyle="--",
            linewidth=1.5,
            label=f"Determinista ({data['det_benefit']:,})",
        )
        ax.axhline(
            data["sto_mean"],
            color="red",
            linestyle=":",
            linewidth=1.5,
            label=f"Media ({data['sto_mean']:,.1f})",
        )

        ax.fill_between(
            run_nums,
            data["sto_mean"] - data["sto_std"],
            data["sto_mean"] + data["sto_std"],
            color="red",
            alpha=0.08,
        )

        ax.set_title(f"Instancia: {name}", fontsize=11, fontweight="bold")
        ax.set_xlabel("Run")
        ax.set_ylabel("Beneficio")
        ax.set_xticks(run_nums)
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.yaxis.set_major_formatter(mticker.FuncFormatter(lambda v, _: f"{int(v):,}"))

    plt.tight_layout()
    fig.savefig(output_dir / "convergencia_evolucion.png", dpi=150, bbox_inches="tight")
    plt.close(fig)


def plot_execution_times(instances: dict, output_dir: Path):
    fig, axes = plt.subplots(1, 2, figsize=(14, 5))
    fig.suptitle("Tiempos de Ejecución — Greedy", fontsize=14, fontweight="bold")

    names = INSTANCE_ORDER
    det_times = [instances[n]["det_time"] for n in names]
    sto_mean_times = [
        np.mean([r["time"] for r in instances[n]["sto_runs"]]) for n in names
    ]

    x = np.arange(len(names))
    width = 0.35

    ax = axes[0]
    ax.bar(
        x - width / 2,
        det_times,
        width,
        label="Determinista",
        color=PALETTE[0],
        alpha=0.85,
    )
    ax.bar(
        x + width / 2,
        sto_mean_times,
        width,
        label="Estocástico (media)",
        color=PALETTE[1],
        alpha=0.85,
    )
    ax.set_title("Tiempo promedio por instancia")
    ax.set_xticks(x)
    ax.set_xticklabels(names)
    ax.set_ylabel("Tiempo (s)")
    ax.legend()
    ax.grid(True, alpha=0.3, axis="y")

    ax2 = axes[1]
    for idx, name in enumerate(names):
        run_times = [r["time"] for r in instances[name]["sto_runs"]]
        bp = ax2.boxplot(
            run_times,
            positions=[idx + 1],
            widths=0.5,
            patch_artist=True,
            boxprops=dict(facecolor=PALETTE[idx], alpha=0.7),
            medianprops=dict(color="black", linewidth=2),
            whiskerprops=dict(linewidth=1.2),
            capprops=dict(linewidth=1.2),
        )
    ax2.set_title("Distribución tiempos estocásticos")
    ax2.set_xticks(range(1, len(names) + 1))
    ax2.set_xticklabels(names)
    ax2.set_ylabel("Tiempo (s)")
    ax2.grid(True, alpha=0.3, axis="y")

    plt.tight_layout()
    fig.savefig(output_dir / "tiempos_ejecucion.png", dpi=150, bbox_inches="tight")
    plt.close(fig)


def plot_solution_quality(instances: dict, output_dir: Path):
    fig, axes = plt.subplots(1, 2, figsize=(14, 6))
    fig.suptitle("Calidad de la Solución — Greedy", fontsize=14, fontweight="bold")

    names = INSTANCE_ORDER
    det_vals = [instances[n]["det_benefit"] for n in names]
    sto_best_vals = [instances[n]["sto_best"] for n in names]
    sto_mean_vals = [instances[n]["sto_mean"] for n in names]
    sto_worst_vals = [instances[n]["sto_worst"] for n in names]

    x = np.arange(len(names))
    width = 0.2

    ax = axes[0]
    ax.bar(
        x - 1.5 * width,
        det_vals,
        width,
        label="Determinista",
        color=PALETTE[0],
        alpha=0.85,
    )
    ax.bar(
        x - 0.5 * width,
        sto_best_vals,
        width,
        label="Sto. Mejor",
        color=PALETTE[1],
        alpha=0.85,
    )
    ax.bar(
        x + 0.5 * width,
        sto_mean_vals,
        width,
        label="Sto. Media",
        color=PALETTE[2],
        alpha=0.85,
    )
    ax.bar(
        x + 1.5 * width,
        sto_worst_vals,
        width,
        label="Sto. Peor",
        color=PALETTE[3],
        alpha=0.85,
    )
    ax.set_xticks(x)
    ax.set_xticklabels(names)
    ax.set_ylabel("Beneficio")
    ax.set_xlabel("Instancia")
    ax.set_title("Comparativa de beneficio")
    ax.legend()
    ax.grid(True, alpha=0.3, axis="y")
    ax.yaxis.set_major_formatter(mticker.FuncFormatter(lambda v, _: f"{int(v):,}"))

    ax2 = axes[1]
    gaps = [
        (instances[n]["sto_best"] - instances[n]["det_benefit"])
        / instances[n]["det_benefit"]
        * 100
        for n in names
    ]
    bar_colors = [PALETTE[1] if g >= 0 else PALETTE[3] for g in gaps]
    bars = ax2.bar(names, gaps, color=bar_colors, alpha=0.85)
    ax2.axhline(0, color="gray", linewidth=1)
    ax2.set_ylabel("Gap vs Determinista (%)")
    ax2.set_xlabel("Instancia")
    ax2.set_title("Gap: Mejor Estocástico vs Determinista")
    ax2.grid(True, alpha=0.3, axis="y")
    for bar, val in zip(bars, gaps):
        ax2.text(
            bar.get_x() + bar.get_width() / 2,
            bar.get_height() + 0.03,
            f"{val:+.2f}%",
            ha="center",
            va="bottom",
            fontsize=9,
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
