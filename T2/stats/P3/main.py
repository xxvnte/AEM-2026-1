import re
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as mticker
from pathlib import Path

# Rutas adaptadas para la P3
RESULTS_PATH = Path(__file__).parent.parent.parent / "P3" / "results.txt"
OUTPUT_DIR = Path(__file__).parent

INSTANCE_ORDER = ["easy", "medium1", "medium2", "hard"]
PALETTE = ["#4C72B0", "#DD8452", "#55A868", "#C44E52"]


def parse_results(path: Path) -> dict:
    content = path.read_text(encoding="utf-8", errors="replace")
    instances = {}

    # Separar bloques por instancia de ACO
    for name, block in re.findall(
        r"ACO - Instancia:\s*(\w+)(.*?)(?=ACO - Instancia|RESUMEN|\Z)",
        content,
        re.DOTALL,
    ):
        name = name.lower()

        # Parsear Greedy Determinista
        det_section = re.search(
            r"\[Greedy Determinista\](.*?)(?=\[ACO)",
            block,
            re.DOTALL,
        )
        det_text = det_section.group(1)
        det_resultado = int(re.search(r"Beneficio\s*:\s*(\d+)", det_text).group(1))
        det_time = float(re.search(r"Tiempo\s*:\s*([\d.]+)s", det_text).group(1))

        # Parsear corridas de ACO
        aco_section = re.search(
            r"\[ACO x\d+ corridas.*?\](.*?)(?=Estad[ií]sticas)",
            block,
            re.DOTALL,
        )
        aco_runs = [
            {
                "run": int(m.group(1)),
                "benefit": int(m.group(2)),
                "time": float(m.group(3)),
            }
            for m in re.finditer(
                r"Run\s+(\d+):\s*beneficio=\s*(\d+)\s+costo=\s*\d+\s+t=([\d.]+)s",
                aco_section.group(1),
            )
        ]

        # Parsear Estadísticas
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


def plot_runs_variance(instances: dict, output_dir: Path):
    """
    Gráfica adaptada para ACO: Muestra la dispersión de las 10 corridas
    independientes vs la línea base determinista.
    """
    fig, axes = plt.subplots(2, 2, figsize=(14, 9))
    fig.suptitle(
        "Estabilidad y Dispersión de Beneficio — ACO (10 corridas)",
        fontsize=14,
        fontweight="bold",
    )

    for idx, name in enumerate(INSTANCE_ORDER):
        if name not in instances:
            continue
            
        ax = axes[idx // 2][idx % 2]
        data = instances[name]
        run_nums = list(range(1, len(data["aco_runs"]) + 1))
        aco_results = [r["benefit"] for r in data["aco_runs"]]

        # Graficar los puntos de las 10 corridas
        ax.plot(
            run_nums,
            aco_results,
            marker="o",
            color=PALETTE[idx],
            linewidth=2,
            markersize=7,
            label="Resultado ACO (run)",
        )

        # Línea de referencia Determinista
        ax.axhline(
            data["det_resultado"],
            color="navy",
            linestyle="-.",
            linewidth=1.5,
            label=f"Greedy Det. ({data['det_resultado']:,})",
        )
        
        # Línea de referencia Media ACO
        ax.axhline(
            data["aco_mean"],
            color="red",
            linestyle=":",
            linewidth=1.5,
            label=f"Media ACO ({data['aco_mean']:,.1f})",
        )

        ax.set_title(f"Instancia: {name}", fontsize=11, fontweight="bold")
        ax.set_xlabel("Run")
        ax.set_ylabel("Beneficio")
        ax.set_xticks(run_nums)
        ax.legend(fontsize=8)
        ax.grid(True, alpha=0.3)
        ax.yaxis.set_major_formatter(mticker.FuncFormatter(lambda v, _: f"{int(v):,}"))

    plt.tight_layout()
    fig.savefig(output_dir / "dispersion_corridas.png", dpi=150, bbox_inches="tight")
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
    ax.bar(
        x - width / 2, det_times, width, label="Greedy Det.", color=PALETTE[0], alpha=0.85
    )
    ax.bar(
        x + width / 2,
        aco_mean_times,
        width,
        label="ACO (media)",
        color=PALETTE[1],
        alpha=0.85,
    )
    ax.errorbar(
        x + width / 2,
        aco_mean_times,
        yerr=[
            [m - mn for m, mn in zip(aco_mean_times, aco_min_times)],
            [mx - m for mx, m in zip(aco_max_times, aco_mean_times)],
        ],
        fmt="none",
        color="black",
        capsize=4,
        linewidth=1.2,
    )
    ax.set_title("Tiempo de ejecución promedio por instancia")
    ax.set_xticks(x)
    ax.set_xticklabels(names)
    ax.set_ylabel("Tiempo (s)")
    ax.legend()
    ax.grid(True, alpha=0.3, axis="y")

    # Escala logarítmica si la diferencia entre Greedy Det y ACO es gigante
    # ax.set_yscale("log") # Descomentar si el Greedy Det desaparece visualmente

    ax2 = axes[1]
    for idx, name in enumerate(names):
        run_times = [r["time"] for r in instances[name]["aco_runs"]]
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
    fig.suptitle(
        "Calidad de la Solución — Ant Colony Optimization vs Greedy", fontsize=14, fontweight="bold"
    )

    names = [n for n in INSTANCE_ORDER if n in instances]
    det_resultado = [instances[n]["det_resultado"] for n in names]
    aco_best = [instances[n]["aco_best"] for n in names]
    aco_mean = [instances[n]["aco_mean"] for n in names]

    x = np.arange(len(names))
    width = 0.25

    ax = axes[0]
    ax.bar(
        x - width, det_resultado, width, label="Greedy Det.", color=PALETTE[0], alpha=0.85
    )
    ax.bar(
        x, aco_mean, width, label="ACO Media", color=PALETTE[2], alpha=0.85
    )
    ax.bar(
        x + width, aco_best, width, label="ACO Mejor", color=PALETTE[1], alpha=1.0
    )
    
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
    ax2.bar(
        x2 - w2 / 2,
        improvement_mean,
        w2,
        label="Mejora Media ACO",
        color=PALETTE[2],
        alpha=0.85,
    )
    ax2.bar(
        x2 + w2 / 2,
        improvement_best,
        w2,
        label="Mejora Mejor ACO",
        color=PALETTE[1],
        alpha=0.85,
    )
    
    ax2.axhline(0, color="gray", linewidth=1)
    ax2.set_xticks(x2)
    ax2.set_xticklabels(names)
    ax2.set_ylabel("Mejora (%)")
    ax2.set_xlabel("Instancia")
    ax2.set_title("Mejora porcentual de ACO sobre Greedy Determinista")
    ax2.legend()
    ax2.grid(True, alpha=0.3, axis="y")
    
    for i, (vmean, vbest) in enumerate(zip(improvement_mean, improvement_best)):
        ax2.text(
            i - w2 / 2, vmean + 0.5, f"{vmean:+.1f}%", ha="center", va="bottom", fontsize=8
        )
        ax2.text(
            i + w2 / 2, vbest + 0.5, f"{vbest:+.1f}%", ha="center", va="bottom", fontsize=8
        )

    plt.tight_layout()
    fig.savefig(output_dir / "calidad_solucion.png", dpi=150, bbox_inches="tight")
    plt.close(fig)


def main():
    if not RESULTS_PATH.exists():
        print(f"Error: no se encontró el archivo {RESULTS_PATH}")
        return

    instances = parse_results(RESULTS_PATH)
    if not instances:
        print("Error: No se lograron parsear instancias. Revisa el formato de results.txt")
        return

    # Crear la carpeta contenedora si no existe
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    plot_runs_variance(instances, OUTPUT_DIR)
    plot_execution_times(instances, OUTPUT_DIR)
    plot_solution_quality(instances, OUTPUT_DIR)
    print(f"Gráficas generadas exitosamente en: {OUTPUT_DIR}")


if __name__ == "__main__":
    main()