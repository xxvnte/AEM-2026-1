from __future__ import annotations

import math
from pathlib import Path

try:
    import matplotlib.pyplot as plt
    import matplotlib.ticker as ticker
except ImportError:
    raise SystemExit("matplotlib no está instalado.  Ejecuta:  pip install matplotlib")

PROJECT_DIR = Path(__file__).parent
PAPER_DIR = PROJECT_DIR / "stats_paper"
SMALL_DIR = PAPER_DIR / "small"
LARGE_DIR = PAPER_DIR / "large"
EXTRAS_DIR = PAPER_DIR / "extras"

for _d in (SMALL_DIR, LARGE_DIR, EXTRAS_DIR):
    _d.mkdir(parents=True, exist_ok=True)

# Tabla 2 - Instancias pequeñas (C12R2 - C24R2)
SMALL_DATA = [
    # inst       N  CPLEX_E   CPLEX_t  ALNS_E    ALNS_t  ALNS_g  AC_E     AC_t  AC_g
    ("C12R2", 12, 95.48, 4.02, 101.00, 0.0, 5.78, 98.33, 0.0, 2.99),
    ("C13R2", 13, 103.26, 6.32, 134.89, 0.0, 30.63, 104.83, 0.0, 1.52),
    ("C14R2", 14, 108.96, 4.31, 122.20, 0.0, 12.15, 110.53, 0.0, 1.44),
    ("C15R2", 15, 111.64, 2.15, 114.92, 0.0, 2.94, 112.47, 0.0, 0.74),
    ("C16R2", 16, 120.62, 3.91, 140.48, 1.0, 16.47, 123.44, 1.0, 2.34),
    ("C17R2", 17, 121.73, 13.71, 141.59, 1.0, 16.31, 121.73, 1.0, 0.00),
    ("C18R2", 18, 125.66, 11.94, 126.43, 1.0, 0.61, 129.00, 1.0, 2.66),
    ("C19R2", 19, 133.76, 36.22, 141.68, 1.0, 5.92, 145.69, 1.0, 8.92),
    ("C20R2", 20, 132.25, 26.24, 146.13, 1.0, 10.49, 132.52, 1.0, 0.20),
    ("C21R2", 21, 137.69, 64.09, 153.08, 2.0, 11.17, 148.95, 1.0, 8.18),
    ("C22R2", 22, 144.64, 1923.73, 157.26, 2.0, 8.73, 154.16, 2.0, 6.59),
    ("C23R2", 23, 145.51, 4890.62, 153.20, 2.0, 5.29, 149.61, 2.0, 2.82),
    ("C24R2", 24, 147.30, 8014.90, 157.37, 2.0, 6.84, 151.95, 2.0, 3.16),
]

SMALL_AVG = dict(
    cplex_e=121.78,
    cplex_t=1000.28,
    alns_e=137.71,
    alns_t=10.33,
    alns_g=10.26,
    ac_e=129.48,
    ac_t=0.92,
    ac_g=3.20,
)

# Tabla 3 - Instancias grandes (ALNS vs AC)
LARGE_DATA = [
    # inst          ALNS_E    ALNS_t  ALNS_rpd  AC_E     AC_t  AC_rpd
    ("C25R2-1", 159.30, 2.0, 2.32, 155.70, 2.0, 0.00),
    ("C25R2-2", 216.41, 12.0, 0.00, 217.78, 12.0, 0.63),
    ("C25R4-1", 289.42, 36.0, 0.66, 287.52, 32.0, 0.00),
    ("C25R4-2", 354.92, 74.0, 1.44, 349.88, 63.0, 0.00),
    ("C25R6-1", 444.94, 255.0, 0.23, 443.94, 183.0, 0.00),
    ("C25R6-2", 173.55, 3.0, 4.00, 166.87, 2.0, 0.00),
    ("C25R8-1", 235.46, 14.0, 1.82, 231.26, 12.0, 0.00),
    ("C25R8-2", 293.17, 36.0, 0.00, 293.43, 32.0, 0.09),
    ("C50R2-1", 347.84, 74.0, 0.00, 348.74, 63.0, 0.26),
    ("C50R2-2", 449.56, 242.0, 0.10, 449.13, 178.0, 0.00),
    ("C50R4-1", 164.49, 3.0, 2.27, 160.84, 2.0, 0.00),
    ("C50R4-2", 215.44, 12.0, 4.36, 206.44, 11.0, 0.00),
    ("C50R6-1", 289.94, 34.0, 3.25, 280.80, 29.0, 0.00),
    ("C50R6-2", 347.63, 71.0, 1.49, 342.53, 62.0, 0.00),
    ("C50R8-1", 439.98, 231.0, 3.08, 426.85, 180.0, 0.00),
    ("C50R8-2", 145.37, 2.0, 0.00, 148.62, 2.0, 2.24),
    ("C75R2-1", 230.68, 13.0, 0.00, 230.68, 12.0, 0.00),
    ("C75R2-2", 270.00, 33.0, 0.00, 270.03, 29.0, 0.01),
    ("C75R4-1", 349.58, 73.0, 2.67, 340.50, 61.0, 0.00),
    ("C75R4-2", 435.42, 233.0, 0.91, 431.49, 172.0, 0.00),
    ("C75R6-1", 178.05, 3.0, 0.00, 178.05, 2.0, 0.00),
    ("C75R6-2", 212.32, 13.0, 1.11, 209.98, 12.0, 0.00),
    ("C75R8-1", 287.57, 35.0, 0.00, 288.65, 32.0, 0.38),
    ("C75R8-2", 346.98, 76.0, 1.46, 341.98, 64.0, 0.00),
    ("C100R2-1", 457.23, 234.0, 1.57, 450.17, 188.0, 0.00),
    ("C100R2-2", 171.26, 2.0, 6.82, 160.32, 2.0, 0.00),
    ("C100R4-1", 245.17, 12.0, 1.44, 241.69, 12.0, 0.00),
    ("C100R4-2", 275.57, 35.0, 1.62, 271.17, 30.0, 0.00),
    ("C100R6-1", 331.00, 72.0, 0.00, 336.35, 60.0, 1.62),
    ("C100R6-2", 445.73, 241.0, 0.79, 442.21, 178.0, 0.00),
    ("C100R8-1", 170.56, 3.0, 1.10, 168.70, 2.0, 0.00),
    ("C100R8-2", 238.32, 13.0, 0.00, 242.14, 12.0, 1.61),
    ("C150R2-1", 274.41, 36.0, 1.03, 271.61, 31.0, 0.00),
    ("C150R2-2", 339.74, 81.0, 0.10, 339.41, 61.0, 0.00),
    ("C150R4-1", 445.46, 236.0, 0.00, 449.65, 172.0, 0.94),
    ("C150R4-2", 194.38, 4.0, 5.87, 183.60, 2.0, 0.00),
    ("C150R6-1", 222.22, 14.0, 0.00, 226.05, 12.0, 1.72),
    ("C150R6-2", 305.67, 36.0, 0.00, 309.90, 30.0, 1.38),
    ("C150R8-1", 356.30, 78.0, 0.87, 353.23, 65.0, 0.00),
    ("C150R8-2", 440.92, 248.0, 0.61, 438.26, 181.0, 0.00),
]
LARGE_AVG = dict(
    alns_e=294.80, alns_t=73.13, alns_rpd=1.32, ac_e=292.15, ac_t=57.18, ac_rpd=0.27
)

# Tabla 4 - Recharge-stations  (resultados del AC)
RECHARGE_DATA = [
    # grupo  avg_energy  avg_visits
    ("R2", 292.36, 0.40),
    ("R4", 294.12, 0.70),
    ("R6", 288.90, 0.60),
    ("R8", 293.24, 0.90),
]

# Tabla 5 - Battery-reserve  (resultados del AC)
BATTERY_DATA = [
    # nivel  usable_kWh  avg_energy  avg_visits
    ("0%", 110, 292.22, 0.65),
    ("10%", 99, 292.61, 1.00),
    ("20%", 88, 292.15, 1.60),
]

# Tabla 6 - Energy vs Distance  (resultados del AC)
EVD_DATA = [
    # inst          E_min    E_dist   pct_inc
    ("C25R2-1", 155.70, 161.87, 3.97),
    ("C25R2-2", 217.78, 235.90, 8.32),
    ("C25R4-1", 287.52, 347.98, 21.03),
    ("C25R4-2", 349.88, 411.91, 17.73),
    ("C25R6-1", 443.94, 491.66, 10.75),
    ("C25R6-2", 166.87, 189.42, 13.51),
    ("C25R8-1", 231.26, 330.86, 43.07),
    ("C25R8-2", 293.43, 354.84, 20.93),
    ("C50R2-1", 348.74, 429.31, 23.10),
    ("C50R2-2", 449.13, 495.09, 10.23),
    ("C50R4-1", 160.84, 186.49, 15.95),
    ("C50R4-2", 206.44, 273.80, 32.63),
    ("C50R6-1", 280.80, 329.18, 17.23),
    ("C50R6-2", 342.53, 388.79, 13.50),
    ("C50R8-1", 426.85, 491.70, 15.19),
    ("C50R8-2", 148.62, 179.48, 20.77),
    ("C75R2-1", 230.68, 288.62, 25.12),
    ("C75R2-2", 270.03, 315.60, 16.88),
    ("C75R4-1", 340.50, 397.11, 16.62),
    ("C75R4-2", 431.49, 482.70, 11.87),
    ("C75R6-1", 178.05, 231.48, 30.01),
    ("C75R6-2", 209.98, 246.39, 17.34),
    ("C75R8-1", 288.65, 360.05, 24.73),
    ("C75R8-2", 341.98, 378.08, 10.56),
    ("C100R2-1", 450.17, 540.12, 19.98),
    ("C100R2-2", 160.32, 194.93, 21.59),
    ("C100R4-1", 241.69, 264.35, 9.38),
    ("C100R4-2", 271.17, 342.60, 26.34),
    ("C100R6-1", 336.35, 411.00, 22.19),
    ("C100R6-2", 442.21, 513.52, 16.12),
    ("C100R8-1", 168.70, 185.74, 10.10),
    ("C100R8-2", 242.14, 272.42, 12.50),
    ("C150R2-1", 271.61, 293.00, 7.88),
    ("C150R2-2", 339.41, 401.84, 18.39),
    ("C150R4-1", 449.65, 486.55, 8.21),
    ("C150R4-2", 183.60, 222.24, 21.05),
    ("C150R6-1", 226.05, 253.14, 11.98),
    ("C150R6-2", 309.90, 330.68, 6.71),
    ("C150R8-1", 353.23, 401.19, 13.58),
    ("C150R8-2", 438.26, 496.29, 13.24),
]
EVD_AVG = dict(e_min=292.15, e_dist=340.20, pct=16.44)


_COLORS = {
    "cplex": "#8B2FC9",
    "alns": "#E07B39",
    "ac": "#3A86FF",
    "e_min": "#3A86FF",
    "e_dist": "#E07B39",
    "energy": "#3A86FF",
    "visits": "#E07B39",
    "pct": "#E07B39",
}

GROUP_SIZE = 10


def _savefig(fig: "plt.Figure", path: Path) -> None:
    fig.savefig(path, dpi=130, bbox_inches="tight")
    plt.close(fig)
    print(f"  {path.relative_to(PROJECT_DIR)}")


def _grouped(data: list, size: int = GROUP_SIZE) -> list[list]:
    return [data[i : i + size] for i in range(0, len(data), size)]


def _bar_positions(n_groups: int, n_series: int, width: float = 0.35):
    import numpy as np

    x = range(n_groups)
    offsets = [(i - (n_series - 1) / 2) * width for i in range(n_series)]
    return x, offsets, width


# SMALL


def _small_energy_bars() -> Path:
    """Energía CPLEX / ALNS / AC por instancia (Tabla 2)."""
    import numpy as np

    labels = [r[0] for r in SMALL_DATA]
    cplex = [r[2] for r in SMALL_DATA]
    alns = [r[4] for r in SMALL_DATA]
    ac = [r[7] for r in SMALL_DATA]

    x = np.arange(len(labels))
    w = 0.25
    fig, ax = plt.subplots(figsize=(13, 5))
    ax.bar(x - w, cplex, w, label="CPLEX (óptimo)", color=_COLORS["cplex"])
    ax.bar(x, alns, w, label="ALNS", color=_COLORS["alns"])
    ax.bar(x + w, ac, w, label="AC (paper)", color=_COLORS["ac"])
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=45, ha="right", fontsize=9)
    ax.set_ylabel("Energía (kWh)")
    ax.set_title("Instancias pequeñas - Energía CPLEX / ALNS / AC (Tabla 2)")
    ax.legend()
    ax.yaxis.set_minor_locator(ticker.AutoMinorLocator())
    ax.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    out = SMALL_DIR / "paper_small_energy_bars.png"
    _savefig(fig, out)
    return out


def _small_gap_bars() -> Path:
    """Absolute gap (%) ALNS vs AC respecto al óptimo CPLEX (Tabla 2)."""
    import numpy as np

    labels = [r[0] for r in SMALL_DATA]
    alns_g = [r[6] for r in SMALL_DATA]
    ac_g = [r[9] for r in SMALL_DATA]

    x = np.arange(len(labels))
    w = 0.35
    fig, ax = plt.subplots(figsize=(13, 5))
    ax.bar(
        x - w / 2,
        alns_g,
        w,
        label=f"ALNS  (avg {SMALL_AVG['alns_g']:.2f}%)",
        color=_COLORS["alns"],
    )
    ax.bar(
        x + w / 2,
        ac_g,
        w,
        label=f"AC    (avg {SMALL_AVG['ac_g']:.2f}%)",
        color=_COLORS["ac"],
    )
    ax.axhline(SMALL_AVG["alns_g"], color=_COLORS["alns"], ls="--", lw=1, alpha=0.6)
    ax.axhline(SMALL_AVG["ac_g"], color=_COLORS["ac"], ls="--", lw=1, alpha=0.6)
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=45, ha="right", fontsize=9)
    ax.set_ylabel("Gap absoluto (%)")
    ax.set_title("Instancias pequeñas - Gap absoluto vs CPLEX (Tabla 2)")
    ax.legend()
    ax.yaxis.set_minor_locator(ticker.AutoMinorLocator())
    ax.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    out = SMALL_DIR / "paper_small_gap_bars.png"
    _savefig(fig, out)
    return out


def _small_time_bars() -> Path:
    """Tiempo CPU CPLEX / ALNS / AC (Tabla 2).  CPLEX en eje secundario."""
    import numpy as np

    labels = [r[0] for r in SMALL_DATA]
    cplex_t = [r[3] for r in SMALL_DATA]
    alns_t = [r[5] for r in SMALL_DATA]
    ac_t = [r[8] for r in SMALL_DATA]

    x = np.arange(len(labels))
    w = 0.25
    fig, ax1 = plt.subplots(figsize=(13, 5))
    ax2 = ax1.twinx()

    ax1.bar(x - w, alns_t, w, label="ALNS", color=_COLORS["alns"])
    ax1.bar(x, ac_t, w, label="AC", color=_COLORS["ac"])
    ax2.bar(x + w, cplex_t, w, label="CPLEX", color=_COLORS["cplex"], alpha=0.6)

    ax1.set_xticks(x)
    ax1.set_xticklabels(labels, rotation=45, ha="right", fontsize=9)
    ax1.set_ylabel("Tiempo ALNS / AC (s)", color="black")
    ax2.set_ylabel("Tiempo CPLEX (s)", color=_COLORS["cplex"])
    ax2.tick_params(axis="y", labelcolor=_COLORS["cplex"])
    ax1.set_title("Instancias pequeñas - Tiempo CPU CPLEX / ALNS / AC (Tabla 2)")
    lines1, lab1 = ax1.get_legend_handles_labels()
    lines2, lab2 = ax2.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, lab1 + lab2)
    ax1.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    out = SMALL_DIR / "paper_small_time_bars.png"
    _savefig(fig, out)
    return out


def generate_small_charts() -> list[Path]:
    print("\n[small] Generando graficos en stats_paper/small/ ...")
    return [
        _small_energy_bars(),
        _small_gap_bars(),
        _small_time_bars(),
    ]


# LARGE


def _large_energy_bars_group(rows: list, g: int, total_groups: int) -> Path:
    """Energía ALNS vs AC para un grupo de instancias (Tabla 3)."""
    import numpy as np

    labels = [r[0] for r in rows]
    alns_e = [r[1] for r in rows]
    ac_e = [r[4] for r in rows]

    x = np.arange(len(labels))
    w = 0.35
    fig, ax = plt.subplots(figsize=(max(8, len(labels) * 1.1), 5))
    ax.bar(x - w / 2, alns_e, w, label="ALNS", color=_COLORS["alns"])
    ax.bar(x + w / 2, ac_e, w, label="AC", color=_COLORS["ac"])
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=45, ha="right", fontsize=9)
    ax.set_ylabel("Energía (kWh)")
    ax.set_title(
        f"Instancias grandes - Energía ALNS vs AC, grupo {g}/{total_groups} (Tabla 3)"
    )
    ax.legend()
    ax.yaxis.set_minor_locator(ticker.AutoMinorLocator())
    ax.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    out = LARGE_DIR / f"paper_large_energy_bars_g{g}.png"
    _savefig(fig, out)
    return out


def _large_rpd_bars_group(rows: list, g: int, total_groups: int) -> Path:
    """RPD (%) ALNS vs AC para un grupo de instancias (Tabla 3)."""
    import numpy as np

    labels = [r[0] for r in rows]
    alns_rpd = [r[3] for r in rows]
    ac_rpd = [r[6] for r in rows]

    x = np.arange(len(labels))
    w = 0.35
    fig, ax = plt.subplots(figsize=(max(8, len(labels) * 1.1), 5))
    ax.bar(
        x - w / 2,
        alns_rpd,
        w,
        label=f"ALNS  (avg {LARGE_AVG['alns_rpd']:.2f}%)",
        color=_COLORS["alns"],
    )
    ax.bar(
        x + w / 2,
        ac_rpd,
        w,
        label=f"AC    (avg {LARGE_AVG['ac_rpd']:.2f}%)",
        color=_COLORS["ac"],
    )
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=45, ha="right", fontsize=9)
    ax.set_ylabel("RPD (%)")
    ax.set_title(
        f"Instancias grandes - RPD ALNS vs AC, grupo {g}/{total_groups} (Tabla 3)"
    )
    ax.legend()
    ax.yaxis.set_minor_locator(ticker.AutoMinorLocator())
    ax.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    out = LARGE_DIR / f"paper_large_rpd_bars_g{g}.png"
    _savefig(fig, out)
    return out


def _large_time_bars_group(rows: list, g: int, total_groups: int) -> Path:
    """Tiempo CPU ALNS vs AC para un grupo de instancias (Tabla 3)."""
    import numpy as np

    labels = [r[0] for r in rows]
    alns_t = [r[2] for r in rows]
    ac_t = [r[5] for r in rows]

    x = np.arange(len(labels))
    w = 0.35
    fig, ax = plt.subplots(figsize=(max(8, len(labels) * 1.1), 5))
    ax.bar(
        x - w / 2,
        alns_t,
        w,
        label=f"ALNS  (avg {LARGE_AVG['alns_t']:.1f}s)",
        color=_COLORS["alns"],
    )
    ax.bar(
        x + w / 2,
        ac_t,
        w,
        label=f"AC    (avg {LARGE_AVG['ac_t']:.1f}s)",
        color=_COLORS["ac"],
    )
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=45, ha="right", fontsize=9)
    ax.set_ylabel("Tiempo (s)")
    ax.set_title(
        f"Instancias grandes - Tiempo CPU ALNS vs AC, grupo {g}/{total_groups} (Tabla 3)"
    )
    ax.legend()
    ax.yaxis.set_minor_locator(ticker.AutoMinorLocator())
    ax.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    out = LARGE_DIR / f"paper_large_time_bars_g{g}.png"
    _savefig(fig, out)
    return out


def _large_energy_bars_all() -> Path:
    """Energía ALNS vs AC — todas las instancias en un solo gráfico (Tabla 3)."""
    import numpy as np

    labels = [r[0] for r in LARGE_DATA]
    alns_e = [r[1] for r in LARGE_DATA]
    ac_e = [r[4] for r in LARGE_DATA]

    x = np.arange(len(labels))
    w = 0.35
    fig, ax = plt.subplots(figsize=(max(13, len(labels) * 0.45), 6))
    ax.bar(x - w / 2, alns_e, w, label="ALNS", color=_COLORS["alns"])
    ax.bar(x + w / 2, ac_e, w, label="AC", color=_COLORS["ac"])
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=90, ha="center", fontsize=7)
    ax.set_ylabel("Energía (kWh)")
    ax.set_title(
        "Instancias grandes - Energía ALNS vs AC, todas las instancias (Tabla 3)"
    )
    ax.legend()
    ax.yaxis.set_minor_locator(ticker.AutoMinorLocator())
    ax.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    out = LARGE_DIR / "paper_large_energy_bars.png"
    _savefig(fig, out)
    return out


def _large_rpd_bars_all() -> Path:
    """RPD (%) ALNS vs AC — todas las instancias en un solo gráfico (Tabla 3)."""
    import numpy as np

    labels = [r[0] for r in LARGE_DATA]
    alns_rpd = [r[3] for r in LARGE_DATA]
    ac_rpd = [r[6] for r in LARGE_DATA]

    x = np.arange(len(labels))
    w = 0.35
    fig, ax = plt.subplots(figsize=(max(13, len(labels) * 0.45), 6))
    ax.bar(
        x - w / 2,
        alns_rpd,
        w,
        label=f"ALNS  (avg {LARGE_AVG['alns_rpd']:.2f}%)",
        color=_COLORS["alns"],
    )
    ax.bar(
        x + w / 2,
        ac_rpd,
        w,
        label=f"AC    (avg {LARGE_AVG['ac_rpd']:.2f}%)",
        color=_COLORS["ac"],
    )
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=90, ha="center", fontsize=7)
    ax.set_ylabel("RPD (%)")
    ax.set_title("Instancias grandes - RPD ALNS vs AC, todas las instancias (Tabla 3)")
    ax.legend()
    ax.yaxis.set_minor_locator(ticker.AutoMinorLocator())
    ax.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    out = LARGE_DIR / "paper_large_rpd_bars.png"
    _savefig(fig, out)
    return out


def _large_time_bars_all() -> Path:
    """Tiempo CPU ALNS vs AC — todas las instancias en un solo gráfico (Tabla 3)."""
    import numpy as np

    labels = [r[0] for r in LARGE_DATA]
    alns_t = [r[2] for r in LARGE_DATA]
    ac_t = [r[5] for r in LARGE_DATA]

    x = np.arange(len(labels))
    w = 0.35
    fig, ax = plt.subplots(figsize=(max(13, len(labels) * 0.45), 6))
    ax.bar(
        x - w / 2,
        alns_t,
        w,
        label=f"ALNS  (avg {LARGE_AVG['alns_t']:.1f}s)",
        color=_COLORS["alns"],
    )
    ax.bar(
        x + w / 2,
        ac_t,
        w,
        label=f"AC    (avg {LARGE_AVG['ac_t']:.1f}s)",
        color=_COLORS["ac"],
    )
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=90, ha="center", fontsize=7)
    ax.set_ylabel("Tiempo (s)")
    ax.set_title(
        "Instancias grandes - Tiempo CPU ALNS vs AC, todas las instancias (Tabla 3)"
    )
    ax.legend()
    ax.yaxis.set_minor_locator(ticker.AutoMinorLocator())
    ax.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    out = LARGE_DIR / "paper_large_time_bars.png"
    _savefig(fig, out)
    return out


def _large_summary_bars() -> Path:
    """Resumen global: promedio energía, RPD y tiempo ALNS vs AC (Tabla 3)."""
    import numpy as np

    metrics = ["Energía (kWh)", "RPD (%)", "Tiempo (s)"]
    alns_vals = [LARGE_AVG["alns_e"], LARGE_AVG["alns_rpd"], LARGE_AVG["alns_t"]]
    ac_vals = [LARGE_AVG["ac_e"], LARGE_AVG["ac_rpd"], LARGE_AVG["ac_t"]]

    x = np.arange(len(metrics))
    w = 0.35
    fig, ax = plt.subplots(figsize=(7, 5))
    ax.bar(x - w / 2, alns_vals, w, label="ALNS", color=_COLORS["alns"])
    ax.bar(x + w / 2, ac_vals, w, label="AC", color=_COLORS["ac"])
    for xi, (a, b) in zip(x, zip(alns_vals, ac_vals)):
        ax.text(xi - w / 2, a + 0.3, f"{a:.2f}", ha="center", va="bottom", fontsize=8)
        ax.text(xi + w / 2, b + 0.3, f"{b:.2f}", ha="center", va="bottom", fontsize=8)
    ax.set_xticks(x)
    ax.set_xticklabels(metrics)
    ax.set_title("Instancias grandes - Resumen promedio ALNS vs AC (Tabla 3)")
    ax.legend()
    ax.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    out = LARGE_DIR / "paper_large_summary.png"
    _savefig(fig, out)
    return out


def generate_large_charts() -> list[Path]:
    print("\n[large] Generando graficos en stats_paper/large/ ...")
    saved: list[Path] = []
    groups = _grouped(LARGE_DATA, GROUP_SIZE)
    n = len(groups)
    for i, grp in enumerate(groups, start=1):
        saved.append(_large_energy_bars_group(grp, i, n))
        saved.append(_large_rpd_bars_group(grp, i, n))
        saved.append(_large_time_bars_group(grp, i, n))
    saved.append(_large_energy_bars_all())
    saved.append(_large_rpd_bars_all())
    saved.append(_large_time_bars_all())
    saved.append(_large_summary_bars())
    return saved


#  EXTRAS / RECHARGE-STATIONS


def _recharge_bars() -> list[Path]:
    """Tabla 4: energía media y visitas por grupo de estaciones."""
    groups = [r[0] for r in RECHARGE_DATA]
    energies = [r[1] for r in RECHARGE_DATA]
    visits = [r[2] for r in RECHARGE_DATA]
    saved: list[Path] = []

    import numpy as np

    x = np.arange(len(groups))
    w = 0.35

    fig, ax = plt.subplots(figsize=(7, 5))
    bars = ax.bar(x, energies, color=_COLORS["energy"])
    for bar, v in zip(bars, energies):
        ax.text(
            bar.get_x() + bar.get_width() / 2,
            bar.get_height() + 0.2,
            f"{v:.2f}",
            ha="center",
            va="bottom",
            fontsize=9,
        )
    ax.set_xticks(x)
    ax.set_xticklabels(groups)
    ax.set_ylabel("Energía media (kWh)")
    ax.set_title("Estaciones de recarga - Energía media por grupo (Tabla 4)")
    ax.set_ylim(285, 298)
    ax.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    out = EXTRAS_DIR / "paper_recharge_energy_bars.png"
    _savefig(fig, out)
    saved.append(out)

    fig, ax = plt.subplots(figsize=(7, 5))
    bars = ax.bar(x, visits, color=_COLORS["visits"])
    for bar, v in zip(bars, visits):
        ax.text(
            bar.get_x() + bar.get_width() / 2,
            bar.get_height() + 0.01,
            f"{v:.2f}",
            ha="center",
            va="bottom",
            fontsize=9,
        )
    ax.set_xticks(x)
    ax.set_xticklabels(groups)
    ax.set_ylabel("Visitas medias a estaciones")
    ax.set_title("Estaciones de recarga - Visitas medias por grupo (Tabla 4)")
    ax.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    out = EXTRAS_DIR / "paper_recharge_visits_bars.png"
    _savefig(fig, out)
    saved.append(out)

    # combinado: doble eje
    fig, ax1 = plt.subplots(figsize=(7, 5))
    ax2 = ax1.twinx()
    ax1.bar(x - w / 2, energies, w, label="Energía (kWh)", color=_COLORS["energy"])
    ax2.bar(x + w / 2, visits, w, label="Visitas", color=_COLORS["visits"])
    ax1.set_xticks(x)
    ax1.set_xticklabels(groups)
    ax1.set_ylabel("Energía media (kWh)", color=_COLORS["energy"])
    ax2.set_ylabel("Visitas medias", color=_COLORS["visits"])
    ax1.tick_params(axis="y", labelcolor=_COLORS["energy"])
    ax2.tick_params(axis="y", labelcolor=_COLORS["visits"])
    ax1.set_title("Estaciones de recarga - Energía y visitas (Tabla 4)")
    lines1, lab1 = ax1.get_legend_handles_labels()
    lines2, lab2 = ax2.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, lab1 + lab2)
    fig.tight_layout()
    out = EXTRAS_DIR / "paper_recharge_bars.png"
    _savefig(fig, out)
    saved.append(out)
    return saved


# EXTRAS / BATTERY-RESERVE


def _battery_bars() -> list[Path]:
    """Tabla 5: energía media y visitas por nivel de reserva."""
    levels = [r[0] for r in BATTERY_DATA]
    usables = [r[1] for r in BATTERY_DATA]
    energies = [r[2] for r in BATTERY_DATA]
    visits = [r[3] for r in BATTERY_DATA]
    saved: list[Path] = []

    import numpy as np

    x = np.arange(len(levels))
    labels_ext = [f"{lv}\n({us} kWh)" for lv, us in zip(levels, usables)]
    w = 0.35

    fig, ax = plt.subplots(figsize=(7, 5))
    bars = ax.bar(x, energies, color=_COLORS["energy"])
    for bar, v in zip(bars, energies):
        ax.text(
            bar.get_x() + bar.get_width() / 2,
            bar.get_height() + 0.02,
            f"{v:.2f}",
            ha="center",
            va="bottom",
            fontsize=9,
        )
    ax.set_xticks(x)
    ax.set_xticklabels(labels_ext)
    ax.set_ylabel("Energía media (kWh)")
    ax.set_title("Reserva de batería - Energía media por nivel (Tabla 5)")
    ax.set_ylim(291.5, 293.5)
    ax.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    out = EXTRAS_DIR / "paper_battery_energy_bars.png"
    _savefig(fig, out)
    saved.append(out)

    fig, ax = plt.subplots(figsize=(7, 5))
    bars = ax.bar(x, visits, color=_COLORS["visits"])
    for bar, v in zip(bars, visits):
        ax.text(
            bar.get_x() + bar.get_width() / 2,
            bar.get_height() + 0.01,
            f"{v:.2f}",
            ha="center",
            va="bottom",
            fontsize=9,
        )
    ax.set_xticks(x)
    ax.set_xticklabels(labels_ext)
    ax.set_ylabel("Visitas medias a estaciones")
    ax.set_title("Reserva de batería - Visitas medias por nivel (Tabla 5)")
    ax.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    out = EXTRAS_DIR / "paper_battery_visits_bars.png"
    _savefig(fig, out)
    saved.append(out)

    fig, ax1 = plt.subplots(figsize=(7, 5))
    ax2 = ax1.twinx()
    ax1.bar(x - w / 2, energies, w, label="Energía (kWh)", color=_COLORS["energy"])
    ax2.bar(x + w / 2, visits, w, label="Visitas", color=_COLORS["visits"])
    ax1.set_xticks(x)
    ax1.set_xticklabels(labels_ext)
    ax1.set_ylabel("Energía media (kWh)", color=_COLORS["energy"])
    ax2.set_ylabel("Visitas medias", color=_COLORS["visits"])
    ax1.tick_params(axis="y", labelcolor=_COLORS["energy"])
    ax2.tick_params(axis="y", labelcolor=_COLORS["visits"])
    ax1.set_title("Reserva de batería - Energía y visitas (Tabla 5)")
    lines1, lab1 = ax1.get_legend_handles_labels()
    lines2, lab2 = ax2.get_legend_handles_labels()
    ax1.legend(lines1 + lines2, lab1 + lab2, loc="upper left")
    fig.tight_layout()
    out = EXTRAS_DIR / "paper_battery_bars.png"
    _savefig(fig, out)
    saved.append(out)
    return saved


# EXTRAS / ENERGY-VS-DISTANCE


def _evd_bars_group(rows: list, g: int, total_groups: int) -> Path:
    """E_min vs E_dist por instancia, un grupo."""
    import numpy as np

    labels = [r[0] for r in rows]
    e_min = [r[1] for r in rows]
    e_dist = [r[2] for r in rows]

    x = np.arange(len(labels))
    w = 0.35
    fig, ax = plt.subplots(figsize=(max(8, len(labels) * 1.1), 5))
    ax.bar(x - w / 2, e_min, w, label="E_min (energía)", color=_COLORS["e_min"])
    ax.bar(x + w / 2, e_dist, w, label="E_dist (distancia)", color=_COLORS["e_dist"])
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=45, ha="right", fontsize=9)
    ax.set_ylabel("Energía (kWh)")
    ax.set_title(
        f"Energía vs Distancia - E_min vs E_dist, grupo {g}/{total_groups} (Tabla 6)"
    )
    ax.legend()
    ax.yaxis.set_minor_locator(ticker.AutoMinorLocator())
    ax.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    out = EXTRAS_DIR / f"paper_evd_bars_g{g}.png"
    _savefig(fig, out)
    return out


def _evd_pct_group(rows: list, g: int, total_groups: int) -> Path:
    """% incremento energía al usar distancia, un grupo."""
    import numpy as np

    labels = [r[0] for r in rows]
    pcts = [r[3] for r in rows]
    avg_g = sum(pcts) / len(pcts)

    x = np.arange(len(labels))
    fig, ax = plt.subplots(figsize=(max(8, len(labels) * 1.1), 5))
    bars = ax.bar(x, pcts, color=_COLORS["pct"])
    ax.axhline(avg_g, color="red", ls="--", lw=1.2, label=f"Media grupo: {avg_g:.2f}%")
    ax.axhline(
        EVD_AVG["pct"],
        color="black",
        ls=":",
        lw=1.2,
        label=f"Media global: {EVD_AVG['pct']:.2f}%",
    )
    for bar, v in zip(bars, pcts):
        ax.text(
            bar.get_x() + bar.get_width() / 2,
            bar.get_height() + 0.2,
            f"{v:.1f}%",
            ha="center",
            va="bottom",
            fontsize=8,
        )
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=45, ha="right", fontsize=9)
    ax.set_ylabel("% incremento energía")
    ax.set_title(
        f"Energía vs Distancia - % incremento, grupo {g}/{total_groups} (Tabla 6)"
    )
    ax.legend()
    ax.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    out = EXTRAS_DIR / f"paper_evd_pct_g{g}.png"
    _savefig(fig, out)
    return out


def _evd_bars_all() -> Path:
    """E_min vs E_dist — todas las instancias en un solo gráfico (Tabla 6)."""
    import numpy as np

    labels = [r[0] for r in EVD_DATA]
    e_min = [r[1] for r in EVD_DATA]
    e_dist = [r[2] for r in EVD_DATA]

    x = np.arange(len(labels))
    w = 0.35
    fig, ax = plt.subplots(figsize=(max(13, len(labels) * 0.45), 6))
    ax.bar(x - w / 2, e_min, w, label="E_min (energía)", color=_COLORS["e_min"])
    ax.bar(x + w / 2, e_dist, w, label="E_dist (distancia)", color=_COLORS["e_dist"])
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=90, ha="center", fontsize=7)
    ax.set_ylabel("Energía (kWh)")
    ax.set_title(
        "Energía vs Distancia - E_min vs E_dist, todas las instancias (Tabla 6)"
    )
    ax.legend()
    ax.yaxis.set_minor_locator(ticker.AutoMinorLocator())
    ax.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    out = EXTRAS_DIR / "paper_evd_bars.png"
    _savefig(fig, out)
    return out


def _evd_pct_all() -> Path:
    """% incremento energía al usar distancia — todas las instancias (Tabla 6)."""
    import numpy as np

    labels = [r[0] for r in EVD_DATA]
    pcts = [r[3] for r in EVD_DATA]

    x = np.arange(len(labels))
    fig, ax = plt.subplots(figsize=(max(13, len(labels) * 0.45), 6))
    bars = ax.bar(x, pcts, color=_COLORS["pct"])
    ax.axhline(
        EVD_AVG["pct"],
        color="black",
        ls=":",
        lw=1.2,
        label=f"Media global: {EVD_AVG['pct']:.2f}%",
    )
    for bar, v in zip(bars, pcts):
        ax.text(
            bar.get_x() + bar.get_width() / 2,
            bar.get_height() + 0.2,
            f"{v:.1f}%",
            ha="center",
            va="bottom",
            fontsize=6,
        )
    ax.set_xticks(x)
    ax.set_xticklabels(labels, rotation=90, ha="center", fontsize=7)
    ax.set_ylabel("% incremento energía")
    ax.set_title("Energía vs Distancia - % incremento, todas las instancias (Tabla 6)")
    ax.legend()
    ax.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    out = EXTRAS_DIR / "paper_evd_pct.png"
    _savefig(fig, out)
    return out


def _evd_summary() -> Path:
    """Resumen global: promedio E_min vs E_dist y % medio (Tabla 6)."""
    import numpy as np

    labels = ["E_min (energía)", "E_dist (distancia)"]
    vals = [EVD_AVG["e_min"], EVD_AVG["e_dist"]]

    x = np.arange(len(labels))
    fig, ax = plt.subplots(figsize=(6, 5))
    bars = ax.bar(x, vals, color=[_COLORS["e_min"], _COLORS["e_dist"]])
    for bar, v in zip(bars, vals):
        ax.text(
            bar.get_x() + bar.get_width() / 2,
            bar.get_height() + 0.5,
            f"{v:.2f}",
            ha="center",
            va="bottom",
            fontsize=10,
        )
    ax.annotate(
        f"Δ = +{EVD_AVG['pct']:.2f}% al minimizar distancia",
        xy=(0.5, 0.92),
        xycoords="axes fraction",
        ha="center",
        fontsize=10,
        bbox=dict(boxstyle="round,pad=0.3", fc="#FFF3CD", ec="#FFC107"),
    )
    ax.set_xticks(x)
    ax.set_xticklabels(labels)
    ax.set_ylabel("Energía media (kWh)")
    ax.set_title("Energía vs Distancia - Resumen global (Tabla 6)")
    ax.set_ylim(0, 380)
    ax.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    out = EXTRAS_DIR / "paper_evd_summary.png"
    _savefig(fig, out)
    return out


def generate_extras_charts() -> list[Path]:
    print("\n[extras] Generando graficos en stats_paper/extras/ ...")
    saved: list[Path] = []

    # recharge-stations
    saved.extend(_recharge_bars())

    # battery-reserve
    saved.extend(_battery_bars())

    # energy-vs-distance
    groups = _grouped(EVD_DATA, GROUP_SIZE)
    n = len(groups)
    for i, grp in enumerate(groups, start=1):
        saved.append(_evd_bars_group(grp, i, n))
        saved.append(_evd_pct_group(grp, i, n))
    saved.append(_evd_bars_all())
    saved.append(_evd_pct_all())
    saved.append(_evd_summary())

    return saved


def main() -> None:
    print("=" * 60)
    print("stats_paper.py - Graficos del paper Zhang et al. (2018)")
    print("=" * 60)

    all_saved: list[Path] = []
    all_saved.extend(generate_small_charts())
    all_saved.extend(generate_large_charts())
    all_saved.extend(generate_extras_charts())

    print(f"\n[OK] {len(all_saved)} grafico(s) generado(s) en stats_paper/")
    print("  small/  :", SMALL_DIR.relative_to(PROJECT_DIR))
    print("  large/  :", LARGE_DIR.relative_to(PROJECT_DIR))
    print("  extras/ :", EXTRAS_DIR.relative_to(PROJECT_DIR))


if __name__ == "__main__":
    main()
