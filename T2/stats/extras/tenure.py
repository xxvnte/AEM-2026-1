import matplotlib.pyplot as plt
from pathlib import Path

output_dir = Path(__file__).parent

tenures = [5, 7, 10, 13, 15, 20, 25, 30, 40, 50]
data = {
    "easy": {
        "mejor": [11313, 11313, 11313, 11313, 11313, 11623, 11623, 11623, 11623, 12045],
        "media": [
            11080.7,
            11080.7,
            11080.7,
            11123.2,
            11132.9,
            11230.0,
            11224.8,
            11244.1,
            11292.2,
            11372.6,
        ],
    },
    "medium1": {
        "mejor": [10973] * 10,
        "media": [
            10044.5,
            10178.2,
            10191.8,
            10191.8,
            10205.7,
            10309.7,
            10400.5,
            10463.7,
            10488.1,
            10531.1,
        ],
    },
    "medium2": {
        "mejor": [12034] * 10,
        "media": [
            11284.8,
            11329.0,
            11329.0,
            11330.0,
            11346.7,
            11408.6,
            11448.8,
            11510.1,
            11535.3,
            11642.2,
        ],
    },
    "hard": {
        "mejor": [9586, 9586, 9586, 9586, 9586, 9600, 9600, 9600, 9600, 9705],
        "media": [
            8985.9,
            9001.2,
            9028.1,
            9050.5,
            9050.5,
            9089.4,
            9089.4,
            9109.4,
            9156.0,
            9167.1,
        ],
    },
}

fig, axes = plt.subplots(1, 2, figsize=(12, 5))
for name, vals in data.items():
    axes[0].plot(tenures, vals["mejor"], marker="o", label=name)
    axes[1].plot(tenures, vals["media"], marker="o", label=name)

axes[0].set_title("Mejor solución vs Tenure")
axes[1].set_title("Media vs Tenure")
for ax in axes:
    ax.set_xlabel("Tenure")
    ax.set_ylabel("Beneficio")
    ax.legend()
    ax.grid(alpha=0.3)

plt.tight_layout()
plt.savefig(output_dir/ "tenure_comparativa.png", dpi=150)
