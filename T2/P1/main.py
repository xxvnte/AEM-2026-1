"""
Tarea 2 - CIT3352: Algoritmos Exactos y Metaheurísticas
Parte 1: Greedy Determinista y Greedy Estocástico (Optimizado)
"""

import random
import statistics
import time
from pathlib import Path

# ─────────────────────────────────────────────
# Parsing
# ─────────────────────────────────────────────

def parse_instance(filepath):
    with open(filepath) as f:
        tokens = f.read().split()

    if not tokens:
        return 0, 0, 0, [], [], []

    idx = 0
    m, n, ne, B = int(tokens[idx]), int(tokens[idx+1]), int(tokens[idx+2]), int(tokens[idx+3])
    idx += 4

    benefits = [int(tokens[idx+i]) for i in range(m)]
    idx += m

    weights = [int(tokens[idx+j]) for j in range(n)]
    idx += n

    adj = [set() for _ in range(m)]
    for _ in range(ne):
        i, j = int(tokens[idx]), int(tokens[idx+1])
        adj[i].add(j)
        idx += 2

    return m, n, B, benefits, weights, adj


# ─────────────────────────────────────────────
# Evaluation & Heuristics
# ─────────────────────────────────────────────

def evaluate(selected, benefits, weights, adj):
    used_resources = set()
    for i in selected:
        used_resources.update(adj[i])
    cost = sum(weights[j] for j in used_resources)
    benefit = sum(benefits[i] for i in selected)
    return benefit, cost


def marginal_cost(i, selected_resources, adj, weights):
    # Optimización: evita instanciar un nuevo set con la resta de conjuntos
    return sum(weights[j] for j in adj[i] if j not in selected_resources)


# ─────────────────────────────────────────────
# Greedy Determinista
# ─────────────────────────────────────────────

def greedy_deterministic(m, B, benefits, weights, adj):
    selected = set()
    selected_resources = set()
    current_cost = 0

    # Usamos set para eliminación O(1)
    candidates = set(range(m))

    while candidates:
        best = None
        best_ratio = -1.0
        best_mc = float('inf')

        for i in candidates:
            mc = marginal_cost(i, selected_resources, adj, weights)
            if current_cost + mc > B:
                continue
            ratio = benefits[i] / mc if mc > 0 else float('inf')
            
            # Criterio lexicográfico
            if (ratio > best_ratio) or (ratio == best_ratio and mc < best_mc):
                best = i
                best_ratio = ratio
                best_mc = mc

        if best is None:
            break  # Ningún candidato restante cabe en el presupuesto

        selected.add(best)
        selected_resources.update(adj[best])
        current_cost += best_mc
        candidates.remove(best)  # Operación O(1)

    return selected


# ─────────────────────────────────────────────
# Greedy Estocástico
# ─────────────────────────────────────────────

def greedy_stochastic(m, B, benefits, weights, adj, alpha=0.3, seed=None):
    rng = random.Random(seed)
    selected = set()
    selected_resources = set()
    current_cost = 0

    candidates = set(range(m))

    while candidates:
        feasible = []
        for i in candidates:
            mc = marginal_cost(i, selected_resources, adj, weights)
            if current_cost + mc <= B:
                ratio = benefits[i] / mc if mc > 0 else float('inf')
                feasible.append((i, ratio, mc))

        if not feasible:
            break

        # Manejo de costos marginales cero (Ratios infinitos)
        inf_cands = [(i, mc) for i, r, mc in feasible if r == float('inf')]
        if inf_cands:
            chosen_i, chosen_mc = rng.choice(inf_cands)
        else:
            ratios = [r for _, r, _ in feasible]
            r_max, r_min = max(ratios), min(ratios)
            
            threshold = r_max - alpha * (r_max - r_min)
            rcl = [(i, mc) for i, r, mc in feasible if r >= threshold]
            chosen_i, chosen_mc = rng.choice(rcl)

        selected.add(chosen_i)
        selected_resources.update(adj[chosen_i])
        current_cost += chosen_mc
        candidates.remove(chosen_i)  # Operación O(1)

    return selected


# ─────────────────────────────────────────────
# Run experiments
# ─────────────────────────────────────────────

def run_instance(name, filepath, n_stochastic=10, alpha=0.3):
    print(f"\n{'='*60}")
    print(f"  Instance: {name}")
    print(f"{'='*60}")

    t0 = time.time()
    m, n, B, benefits, weights, adj = parse_instance(filepath)
    print(f"  m={m}, n={n}, B={B}")

    # ── Greedy Determinista ──────────────────
    t1 = time.time()
    det_sol = greedy_deterministic(m, B, benefits, weights, adj)
    det_benefit, det_cost = evaluate(det_sol, benefits, weights, adj)
    t_det = time.time() - t1
    print(f"\n  [Greedy Determinista]")
    print(f"    Beneficio : {det_benefit}")
    print(f"    Costo     : {det_cost}  (cap={B})")
    print(f"    |Sol|     : {len(det_sol)}")
    print(f"    Tiempo    : {t_det:.4f}s")

    # ── Greedy Estocástico ───────────────────
    sto_benefits = []
    best_sto_sol = None
    best_sto_benefit = -1

    print(f"\n  [Greedy Estocástico x{n_stochastic}, alpha={alpha}]")
    for run in range(n_stochastic):
        t1 = time.time()
        sol = greedy_stochastic(m, B, benefits, weights, adj, alpha=alpha, seed=run*42+7)
        b, c = evaluate(sol, benefits, weights, adj)
        elapsed = time.time() - t1
        sto_benefits.append(b)
        if b > best_sto_benefit:
            best_sto_benefit = b
            best_sto_sol = sol
        print(f"    Run {run+1:2d}: beneficio={b:>8}  costo={c:>8}  t={elapsed:.4f}s")

    mean_b   = statistics.mean(sto_benefits)
    std_b    = statistics.stdev(sto_benefits) if n_stochastic > 1 else 0
    best_b   = max(sto_benefits)
    worst_b  = min(sto_benefits)
    median_b = statistics.median(sto_benefits)

    print(f"\n  Estadísticas Greedy Estocástico:")
    print(f"    Media     : {mean_b:.2f}")
    print(f"    Desv. Est.: {std_b:.2f}")
    print(f"    Mejor     : {best_b}")
    print(f"    Peor      : {worst_b}")
    print(f"    Mediana   : {median_b}")

    print(f"\n  Comparación:")
    print(f"    Greedy Det.   : {det_benefit}")
    print(f"    Mejor Estoc.  : {best_sto_benefit}")
    gap = (best_sto_benefit - det_benefit) / det_benefit * 100 if det_benefit > 0 else 0
    print(f"    Gap (mejor estoc. vs det.): {gap:+.2f}%")

    total_t = time.time() - t0
    print(f"\n  Tiempo total instancia: {total_t:.4f}s")

    return {
        "name": name,
        "det_benefit": det_benefit,
        "sto_best_benefit": best_sto_benefit,
        "mean": mean_b,
        "std": std_b,
    }


if __name__ == "__main__":
    base = Path(".")
    instances = [
        ("easy",    base / "easy.txt"),
        ("medium1", base / "medium1.txt"),
        ("medium2", base / "medium2.txt"),
        ("hard",    base / "hard.txt"),
    ]

    results = []
    for name, path in instances:
        if path.exists():
            r = run_instance(name, path, n_stochastic=10, alpha=0.3)
            results.append(r)
        else:
            print(f"Archivo no encontrado: {path}")

    if results:
        print("\n\n" + "="*60)
        print("  RESUMEN GLOBAL")
        print("="*60)
        print(f"  {'Instancia':<12} {'Det.':>10} {'Sto.Best':>10} {'Sto.Mean':>10} {'Sto.Std':>9}")
        print("  " + "-"*55)
        for r in results:
            print(f"  {r['name']:<12} {r['det_benefit']:>10} {r['sto_best_benefit']:>10} "
                  f"{r['mean']:>10.1f} {r['std']:>9.1f}")