import random
import statistics
import time
from pathlib import Path


def parse_instance(file_path):
    with open(file_path) as file_handle:
        tokens = file_handle.read().split()

    if not tokens:
        return 0, 0, 0, [], [], []

    index = 0
    item_count, resource_count, edge_count, budget = (
        int(tokens[index]),
        int(tokens[index + 1]),
        int(tokens[index + 2]),
        int(tokens[index + 3]),
    )
    index += 4

    benefits = [int(tokens[index + i]) for i in range(item_count)]
    index += item_count

    weights = [int(tokens[index + j]) for j in range(resource_count)]
    index += resource_count

    adjacency = [set() for _ in range(item_count)]
    for _ in range(edge_count):
        item_index, resource_index = int(tokens[index]), int(tokens[index + 1])
        adjacency[item_index].add(resource_index)
        index += 2

    return item_count, resource_count, budget, benefits, weights, adjacency


def evaluate_solution(selected, benefits, weights, adjacency):
    used_resources = set()
    for item_index in selected:
        used_resources.update(adjacency[item_index])
    cost = sum(weights[resource_index] for resource_index in used_resources)
    benefit = sum(benefits[item_index] for item_index in selected)
    return benefit, cost


def compute_marginal_cost(item_index, selected_resources, adjacency, weights):
    return sum(
        weights[resource_index]
        for resource_index in adjacency[item_index]
        if resource_index not in selected_resources
    )


def greedy_deterministic(item_count, budget, benefits, weights, adjacency):
    selected = set()
    selected_resources = set()
    current_cost = 0
    candidates = set(range(item_count))

    while candidates:
        best_item = None
        best_ratio = -1.0
        best_marginal_cost = float("inf")

        for item_index in candidates:
            marginal = compute_marginal_cost(
                item_index, selected_resources, adjacency, weights
            )
            if current_cost + marginal > budget:
                continue
            ratio = benefits[item_index] / marginal if marginal > 0 else float("inf")

            if (ratio > best_ratio) or (
                ratio == best_ratio and marginal < best_marginal_cost
            ):
                best_item = item_index
                best_ratio = ratio
                best_marginal_cost = marginal

        if best_item is None:
            break

        selected.add(best_item)
        selected_resources.update(adjacency[best_item])
        current_cost += best_marginal_cost
        candidates.remove(best_item)

    return selected


def greedy_stochastic(
    item_count, budget, benefits, weights, adjacency, alpha=0.3, seed=None
):
    random_generator = random.Random(seed)
    selected = set()
    selected_resources = set()
    current_cost = 0
    candidates = set(range(item_count))

    while candidates:
        feasible = []
        for item_index in candidates:
            marginal = compute_marginal_cost(
                item_index, selected_resources, adjacency, weights
            )
            if current_cost + marginal <= budget:
                ratio = (
                    benefits[item_index] / marginal if marginal > 0 else float("inf")
                )
                feasible.append((item_index, ratio, marginal))

        if not feasible:
            break

        infinite_ratio_candidates = [
            (item_index, marginal)
            for item_index, ratio, marginal in feasible
            if ratio == float("inf")
        ]
        if infinite_ratio_candidates:
            chosen_item, chosen_marginal = random_generator.choice(
                infinite_ratio_candidates
            )
        else:
            ratios = [ratio for _, ratio, _ in feasible]
            ratio_max, ratio_min = max(ratios), min(ratios)
            threshold = ratio_max - alpha * (ratio_max - ratio_min)
            restricted_list = [
                (item_index, marginal)
                for item_index, ratio, marginal in feasible
                if ratio >= threshold
            ]
            chosen_item, chosen_marginal = random_generator.choice(restricted_list)

        selected.add(chosen_item)
        selected_resources.update(adjacency[chosen_item])
        current_cost += chosen_marginal
        candidates.remove(chosen_item)

    return selected


def run_instance(instance_name, file_path, stochastic_runs=10, alpha=0.3):
    print(f"\n{'=' * 60}")
    print(f"  Instance: {instance_name}")
    print(f"{'=' * 60}")

    start_time = time.time()
    item_count, resource_count, budget, benefits, weights, adjacency = parse_instance(
        file_path
    )
    print(f"  m={item_count}, n={resource_count}, B={budget}")

    deterministic_start = time.time()
    deterministic_solution = greedy_deterministic(
        item_count, budget, benefits, weights, adjacency
    )
    deterministic_benefit, deterministic_cost = evaluate_solution(
        deterministic_solution, benefits, weights, adjacency
    )
    deterministic_elapsed = time.time() - deterministic_start
    print(f"\n  [Greedy Determinista]")
    print(f"    Beneficio : {deterministic_benefit}")
    print(f"    Costo     : {deterministic_cost}  (cap={budget})")
    print(f"    |Sol|     : {len(deterministic_solution)}")
    print(f"    Tiempo    : {deterministic_elapsed:.4f}s")

    stochastic_benefits = []
    best_stochastic_benefit = -1

    print(f"\n  [Greedy Estocástico x{stochastic_runs}, alpha={alpha}]")
    for run_index in range(stochastic_runs):
        run_start = time.time()
        stochastic_solution = greedy_stochastic(
            item_count,
            budget,
            benefits,
            weights,
            adjacency,
            alpha=alpha,
            seed=run_index * 42 + 7,
        )
        run_benefit, run_cost = evaluate_solution(
            stochastic_solution, benefits, weights, adjacency
        )
        run_elapsed = time.time() - run_start
        stochastic_benefits.append(run_benefit)
        if run_benefit > best_stochastic_benefit:
            best_stochastic_benefit = run_benefit
        print(
            f"    Run {run_index + 1:2d}: beneficio={run_benefit:>8}  "
            f"costo={run_cost:>8}  t={run_elapsed:.4f}s"
        )

    stochastic_mean = statistics.mean(stochastic_benefits)
    stochastic_std = statistics.stdev(stochastic_benefits) if stochastic_runs > 1 else 0
    stochastic_best = max(stochastic_benefits)
    stochastic_worst = min(stochastic_benefits)
    stochastic_median = statistics.median(stochastic_benefits)

    print(f"\n  Estadísticas Greedy Estocástico:")
    print(f"    Media     : {stochastic_mean:.2f}")
    print(f"    Desv. Est.: {stochastic_std:.2f}")
    print(f"    Mejor     : {stochastic_best}")
    print(f"    Peor      : {stochastic_worst}")
    print(f"    Mediana   : {stochastic_median}")

    print(f"\n  Comparación:")
    print(f"    Greedy Det.   : {deterministic_benefit}")
    print(f"    Mejor Estoc.  : {best_stochastic_benefit}")
    improvement_gap = (
        (best_stochastic_benefit - deterministic_benefit) / deterministic_benefit * 100
        if deterministic_benefit > 0
        else 0
    )
    print(f"    Gap (mejor estoc. vs det.): {improvement_gap:+.2f}%")

    total_elapsed = time.time() - start_time
    print(f"\n  Tiempo total instancia: {total_elapsed:.4f}s")

    return {
        "name": instance_name,
        "det_benefit": deterministic_benefit,
        "sto_best_benefit": best_stochastic_benefit,
        "mean": stochastic_mean,
        "std": stochastic_std,
    }


if __name__ == "__main__":
    PART_DIR = Path(__file__).resolve().parent
    CASES_DIR = PART_DIR.parent / "cases"
    INSTANCE_SPECS = [
        ("easy", CASES_DIR / "easy.txt"),
        ("medium1", CASES_DIR / "medium1.txt"),
        ("medium2", CASES_DIR / "medium2.txt"),
        ("hard", CASES_DIR / "hard.txt"),
    ]
    STOCHASTIC_RUNS = 10
    STOCHASTIC_ALPHA = 0.3

    instance_results = []
    for instance_name, instance_path in INSTANCE_SPECS:
        if instance_path.exists():
            instance_results.append(
                run_instance(
                    instance_name,
                    instance_path,
                    stochastic_runs=STOCHASTIC_RUNS,
                    alpha=STOCHASTIC_ALPHA,
                )
            )
        else:
            print(f"Archivo no encontrado: {instance_path}")

    if instance_results:
        print("\n\n" + "=" * 60)
        print("  RESUMEN GLOBAL")
        print("=" * 60)
        print(
            f"  {'Instancia':<12} {'Det.':>10} {'Sto.Best':>10} "
            f"{'Sto.Mean':>10} {'Sto.Std':>9}"
        )
        print("  " + "-" * 55)
        for result in instance_results:
            print(
                f"  {result['name']:<12} {result['det_benefit']:>10} "
                f"{result['sto_best_benefit']:>10} {result['mean']:>10.1f} "
                f"{result['std']:>9.1f}"
            )
