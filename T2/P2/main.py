import random
import statistics
import time
from collections import deque
from pathlib import Path

DEFAULT_LIST = 50


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
        best_item, best_ratio, best_marginal_cost = None, -1.0, float("inf")
        for item_index in candidates:
            marginal = compute_marginal_cost(
                item_index, selected_resources, adjacency, weights
            )
            if current_cost + marginal > budget:
                continue
            ratio = benefits[item_index] / marginal if marginal > 0 else float("inf")
            if ratio > best_ratio or (
                ratio == best_ratio and marginal < best_marginal_cost
            ):
                best_item, best_ratio, best_marginal_cost = item_index, ratio, marginal
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


def get_used_resources(selected, adjacency):
    used_resources = set()
    for item_index in selected:
        used_resources.update(adjacency[item_index])
    return used_resources


def compute_cost_from_resources(used_resources, weights):
    return sum(weights[resource_index] for resource_index in used_resources)


def evaluate_add(
    item_index,
    selected,
    selected_resources,
    benefits,
    weights,
    adjacency,
    budget,
    current_cost,
):
    extra_cost = sum(
        weights[resource_index]
        for resource_index in adjacency[item_index]
        if resource_index not in selected_resources
    )
    new_cost = current_cost + extra_cost
    if new_cost > budget:
        return None
    new_benefit = (
        sum(benefits[selected_item] for selected_item in selected)
        + benefits[item_index]
    )
    return new_benefit, new_cost


def evaluate_remove(
    item_index, selected, selected_resources, benefits, weights, adjacency
):
    remaining = selected - {item_index}
    new_used_resources = get_used_resources(remaining, adjacency)
    new_cost = compute_cost_from_resources(new_used_resources, weights)
    new_benefit = sum(benefits[selected_item] for selected_item in remaining)
    return new_benefit, new_cost, new_used_resources


def evaluate_swap(
    remove_item,
    add_item,
    selected,
    selected_resources,
    benefits,
    weights,
    adjacency,
    budget,
    current_benefit,
    current_cost,
):
    remaining = selected - {remove_item}
    new_used_resources = get_used_resources(remaining, adjacency)
    extra_cost = sum(
        weights[resource_index]
        for resource_index in adjacency[add_item]
        if resource_index not in new_used_resources
    )
    new_cost = compute_cost_from_resources(new_used_resources, weights) + extra_cost
    if new_cost > budget:
        return None
    new_benefit = (
        sum(benefits[selected_item] for selected_item in remaining) + benefits[add_item]
    )
    new_used_resources.update(adjacency[add_item])
    return new_benefit, new_cost, new_used_resources


def tabu_search(
    item_count,
    budget,
    benefits,
    weights,
    adjacency,
    initial_solution,
    max_iterations=200,
    tabu_tenure=10,
    neighbor_sample_size=None,
    seed=None,
):
    random_generator = random.Random(seed)

    current_solution = set(initial_solution)
    current_benefit, current_cost = evaluate_solution(
        current_solution, benefits, weights, adjacency
    )
    current_used_resources = get_used_resources(current_solution, adjacency)

    best_solution = set(current_solution)
    best_benefit = current_benefit

    tabu_queue = deque()
    tabu_moves = set()

    best_history = [current_benefit]
    current_history = [current_benefit]
    iter_times = [0.0]

    non_selected = set(range(item_count)) - current_solution
    t_start = time.time()

    for iteration in range(max_iterations):
        neighbors = []

        add_candidates = list(non_selected)
        if neighbor_sample_size and len(add_candidates) > neighbor_sample_size // 2:
            add_candidates = random_generator.sample(
                add_candidates, neighbor_sample_size // 2
            )

        for item_index in add_candidates:
            result = evaluate_add(
                item_index,
                current_solution,
                current_used_resources,
                benefits,
                weights,
                adjacency,
                budget,
                current_cost,
            )
            if result is None:
                continue
            new_benefit, new_cost = result
            move = ("add", item_index)
            inverse_move = ("remove", item_index)
            neighbors.append((new_benefit, new_cost, move, inverse_move))

        remove_candidates = list(current_solution)
        if neighbor_sample_size and len(remove_candidates) > neighbor_sample_size // 4:
            remove_candidates = random_generator.sample(
                remove_candidates, neighbor_sample_size // 4
            )

        for item_index in remove_candidates:
            new_benefit, new_cost, _ = evaluate_remove(
                item_index,
                current_solution,
                current_used_resources,
                benefits,
                weights,
                adjacency,
            )
            move = ("remove", item_index)
            inverse_move = ("add", item_index)
            neighbors.append((new_benefit, new_cost, move, inverse_move))

        swap_out_candidates = list(current_solution)
        swap_in_candidates = list(non_selected)
        if neighbor_sample_size:
            sample_count = max(1, neighbor_sample_size // 4)
            swap_out_candidates = random_generator.sample(
                swap_out_candidates, min(sample_count, len(swap_out_candidates))
            )
            swap_in_candidates = random_generator.sample(
                swap_in_candidates, min(sample_count, len(swap_in_candidates))
            )

        for remove_item in swap_out_candidates:
            for add_item in swap_in_candidates:
                result = evaluate_swap(
                    remove_item,
                    add_item,
                    current_solution,
                    current_used_resources,
                    benefits,
                    weights,
                    adjacency,
                    budget,
                    current_benefit,
                    current_cost,
                )
                if result is None:
                    continue
                new_benefit, new_cost, _ = result
                move = ("swap", remove_item, add_item)
                inverse_move = ("swap", add_item, remove_item)
                neighbors.append((new_benefit, new_cost, move, inverse_move))

        if not neighbors:
            break

        neighbors.sort(key=lambda neighbor: (-neighbor[0], neighbor[1]))

        chosen_neighbor = None
        for neighbor in neighbors:
            new_benefit, new_cost, move, inverse_move = neighbor
            is_tabu = inverse_move in tabu_moves
            if is_tabu and new_benefit <= best_benefit:
                continue
            chosen_neighbor = neighbor
            break

        if chosen_neighbor is None:
            chosen_neighbor = neighbors[0]

        new_benefit, new_cost, move, inverse_move = chosen_neighbor

        if move[0] == "add":
            item_index = move[1]
            current_solution.add(item_index)
            non_selected.discard(item_index)
            current_used_resources.update(adjacency[item_index])
        elif move[0] == "remove":
            item_index = move[1]
            current_solution.discard(item_index)
            non_selected.add(item_index)
            current_used_resources = get_used_resources(current_solution, adjacency)
        else:
            remove_item, add_item = move[1], move[2]
            current_solution.discard(remove_item)
            non_selected.add(remove_item)
            current_solution.add(add_item)
            non_selected.discard(add_item)
            current_used_resources = get_used_resources(current_solution, adjacency)

        current_benefit = new_benefit
        current_cost = new_cost

        tabu_queue.append((inverse_move, iteration + tabu_tenure))
        tabu_moves.add(inverse_move)

        while tabu_queue and tabu_queue[0][1] <= iteration:
            expired_move, _ = tabu_queue.popleft()
            tabu_moves.discard(expired_move)

        if current_benefit > best_benefit:
            best_benefit = current_benefit
            best_solution = set(current_solution)

        best_history.append(best_benefit)
        current_history.append(current_benefit)
        iter_times.append(time.time() - t_start)

    return best_solution, best_benefit, best_history, current_history, iter_times


def ask_tenure():
    print(f"\n  Parámetro: largo de lista tabú (tenure)")
    print(f"  Default: {DEFAULT_LIST}")
    raw = input(f"  Ingrese tenure [Enter = {DEFAULT_LIST}]: ").strip()
    if raw == "":
        print(f"  -> Usando tenure default: {DEFAULT_LIST}")
        return DEFAULT_LIST
    try:
        val = int(raw)
        if val <= 0:
            raise ValueError
        print(f"  -> Usando tenure: {val}")
        return val
    except ValueError:
        print(f"  -> Valor invalido, usando default: {DEFAULT_LIST}")
        return DEFAULT_LIST


def run_tabu_instance(
    instance_name,
    file_path,
    tabu_tenure,
    max_iterations=200,
    stochastic_runs=10,
    stochastic_alpha=0.3,
    output_lines=None,
):
    def log(line):
        print(line)
        if output_lines is not None:
            output_lines.append(line)

    log(f"\n{'=' * 65}")
    log(f"  TABU SEARCH - Instancia: {instance_name}")
    log(f"{'=' * 65}")

    item_count, resource_count, budget, benefits, weights, adjacency = parse_instance(
        file_path
    )
    log(f"  m={item_count}, n={resource_count}, B={budget}")
    log(f"  Tenure usado: {tabu_tenure}")
    log(f"  Parámetros TS: max_iter={max_iterations}, tenure={tabu_tenure}")

    deterministic_start = time.time()
    deterministic_solution = greedy_deterministic(
        item_count, budget, benefits, weights, adjacency
    )
    deterministic_benefit, deterministic_cost = evaluate_solution(
        deterministic_solution, benefits, weights, adjacency
    )

    (
        tabu_from_deterministic_solution,
        tabu_from_deterministic_benefit,
        best_hist_det,
        curr_hist_det,
        times_det,
    ) = tabu_search(
        item_count,
        budget,
        benefits,
        weights,
        adjacency,
        initial_solution=deterministic_solution,
        max_iterations=max_iterations,
        tabu_tenure=tabu_tenure,
        seed=0,
    )
    tabu_from_deterministic_cost = evaluate_solution(
        tabu_from_deterministic_solution, benefits, weights, adjacency
    )[1]
    deterministic_elapsed = time.time() - deterministic_start

    log(f"\n  [TS desde Greedy Determinista]")
    log(f"    Inicio   : beneficio={deterministic_benefit}, costo={deterministic_cost}")
    log(
        f"    Resultado: beneficio={tabu_from_deterministic_benefit}, "
        f"costo={tabu_from_deterministic_cost}"
    )
    deterministic_improvement = (
        (tabu_from_deterministic_benefit - deterministic_benefit)
        / deterministic_benefit
        * 100
        if deterministic_benefit > 0
        else 0
    )
    log(f"    Mejora vs partida: {deterministic_improvement:+.2f}%")
    log(f"    Tiempo   : {deterministic_elapsed:.4f}s")
    log(f"    Best History Det: {' '.join(map(str, best_hist_det))}")
    log(f"    Current History Det: {' '.join(map(str, curr_hist_det))}")
    log(f"    Iter Times Det: {' '.join(f'{v:.4f}' for v in times_det)}")

    log(
        f"\n  [TS desde Greedy Estocástico x{stochastic_runs}, alpha={stochastic_alpha}]"
    )
    stochastic_start_benefits = []
    tabu_from_stochastic_results = []

    for run_index in range(stochastic_runs):
        run_start = time.time()
        stochastic_solution = greedy_stochastic(
            item_count,
            budget,
            benefits,
            weights,
            adjacency,
            alpha=stochastic_alpha,
            seed=run_index * 42 + 7,
        )
        stochastic_benefit, stochastic_cost = evaluate_solution(
            stochastic_solution, benefits, weights, adjacency
        )
        stochastic_start_benefits.append(stochastic_benefit)

        (
            tabu_solution,
            tabu_benefit,
            best_hist_sto,
            curr_hist_sto,
            times_sto,
        ) = tabu_search(
            item_count,
            budget,
            benefits,
            weights,
            adjacency,
            initial_solution=stochastic_solution,
            max_iterations=max_iterations,
            tabu_tenure=tabu_tenure,
            seed=run_index,
        )
        tabu_cost = evaluate_solution(tabu_solution, benefits, weights, adjacency)[1]
        run_elapsed = time.time() - run_start
        tabu_from_stochastic_results.append(
            (
                tabu_benefit,
                tabu_cost,
                run_elapsed,
                best_hist_sto,
                curr_hist_sto,
                times_sto,
            )
        )

        log(
            f"    Run {run_index + 1:2d}: partida={stochastic_benefit:>7}  "
            f"TS={tabu_benefit:>7}  costo={tabu_cost:>7}  t={run_elapsed:.4f}s"
        )
        log(
            f"    Best History Sto {run_index + 1}: {' '.join(map(str, best_hist_sto))}"
        )
        log(
            f"    Current History Sto {run_index + 1}: {' '.join(map(str, curr_hist_sto))}"
        )
        log(
            f"    Iter Times Sto {run_index + 1}: "
            f"{' '.join(f'{v:.4f}' for v in times_sto)}"
        )

    tabu_stochastic_benefits = [result[0] for result in tabu_from_stochastic_results]
    stochastic_mean = statistics.mean(tabu_stochastic_benefits)
    stochastic_std = (
        statistics.stdev(tabu_stochastic_benefits) if stochastic_runs > 1 else 0
    )
    stochastic_best = max(tabu_stochastic_benefits)
    stochastic_worst = min(tabu_stochastic_benefits)
    stochastic_median = statistics.median(tabu_stochastic_benefits)

    log(f"\n  Estadísticas TS desde Greedy Estocástico:")
    log(f"    Media     : {stochastic_mean:.2f}")
    log(f"    Desv. Est.: {stochastic_std:.2f}")
    log(f"    Mejor     : {stochastic_best}")
    log(f"    Peor      : {stochastic_worst}")
    log(f"    Mediana   : {stochastic_median:.1f}")

    best_overall = max(tabu_from_deterministic_benefit, stochastic_best)
    log(f"\n  Mejor global (det vs sto): {best_overall}")

    return {
        "name": instance_name,
        "m": item_count,
        "n": resource_count,
        "B": budget,
        "det_ben": deterministic_benefit,
        "sto_best_ben": max(stochastic_start_benefits),
        "ts_det_ben": tabu_from_deterministic_benefit,
        "ts_det_cost": tabu_from_deterministic_cost,
        "best_hist_det": best_hist_det,
        "curr_hist_det": curr_hist_det,
        "times_det": times_det,
        "ts_sto_results": tabu_from_stochastic_results,
        "ts_sto_mean": stochastic_mean,
        "ts_sto_std": stochastic_std,
        "ts_sto_best": stochastic_best,
        "ts_sto_worst": stochastic_worst,
        "ts_sto_median": stochastic_median,
        "ts_best_overall": best_overall,
        "tabu_tenure": tabu_tenure,
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
    TABU_MAX_ITERATIONS = 200
    STOCHASTIC_RUNS = 10
    STOCHASTIC_ALPHA = 0.3

    TABU_TENURE = ask_tenure()
    RESULTS_FILE = PART_DIR / f"results_tenure{TABU_TENURE}.txt"

    output_lines = [f"  Tenure usado: {TABU_TENURE}"]
    all_results = []

    for instance_name, instance_path in INSTANCE_SPECS:
        if instance_path.exists():
            all_results.append(
                run_tabu_instance(
                    instance_name,
                    instance_path,
                    tabu_tenure=TABU_TENURE,
                    max_iterations=TABU_MAX_ITERATIONS,
                    stochastic_runs=STOCHASTIC_RUNS,
                    stochastic_alpha=STOCHASTIC_ALPHA,
                    output_lines=output_lines,
                )
            )
        else:
            print(f"Archivo no encontrado: {instance_path}")

    if all_results:
        print("\n\n" + "=" * 75)
        print("  RESUMEN GLOBAL - TABU SEARCH")
        print("=" * 75)
        summary_header = (
            f"  {'Inst.':<10} {'Det.Ini':>8} {'TS/Det':>8} {'StoIni':>8} "
            f"{'TS/Sto.B':>9} {'TS/Sto.M':>9} {'TS/Sto.S':>8}"
        )
        print(summary_header)
        print("  " + "-" * 70)

        output_lines.extend(
            [
                "",
                "",
                "=" * 75,
                "  RESUMEN GLOBAL - TABU SEARCH",
                "=" * 75,
                summary_header,
                "  " + "-" * 70,
            ]
        )

        for result in all_results:
            row = (
                f"  {result['name']:<10} "
                f"{result['det_ben']:>8} "
                f"{result['ts_det_ben']:>8} "
                f"{result['sto_best_ben']:>8} "
                f"{result['ts_sto_best']:>9} "
                f"{result['ts_sto_mean']:>9.1f} "
                f"{result['ts_sto_std']:>8.1f}"
            )
            print(row)
            output_lines.append(row)

    RESULTS_FILE.write_text("\n".join(output_lines), encoding="utf-8")
    print(f"\n  Resultados guardados en: {RESULTS_FILE}")
