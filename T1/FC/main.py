"""
Commands:

structured as:
python main.py [case.txt|all] [runway_count] [timeout_seconds] (defaults: all cases, 1 runway, 60s timeout)

examples:
    python main.py all
    python main.py case1.txt
    python main.py case2.txt 2
    python main.py case3.txt 3 120
"""

import os
import sys
import time


def load_case(file_path):
    with open(file_path, "r", encoding="utf-8") as handle:
        tokens = handle.read().split()

    index = 0
    aircraft_count = int(tokens[index])
    index += 1

    aircraft = []
    for _ in range(aircraft_count):
        earliest = int(float(tokens[index]))
        index += 1
        preferred = int(float(tokens[index]))
        index += 1
        latest = int(float(tokens[index]))
        index += 1
        early_penalty = float(tokens[index])
        index += 1
        late_penalty = float(tokens[index])
        index += 1

        separation = [int(float(tokens[index + j])) for j in range(aircraft_count)]
        index += aircraft_count

        aircraft.append(
            {
                "E": earliest,
                "P": preferred,
                "L": latest,
                "C": early_penalty,
                "Cp": late_penalty,
                "sep": separation,
            }
        )

    return aircraft


def aircraft_cost(landing_time, plane):
    if landing_time < plane["P"]:
        return plane["C"] * (plane["P"] - landing_time)
    return plane["Cp"] * (landing_time - plane["P"])


def pair_is_compatible(i, value_i, j, value_j, aircraft):
    time_i, runway_i = value_i
    time_j, runway_j = value_j

    if runway_i != runway_j:
        return True

    if time_i == time_j:
        return False

    if time_i < time_j:
        return time_j >= time_i + aircraft[i]["sep"][j]
    return time_i >= time_j + aircraft[j]["sep"][i]


def is_consistent_with_assignment(k, candidate_value, assignment, aircraft):
    for i, value_i in assignment.items():
        if not pair_is_compatible(i, value_i, k, candidate_value, aircraft):
            return False
    return True


def build_full_domains(aircraft, runway_count):
    domains = {}
    for k, plane in enumerate(aircraft):
        values = []
        for landing_time in range(plane["E"], plane["L"] + 1):
            for runway in range(runway_count):
                values.append((landing_time, runway))
        domains[k] = values
    return domains


def build_overlap_matrix(aircraft):
    size = len(aircraft)
    overlap = [[0] * size for _ in range(size)]

    for i in range(size):
        ei = aircraft[i]["E"]
        li = aircraft[i]["L"]
        for j in range(size):
            if i == j:
                continue
            ej = aircraft[j]["E"]
            lj = aircraft[j]["L"]
            overlap_count = min(li, lj) - max(ei, ej) + 1
            overlap[i][j] = max(0, overlap_count)

    return overlap


def select_variable(unassigned, domains, overlap):
    def rank(plane_id):
        degree_score = sum(overlap[plane_id][j] for j in unassigned if j != plane_id)
        return (len(domains[plane_id]), -degree_score, plane_id)

    return min(unassigned, key=rank)


def order_values(plane_id, domain_values, assignment, aircraft, runway_count):
    occupancy = [0] * runway_count
    for _, (_, runway) in assignment.items():
        occupancy[runway] += 1

    preferred_time = aircraft[plane_id]["P"]
    plane = aircraft[plane_id]

    def rank(value):
        landing_time, runway = value
        return (
            aircraft_cost(landing_time, plane),
            abs(landing_time - preferred_time),
            occupancy[runway],
            landing_time,
            runway,
        )

    return sorted(domain_values, key=rank)


def propagate_forward(assigned_plane, assigned_value, unassigned, domains, aircraft):
    changed_domains = {}

    for j in unassigned:
        old_domain = domains[j]
        new_domain = [
            value_j
            for value_j in old_domain
            if pair_is_compatible(assigned_plane, assigned_value, j, value_j, aircraft)
        ]

        if len(new_domain) != len(old_domain):
            changed_domains[j] = old_domain
            domains[j] = new_domain

        if not domains[j]:
            return False, changed_domains

    return True, changed_domains


def restore_domains(domains, changed_domains):
    for plane_id, previous_domain in changed_domains.items():
        domains[plane_id] = previous_domain


def fc_search(
    unassigned,
    assignment,
    domains,
    aircraft,
    runway_count,
    overlap,
    partial_cost,
    state,
    start_time,
    timeout,
):
    if time.time() - start_time > timeout:
        state["timeout"] = True
        return

    if not unassigned:
        if partial_cost < state["best_cost"]:
            state["best_cost"] = partial_cost
            state["best_solution"] = dict(assignment)
        return

    if partial_cost >= state["best_cost"]:
        return

    plane_id = select_variable(unassigned, domains, overlap)
    ordered_values = order_values(
        plane_id,
        domains[plane_id],
        assignment,
        aircraft,
        runway_count,
    )

    for candidate in ordered_values:
        if state["timeout"]:
            return

        state["checks"] += 1
        if not is_consistent_with_assignment(plane_id, candidate, assignment, aircraft):
            continue

        assignment[plane_id] = candidate
        unassigned.remove(plane_id)
        state["nodes"] += 1

        ok, changed_domains = propagate_forward(
            plane_id,
            candidate,
            unassigned,
            domains,
            aircraft,
        )

        if ok:
            landing_time, _ = candidate
            next_cost = partial_cost + aircraft_cost(landing_time, aircraft[plane_id])
            fc_search(
                unassigned,
                assignment,
                domains,
                aircraft,
                runway_count,
                overlap,
                next_cost,
                state,
                start_time,
                timeout,
            )

        restore_domains(domains, changed_domains)
        unassigned.add(plane_id)
        del assignment[plane_id]


def forward_checking(file_path, runway_count=1, timeout=120):
    aircraft = load_case(file_path)
    plane_count = len(aircraft)
    domains = build_full_domains(aircraft, runway_count)
    overlap = build_overlap_matrix(aircraft)

    state = {
        "best_cost": float("inf"),
        "best_solution": None,
        "nodes": 0,
        "checks": 0,
        "timeout": False,
    }

    start_time = time.time()
    fc_search(
        unassigned=set(range(plane_count)),
        assignment={},
        domains=domains,
        aircraft=aircraft,
        runway_count=runway_count,
        overlap=overlap,
        partial_cost=0.0,
        state=state,
        start_time=start_time,
        timeout=timeout,
    )

    return (
        state["best_solution"],
        state["best_cost"],
        state["nodes"],
        state["checks"],
        time.time() - start_time,
        state["timeout"],
        aircraft,
    )


def print_result(
    case_name,
    runways,
    solution,
    total_cost,
    nodes,
    checks,
    elapsed,
    timed_out,
    aircraft,
):
    print(f"\n{'=' * 60}")
    print(f"  caso   : {case_name}  |  pistas: {runways}")
    print(
        f"  estado : {'TIMEOUT - mejor parcial encontrado' if timed_out else 'SOLUCIÓN OPTIMA'}"
    )
    print(f"{'=' * 60}")

    if solution is None:
        print("  sin solución encontrada.")
        return

    print(f"  costo total       : {total_cost:.2f}")
    print(f"  nodos explorados  : {nodes}")
    print(f"  checks realizados : {checks}")
    print(f"  tiempo CPU (seg)  : {elapsed:.4f}")
    print(f"\n  {'avion':>6} | {'T asig':>8} | {'pista':>5} | {'Pk':>8} | {'costo':>8}")
    print(f"  {'-' * 48}")

    for k in sorted(solution):
        landing_time, runway = solution[k]
        print(
            f"  {k + 1:>6} | {landing_time:>8} | {runway + 1:>5} | "
            f"{aircraft[k]['P']:>8} | {aircraft_cost(landing_time, aircraft[k]):>8.2f}"
        )


def default_case_paths():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    cases_dir = os.path.normpath(os.path.join(script_dir, "..", "cases"))
    default_cases = [os.path.join(cases_dir, f"case{i}.txt") for i in range(1, 5)]
    return default_cases, cases_dir


def resolve_case_path(raw_value, cases_dir):
    if os.path.exists(raw_value):
        return raw_value

    candidate = os.path.join(cases_dir, raw_value)
    if os.path.exists(candidate):
        return candidate

    return raw_value


if __name__ == "__main__":
    cases, cases_dir = default_case_paths()
    runways = 1
    timeout = 60

    if len(sys.argv) >= 2:
        first_arg = sys.argv[1]
        if first_arg.lower() == "all":
            cases = cases
        else:
            cases = [resolve_case_path(first_arg, cases_dir)]

    if len(sys.argv) >= 3:
        runways = int(sys.argv[2])
    if len(sys.argv) >= 4:
        timeout = int(sys.argv[3])

    for case_file in cases:
        if not os.path.exists(case_file):
            print(f"archivo no encontrado: {case_file}")
            continue

        solution, total_cost, nodes, checks, elapsed, timed_out, aircraft = (
            forward_checking(
                case_file,
                runways,
                timeout,
            )
        )
        print_result(
            os.path.basename(case_file),
            runways,
            solution,
            total_cost,
            nodes,
            checks,
            elapsed,
            timed_out,
            aircraft,
        )
