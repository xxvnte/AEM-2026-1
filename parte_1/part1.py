# python3 part1.py case1.txt 1
# python3 part1.py case2.txt 1
# python3 part1.py case3.txt 1
# python3 part1.py case4.txt 1

import sys
import time
import copy

# ─────────────────────────────────────────
# 1. LECTURA DEL CASO
# ─────────────────────────────────────────

def parse_instance(filepath):
    with open(filepath) as f:
        tokens = f.read().split()

    idx = 0
    D = int(tokens[idx]); idx += 1

    planes = []
    sep = []

    
    for k in range(D):
        E   = float(tokens[idx]); idx += 1
        P   = float(tokens[idx]); idx += 1
        L   = float(tokens[idx]); idx += 1
        c_e = float(tokens[idx]); idx += 1
        c_l = float(tokens[idx]); idx += 1
        planes.append({'E': E, 'P': P, 'L': L, 'c_early': c_e, 'c_late': c_l})

        row = []
        for j in range(D):
            row.append(float(tokens[idx])); idx += 1
        sep.append(row)

    return D, planes, sep


# ─────────────────────────────────────────
# 2. FUNCIÓN DE COSTO
# ─────────────────────────────────────────

def cost(plane, t):
    if t < plane['P']:
        return plane['c_early'] * (plane['P'] - t)
    else:
        return plane['c_late']  * (t - plane['P'])


def total_cost(planes, assignment):
    return sum(cost(planes[i], t) for i, t in assignment.items())


# ─────────────────────────────────────────
# 3. DOMINIOS
# ─────────────────────────────────────────

def build_domains(planes, granularity=5):
    
    domains = []
    for plane in planes:
        E = int(plane['E'])
        L = int(plane['L'])
        P = int(plane['P'])
        vals = list(range(E, L + 1, granularity))
        if P not in vals:
            vals.append(P)
        vals.sort(key=lambda t: abs(t - P))
        domains.append(vals)
    return domains


# ─────────────────────────────────────────
# 4. VERIFICACIÓN DE RESTRICCIONES
# ─────────────────────────────────────────

def is_consistent(i, t_i, r_i, assignment, runway_of, sep):
   
    for j, t_j in assignment.items():
        r_j = runway_of[j]
        if r_i != r_j:
            continue
        if t_i == t_j:
            return False
        if t_i > t_j and t_i < t_j + sep[j][i]:
            return False
        if t_j > t_i and t_j < t_i + sep[i][j]:
            return False
    return True


# ─────────────────────────────────────────
# 5. HEURÍSTICA DE VARIABLE: orden por Ek
# ─────────────────────────────────────────

def build_order(planes):
   
    return sorted(range(len(planes)), key=lambda k: planes[k]['E'])


# ─────────────────────────────────────────
# 6. BACKTRACKING CRONOLÓGICO CON LÍMITE DE TIEMPO
# ─────────────────────────────────────────

def backtracking(D, planes, sep, num_runways, domains, time_limit):
    best = {
        'cost': float('inf'),
        'assignment': None,
        'runway': None
    }
    nodes_explored = [0]
    time_exceeded  = [False]
    deadline = time.time() + time_limit

    variable_order = build_order(planes)
    runway_of = [None] * D

    def bt(assignment, pos, current_cost):

        
        if time.time() > deadline:
            time_exceeded[0] = True
            return

        nodes_explored[0] += 1

       
        if current_cost >= best['cost']:
            return

        
        if pos == D:
            best['cost']       = current_cost
            best['assignment'] = dict(assignment)
            best['runway']     = list(runway_of)
            return

        i = variable_order[pos]

        for t in domains[i]:

            
            if time.time() > deadline:
                time_exceeded[0] = True
                return

            c_i = cost(planes[i], t)

           
            if current_cost + c_i >= best['cost']:
                continue

            for r in range(num_runways):
                if is_consistent(i, t, r, assignment, runway_of, sep):
                    assignment[i] = t
                    runway_of[i]  = r
                    bt(assignment, pos + 1, current_cost + c_i)
                    del assignment[i]
                    runway_of[i] = None

    bt({}, 0, 0.0)

    return (
        best['assignment'],
        best['runway'],
        best['cost'],
        nodes_explored[0],
        time_exceeded[0]
    )


# ─────────────────────────────────────────
# 7. IMPRESIÓN DE RESULTADOS
# ─────────────────────────────────────────

def print_solution(D, planes, assignment, runway_of, cost_val,
                   nodes, elapsed, time_exceeded):
    print("\n" + "="*60)
    print("  RESULTADO — Backtracking Cronológico")
    print("="*60)

    if assignment is None:
        print("  No se encontró solución factible en el tiempo límite.")
        print(f"  Nodos explorados  : {nodes}")
        print(f"  Tiempo (s)        : {elapsed:.4f}")
        return

    status = " (ÓPTIMO)" if not time_exceeded else " (MEJOR ENCONTRADA — límite de tiempo alcanzado)"
    print(f"  Costo total       : {cost_val:.2f}{status}")
    print(f"  Nodos explorados  : {nodes}")
    print(f"  Tiempo (s)        : {elapsed:.4f}")
    print()
    print(f"  {'Avión':>5} {'Pista':>5} {'E':>6} {'P':>6} {'L':>6} "
          f"{'T asig':>7} {'Costo':>8}")
    print("  " + "-"*55)
    for i in range(D):
        t = assignment[i]
        r = runway_of[i]
        c = cost(planes[i], t)
        E = int(planes[i]['E'])
        P = int(planes[i]['P'])
        L = int(planes[i]['L'])
        print(f"  {i+1:>5} {r+1:>5} {E:>6} {P:>6} {L:>6} {t:>7} {c:>8.2f}")
    print("="*60)


# ─────────────────────────────────────────
# 8. MAIN
# ─────────────────────────────────────────

def main():
    filepath    = sys.argv[1] if len(sys.argv) > 1 else "case1.txt"
    num_runways = int(sys.argv[2])   if len(sys.argv) > 2 else 1
    time_limit  = float(sys.argv[3]) if len(sys.argv) > 3 else 60.0

    print(f"\nInstancia  : {filepath}")
    print(f"Pistas     : {num_runways}")
    print(f"Límite (s) : {time_limit}")

    D, planes, sep = parse_instance(filepath)
    print(f"Aviones    : {D}")

    domains = build_domains(planes, granularity=5)

    t0 = time.time()
    assignment, runway_of, best_cost, nodes, time_exceeded = backtracking(
        D, planes, sep, num_runways, copy.deepcopy(domains), time_limit
    )
    elapsed = time.time() - t0

    print_solution(D, planes, assignment, runway_of, best_cost,
                   nodes, elapsed, time_exceeded)


if __name__ == "__main__":
    main()