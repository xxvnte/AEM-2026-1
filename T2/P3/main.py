"""
==============================================================================
main.py  –  ACO para Selección de Proyectos con Costos Compartidos
Curso   : CIT3352 – Algoritmos Exactos y Metaheurísticas
Tarea 2 – Parte 3 (Algoritmo de Población: Ant Colony Optimization)
==============================================================================

DECISIONES DE DISEÑO
──────────────────────────────────────────────────────────────────────────────
1. ESTRUCTURA DE FEROMONA (nodos, no aristas)
   El problema consiste en decidir si INCLUIR o no cada alternativa i en la
   solución, sin que haya un orden secuencial obligatorio entre ellas.
   No existe una secuencia de ciudades (como TSP) ni una dependencia "de→a"
   entre proyectos, por lo que la feromona sobre ARISTAS no aporta información
   útil. En cambio, τ_i representa el "atractivo aprendido" de incluir la
   alternativa i en la solución, análogo al rastro de feromona sobre un ítem
   en el Ant-System para Knapsack. Esto reduce la dimensionalidad de O(m²) a
   O(m) y es la elección estándar en la literatura ACO para set-packing y
   knapsack (Leguizamón & Michalewicz 1999; Ke et al. 2010).

2. FUNCIÓN HEURÍSTICA η_i (DINÁMICA)
   La heurística clásica η = 1/w no aplica aquí porque el "costo" de agregar
   una alternativa depende de qué recursos ya están activados (costo marginal).
   Usamos:
       η_i = p_i / (costo_marginal_i + ε)
   donde costo_marginal_i = Σ w_j  para todo j que requiere i pero que
   ninguna alternativa ya seleccionada usa. Si el costo marginal es 0 (todos
   los recursos de i ya están pagados por la solución parcial), η_i se dispara
   → la hormiga casi seguro tomará un proyecto "gratuito" que incrementa el
   beneficio sin costo adicional. Esto imita el comportamiento óptimo: aprovechar
   la compartición de recursos.

3. HIPERPARÁMETROS JUSTIFICADOS
   ─────────────────────────────
   α = 1.0  → peso de feromona. Valor neutro que permite que la feromona guíe
              sin dominar; equilibra exploración y explotación.
   β = 2.0  → peso heurístico. Valor más alto favorece la calidad inmediata
              (ratio beneficio/costo marginal), útil en problemas de mochila
              donde la heurística voraz es fuerte. β ∈ [1,3] es canónico en
              ACO para knapsack.
   ρ = 0.1  → evaporación. Valor bajo para no olvidar buenas soluciones
              rápidamente; permite convergencia gradual. En instancias difíciles
              se reduce a 0.05 para mantener diversidad más tiempo.
   n_ants   = max(20, m//5) → escala con el tamaño del problema; suficientes
              hormigas para cubrir el espacio de soluciones pero sin costo
              computacional excesivo.
   n_iter   = 150 iteraciones base; para instancias fáciles 100 es suficiente.
   τ_min, τ_max → límites MMAS (MAX-MIN Ant System) que evitan convergencia
              prematura: ningún rastro cae a 0 ni satura en ∞.

4. INICIALIZACIÓN CON GREEDY ESTOCÁSTICO
   La feromona inicial τ_i se fija con base en cuántas de las K soluciones
   greedy estocásticas seleccionaron la alternativa i:
       τ_i = τ_min + (τ_max - τ_min) * (frecuencia_i en greedy)
   Esto "calienta" el rastro inicial hacia regiones prometedoras, equivalente a
   una búsqueda local de arranque que reduce el tiempo de convergencia.

5. ACTUALIZACIÓN DE FEROMONA (ACS-like + MMAS bounds)
   Tras cada iteración:
       τ_i ← clip( (1-ρ)·τ_i + Σ_k Δτ_i^k,  τ_min, τ_max )
   donde  Δτ_i^k = L_k / L_best_global  si la hormiga k seleccionó i, else 0.
   Depositar proporcional a L_k/L_best incentiva a las hormigas que encontraron
   mejores soluciones a dejar más rastro, análogo a "más comida → más feromona".

6. VERSIÓN EXTENDIDA — agrega:
  (A) Registro de convergencia poblacional por iteración (mejor / media / peor
      de la colonia + tiempo acumulado) -> convergence_data.csv
  (B) Barrido de parámetros (rho) con >=5 valores y >=5 repeticiones cada uno,
      análogo a la tabla de "tenure" usada para Tabu Search -> param_sweep.csv
      y agregado también al texto de results.txt

==============================================================================
"""

import os
import sys
import csv
import random
import math
import time
import statistics
from typing import List, Tuple, Dict, Set, Optional

# ─────────────────────────── CONSTANTES GLOBALES ────────────────────────────

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
CASES_DIR = os.path.join(SCRIPT_DIR, "..", "cases")
RESULTS_FILE = os.path.join(SCRIPT_DIR, "results.txt")
CONVERGENCE_FILE = os.path.join(SCRIPT_DIR, "convergence_data.csv")
SWEEP_FILE = os.path.join(SCRIPT_DIR, "param_sweep.csv")

INSTANCE_FILES = ["easy.txt", "medium1.txt", "medium2.txt", "hard.txt"]

# Semilla base para reproducibilidad (cada ejecución la desplaza levemente)
BASE_SEED = 42

# Instancias sobre las que se ejecuta el barrido de parámetros.
# ADVERTENCIA: el ACO completo es costoso (ver tiempos en tu results.txt
# original: ~13s easy, ~150-370s medium, ~330s hard POR CORRIDA). Un barrido
# de 5 valores x 5 repeticiones sobre TODAS las instancias puede tardar horas.
# Por defecto se corre solo en "easy" y "medium1"; agrega más nombres si tienes
# tiempo disponible (o reduce N_ITER_SWEEP / N_REPS_SWEEP).

# ========================================================
# SWEEP_INSTANCES = ["easy", "medium1"]
SWEEP_INSTANCES = ["easy", "medium1", "medium2", "hard"]
# ========================================================


RHO_SWEEP_VALUES = [0.03, 0.05, 0.08, 0.12, 0.18]
N_REPS_SWEEP = 5
# Si quieres un barrido más rápido (a costa de menor precisión), reduce las
# iteraciones SOLO para el barrido (no afecta las corridas normales del ACO):
SWEEP_ITER_FACTOR = 1.0  # 1.0 = mismas iteraciones que la config base; 0.5 = mitad

# ══════════════════════════════════════════════════════════════════════════════
#  1. PARSER DE INSTANCIAS
# ══════════════════════════════════════════════════════════════════════════════

class Instance:

    """
    Almacena todos los datos de una instancia del problema.
    Attributes:
        m         : número de alternativas (proyectos)
        n         : número de recursos
        ne        : número de relaciones alternativa-recurso
        B         : capacidad máxima
        benefits  : lista de beneficios p_i  (índice 0..m-1)
        weights   : lista de pesos w_j       (índice 0..n-1)
        alt_resources : dict {i: set(j)} – recursos que requiere la alternativa i
    """

    def __init__(self, m: int, n: int, ne: int, B: int,
                 benefits: List[int], weights: List[int],
                 alt_resources: Dict[int, Set[int]]):
        self.m = m
        self.n = n
        self.ne = ne
        self.B = B
        self.benefits = benefits
        self.weights = weights
        self.alt_resources = alt_resources

    @classmethod
    def from_file(cls, filepath: str) -> "Instance":

        # Lee una instancia desde el formato estandar de benchmarks
        with open(filepath, "r") as f:
            lines = [ln.strip() for ln in f if ln.strip()]

        # Línea 1: m n ne B
        header = lines[0].split()
        m, n, ne, B = int(header[0]), int(header[1]), int(header[2]), int(header[3])

        # Línea 2: beneficios
        benefits = list(map(int, lines[1].split()))
        assert len(benefits) == m, f"Se esperaban {m} beneficios, se encontraron {len(benefits)}"

        # Línea 3: pesos de recursos
        weights = list(map(int, lines[2].split()))
        assert len(weights) == n, f"Se esperaban {n} pesos, se encontraron {len(weights)}"

        # Relaciones alternativa-recurso (0-indexed)
        alt_resources: Dict[int, Set[int]] = {i: set() for i in range(m)}
        for line in lines[3:3 + ne]:
            parts = line.split()
            i, j = int(parts[0]), int(parts[1])
            alt_resources[i].add(j)

        return cls(m, n, ne, B, benefits, weights, alt_resources)

# ══════════════════════════════════════════════════════════════════════════════
#  2. EVALUACIÓN DE SOLUCIONES
# ══════════════════════════════════════════════════════════════════════════════

def evaluate_solution(selected: List[int], inst: Instance) -> Tuple[int, int, bool]:

    """
    Evalúa un subconjunto de alternativas seleccionadas.
    Retorna (beneficio_total, costo_total, es_factible).
    El costo se calcula usando UNION de recursos (costo compartido).
    """

    used_resources: Set[int] = set()
    total_benefit = 0
    for i in selected:
        used_resources |= inst.alt_resources[i]
        total_benefit += inst.benefits[i]

    total_cost = sum(inst.weights[j] for j in used_resources)
    return total_benefit, total_cost, (total_cost <= inst.B)


def marginal_cost(i: int, active_resources: Set[int], inst: Instance) -> int:

    """
    Costo marginal de agregar la alternativa i dado que 'active_resources'
    ya está activado en la solución parcial.
    = suma de pesos de recursos nuevos que i aportaría.
    """

    new_res = inst.alt_resources[i] - active_resources
    return sum(inst.weights[j] for j in new_res)

# ══════════════════════════════════════════════════════════════════════════════
#  3. GREEDY DETERMINISTA
# ══════════════════════════════════════════════════════════════════════════════

def greedy_deterministic(inst: Instance) -> List[int]:

    """
    Greedy determinista: ordena alternativas por ratio beneficio/costo_inicial
    y las incorpora en orden decreciente siempre que no excedan la capacidad.
    Nota: el costo se recalcula de forma marginal a medida que se construye la
    solución, para aprovechar la compartición de recursos.
    """

    # Ranking inicial por p_i / (costo_propio_i + ε)
    def init_ratio(i):
        c = sum(inst.weights[j] for j in inst.alt_resources[i])
        return inst.benefits[i] / (c + 1e-9)

    candidates = sorted(range(inst.m), key=init_ratio, reverse=True)

    selected: List[int] = []
    active_res: Set[int] = set()
    current_cost = 0

    for i in candidates:
        mc = marginal_cost(i, active_res, inst)
        if current_cost + mc <= inst.B:
            selected.append(i)
            active_res |= inst.alt_resources[i]
            current_cost += mc

    return selected

# ══════════════════════════════════════════════════════════════════════════════
#  4. GREEDY ESTOCÁSTICO
# ══════════════════════════════════════════════════════════════════════════════

def greedy_stochastic(inst: Instance, rng: random.Random,
                      alpha_g: float = 0.3) -> List[int]:
    
    """
    Greedy estocástico tipo GRASP: construye una Lista Restringida de
    Candidatos (RCL) y elige al azar de ella.
    alpha_g ∈ [0,1]: 0 = completamente greedy, 1 = completamente aleatorio.
    """

    selected: List[int] = []
    active_res: Set[int] = set()
    current_cost = 0
    remaining = set(range(inst.m))

    # Calcular ratio beneficio/costo_marginal para cada candidato restante
    while remaining:
        feasible = []
        for i in remaining:
            mc = marginal_cost(i, active_res, inst)
            if current_cost + mc <= inst.B:
                ratio = inst.benefits[i] / (mc + 1e-9)
                feasible.append((i, ratio))

        if not feasible:
            break

        # Construir RCL: alternativas cuyo ratio está en [r_max - α*(r_max-r_min), r_max]
        r_max = max(r for _, r in feasible)
        r_min = min(r for _, r in feasible)
        threshold = r_max - alpha_g * (r_max - r_min)
        rcl = [i for i, r in feasible if r >= threshold]

        # Elegir aleatoriamente de la RCL
        chosen = rng.choice(rcl)
        mc = marginal_cost(chosen, active_res, inst)
        selected.append(chosen)
        active_res |= inst.alt_resources[chosen]
        current_cost += mc
        remaining.discard(chosen)

    return selected

# ══════════════════════════════════════════════════════════════════════════════
#  5. ALGORITMO ACO
# ══════════════════════════════════════════════════════════════════════════════

class ACO:

    """
    Ant Colony Optimization para maximización con costos compartidos.

    Parámetros
    ──────────
    alpha   : importancia de la feromona τ  (default 1.0)
    beta    : importancia de la heurística η (default 2.0)
    rho     : tasa de evaporación ρ          (default 0.1)
    n_ants  : número de hormigas por iteración
    n_iter  : número de iteraciones del ciclo principal
    tau_min : límite inferior de feromona (MMAS)
    tau_max : límite superior de feromona (MMAS)
    K_greedy: soluciones greedy estocásticas para inicializar feromona
    seed    : semilla para reproducibilidad
    """

    # ─── hiperparámetros por tipo de instancia ────────────────────────────
    CONFIGS = {
        "easy":    dict(alpha=1.0, beta=2.0, rho=0.10, n_ants=20,  n_iter=100),
        "medium1": dict(alpha=1.0, beta=2.0, rho=0.08, n_ants=30,  n_iter=150),
        "medium2": dict(alpha=1.0, beta=2.0, rho=0.08, n_ants=30,  n_iter=150),
        "hard":    dict(alpha=1.0, beta=1.5, rho=0.05, n_ants=40,  n_iter=200),
    }

    def __init__(self, inst: Instance, config_name: str = "medium1",
                 seed: int = BASE_SEED, K_greedy: int = 20,
                 verbose: bool = False, param_overrides: Optional[Dict] = None,
                 iter_factor: float = 1.0):
        self.inst = inst
        cfg = dict(ACO.CONFIGS.get(config_name, ACO.CONFIGS["medium1"]))
        if param_overrides:
            cfg.update(param_overrides)

        self.alpha   = cfg["alpha"]
        self.beta    = cfg["beta"]
        self.rho     = cfg["rho"]
        self.n_ants  = cfg["n_ants"]
        self.n_iter  = max(1, int(cfg["n_iter"] * iter_factor))
        self.K_greedy = K_greedy
        self.verbose  = verbose
        self.rng = random.Random(seed)

        # Límites MMAS para evitar convergencia prematura
        self.tau_max = 1.0
        self.tau_min = 0.01

        # Vector de feromonas τ_i para cada alternativa i (nodo, no arista)
        self.tau: List[float] = [self.tau_min] * inst.m

        # Mejor solución global
        self.best_solution: List[int] = []
        self.best_benefit: int = 0

        # Historial de convergencia poblacional: lista de dicts por iteración
        self.history: List[Dict] = []

    # ── 5.1 Inicialización de feromona con greedy estocástico ─────────────

    def _init_pheromone_from_greedy(self):

        """
        Ejecuta K_greedy soluciones greedy estocásticas y fija la feromona
        inicial de cada alternativa proporcional a su frecuencia de aparición.
        """
        freq = [0] * self.inst.m
        best_b = 0
        best_sol: List[int] = []

        for k in range(self.K_greedy):
            sol = greedy_stochastic(self.inst, self.rng)
            benefit, cost, feasible = evaluate_solution(sol, self.inst)
            if feasible and benefit > best_b:
                best_b = benefit
                best_sol = sol[:]
            for i in sol:
                freq[i] += 1

        # Actualizar mejor solución con el greedy
        if best_b > self.best_benefit:
            self.best_benefit = best_b
            self.best_solution = best_sol[:]

        # Feromona proporcional a la frecuencia normalizada
        max_freq = max(freq) if max(freq) > 0 else 1
        for i in range(self.inst.m):
            normalized = freq[i] / max_freq
            self.tau[i] = self.tau_min + (self.tau_max - self.tau_min) * normalized

        if self.verbose:
            print(f"  [Greedy Init] Mejor greedy estocástico: {best_b}")


    # ── 5.2 Construcción de solución por una hormiga ──────────────────────

    def _construct_solution(self) -> List[int]:

        """
        Una hormiga construye una solución seleccionando alternativas de forma
        probabilística según τ^α · η^β.

        Orden de evaluación: aleatoriamente permutado para evitar sesgo posicional.

        La selección se hace por ruleta (roulette wheel) sobre los candidatos
        factibles, recalculando η dinámicamente según los recursos ya activos.
        """

        selected: List[int] = []
        active_res: Set[int] = set()
        current_cost: int = 0

        # Permutación aleatoria de candidatos para orden no determinista
        candidates = list(range(self.inst.m))
        self.rng.shuffle(candidates)

        # Construcción greedy-probabilística
        remaining = set(candidates)

        while remaining:
            # Filtrar candidatos factibles (no exceden capacidad)
            feasible_with_scores = []
            for i in remaining:
                mc = marginal_cost(i, active_res, self.inst)
                if current_cost + mc <= self.inst.B:
                    # η dinámica: beneficio / (costo_marginal + ε)
                    eta = self.inst.benefits[i] / (mc + 1e-9)
                    score = (self.tau[i] ** self.alpha) * (eta ** self.beta)
                    feasible_with_scores.append((i, mc, score))

            if not feasible_with_scores:
                break

            # Selección por ruleta
            total = sum(s for _, _, s in feasible_with_scores)
            if total == 0:
                # Fallback: elección uniforme entre factibles
                chosen_idx = self.rng.randrange(len(feasible_with_scores))
                chosen_i, chosen_mc, _ = feasible_with_scores[chosen_idx]
            else:
                r = self.rng.random() * total
                cumulative = 0.0
                chosen_i, chosen_mc = feasible_with_scores[-1][0], feasible_with_scores[-1][1]
                for i, mc, s in feasible_with_scores:
                    cumulative += s
                    if cumulative >= r:
                        chosen_i, chosen_mc = i, mc
                        break

            # Agregar a la solución
            selected.append(chosen_i)
            active_res |= self.inst.alt_resources[chosen_i]
            current_cost += chosen_mc
            remaining.discard(chosen_i)

            # Optimización: si capacidad residual < mínimo peso de recurso,
            # no podemos añadir nada con costo nuevo → salir temprano
            residual = self.inst.B - current_cost
            if residual <= 0:
                break

        return selected


    # ── 5.3 Actualización de feromona ─────────────────────────────────────

    def _update_pheromone(self, ant_solutions: List[Tuple[List[int], int]]):

        """
        Actualiza la feromona global con evaporación y refuerzo.

        Δτ_i^k = L_k / L_best_global  si la hormiga k seleccionó i
                 0                      en caso contrario

        Escalar por L_best evita que Δτ sea demasiado grande cuando L_best
        crece con la convergencia, manteniendo τ en rango estable.
        """

        # Evaporacion
        for i in range(self.inst.m):
            self.tau[i] *= (1.0 - self.rho)

        # Deposito proporcional a la calidad
        denom = self.best_benefit if self.best_benefit > 0 else 1
        for sol, benefit in ant_solutions:
            if benefit == 0:
                continue
            delta = benefit / denom
            for i in sol:
                self.tau[i] += delta

        # Aplico limites τ_i a [τ_min, τ_max] (MMAS)
        for i in range(self.inst.m):
            self.tau[i] = max(self.tau_min, min(self.tau_max, self.tau[i]))


    # ── 5.4 Búsqueda local (2-opt simplificado) ──────────────────────────

    def _local_search(self, sol: List[int]) -> List[int]:

        """
        Mejora local: intenta intercambiar alternativas seleccionadas por no
        seleccionadas si mejora el beneficio sin violar la restricción.
        Solo se hace un pasada para mantener la eficiencia.
        """

        selected_set = set(sol)
        not_selected = [i for i in range(self.inst.m) if i not in selected_set]

        current_benefit, _, _ = evaluate_solution(sol, self.inst)

        improved = True
        while improved:
            improved = False
            for out_i in list(selected_set):
                for in_i in not_selected:
                    new_sol_set = (selected_set - {out_i}) | {in_i}
                    new_sol = list(new_sol_set)
                    new_b, new_c, feasible = evaluate_solution(new_sol, self.inst)
                    if feasible and new_b > current_benefit:
                        selected_set = new_sol_set
                        not_selected = [i for i in range(self.inst.m)
                                        if i not in selected_set]
                        current_benefit = new_b
                        improved = True
                        break
                if improved:
                    break

        return list(selected_set)


    # ── 5.5 Ciclo principal ACO ───────────────────────────────────────────

    def run(self) -> Tuple[List[int], int]:
        """
        Ejecuta el algoritmo ACO completo.
        Además de retornar (mejor_solución, mejor_beneficio), llena
        self.history con, por cada iteración:
            iteration, best_global, iter_best, iter_mean, iter_worst, time_cum
        donde iter_mean / iter_worst se calculan sobre TODAS las soluciones
        factibles construidas por la colonia en esa iteración (incluida la
        mejorada por búsqueda local si aplica), es decir, la convergencia
        POBLACIONAL real (no solo la trayectoria del mejor global).
        """

        # Paso 1: inicializar feromona con greedy estocástico
        self._init_pheromone_from_greedy()
        self.history = [] # Historial

        t_start = time.perf_counter()
        no_improve_count = 0
        MAX_NO_IMPROVE = 30 # criterio de parada anticipada

        for iteration in range(self.n_iter):
            ant_solutions: List[Tuple[List[int], int]] = []
            iter_benefits: List[int] = []
            iter_best_b = 0
            iter_best_sol: List[int] = []

            # Paso 2: cada hormiga construye una solución
            for _ in range(self.n_ants):
                sol = self._construct_solution()
                benefit, _, feasible = evaluate_solution(sol, self.inst)

                if feasible:
                    ant_solutions.append((sol, benefit))
                    iter_benefits.append(benefit)

                    if benefit > iter_best_b:
                        iter_best_b = benefit
                        iter_best_sol = sol[:]

                    # Actualizar mejor global
                    if benefit > self.best_benefit:
                        self.best_benefit = benefit
                        self.best_solution = sol[:]
                        no_improve_count = 0

            # Paso 3: búsqueda local solo en la mejor de la iteración
            if iter_best_sol:
                improved = self._local_search(iter_best_sol)
                imp_b, _, _ = evaluate_solution(improved, self.inst)
                iter_benefits.append(imp_b)
                if imp_b > self.best_benefit:
                    self.best_benefit = imp_b
                    self.best_solution = improved[:]
                    no_improve_count = 0
                # Reemplazar la hormiga si mejoró
                if imp_b > iter_best_b:
                    ant_solutions.append((improved, imp_b))
                    iter_best_b = imp_b

            # Paso 4: actualizar feromonas
            if ant_solutions:
                self._update_pheromone(ant_solutions)

            # ── Registro de convergencia poblacional de esta iteración ──
            if iter_benefits:
                iter_mean = statistics.mean(iter_benefits)
                iter_worst = min(iter_benefits)
            else:
                iter_mean = self.best_benefit
                iter_worst = self.best_benefit

            t_cum = time.perf_counter() - t_start
            self.history.append({
                "iteration": iteration + 1,
                "best_global": self.best_benefit,
                "iter_best": iter_best_b,
                "iter_mean": iter_mean,
                "iter_worst": iter_worst,
                "time_cum": t_cum,
            })

            no_improve_count += 1
            if self.verbose and (iteration + 1) % 25 == 0:
                print(f"    Iter {iteration+1:4d}/{self.n_iter} | "
                      f"Iter best: {iter_best_b} | Global best: {self.best_benefit}")

            # Parada anticipada
            if no_improve_count >= MAX_NO_IMPROVE:
                if self.verbose:
                    print(f"    Convergencia temprana en iteración {iteration+1}")
                break

        return self.best_solution, self.best_benefit

# ══════════════════════════════════════════════════════════════════════════════
#  6. RUNNER DE EXPERIMENTOS
# ══════════════════════════════════════════════════════════════════════════════

def run_experiments(inst: Instance, inst_name: str, n_runs: int = 10
                     ) -> Tuple[Dict, str, List[Dict]]:
    """
    Retorna una lista de filas de convergencia (una por iteración 
    por corrida) para exportar a CSV.
    """
    config_name = inst_name.replace(".txt", "")
    cfg = ACO.CONFIGS.get(config_name, ACO.CONFIGS["medium1"])

    lines = []
    def log(text=""):
        print(text)
        lines.append(text)

    log("=================================================================")
    log(f"  ACO - Instancia: {config_name}")
    log("=================================================================")
    log(f"  m={inst.m}, n={inst.n}, B={inst.B}")
    log(f"  Parámetros ACO: alpha={cfg['alpha']}, beta={cfg['beta']}, rho={cfg['rho']:.2f}, n_ants={cfg['n_ants']}, max_iter={cfg['n_iter']}")
    log()

    # Greedy Determinista
    t0 = time.perf_counter()
    gd_sol = greedy_deterministic(inst)
    gd_time = time.perf_counter() - t0
    gd_b, gd_c, _ = evaluate_solution(gd_sol, inst)

    log("  [Greedy Determinista]")
    log(f"    Beneficio : {gd_b}")
    log(f"    Costo     : {gd_c}  (cap={inst.B})")
    log(f"    |Sol|     : {len(gd_sol)}")
    log(f"    Tiempo    : {gd_time:.4f}s")
    log()

    # Greedy Estocástico (10 corridas)
    sto_benefits = []
    sto_costs = []
    sto_times = []
    for run_id in range(n_runs):
        seed = BASE_SEED + run_id * 7
        rng = random.Random(seed)
        t0 = time.perf_counter()
        sol = greedy_stochastic(inst, rng, alpha_g=0.3)
        elapsed = time.perf_counter() - t0
        benefit, cost, _ = evaluate_solution(sol, inst)
        sto_benefits.append(benefit)
        sto_costs.append(cost)
        sto_times.append(elapsed)

    log("  [Greedy Estocástico (10 corridas)]")
    for idx, (b, c, t) in enumerate(zip(sto_benefits, sto_costs, sto_times), 1):
        log(f"    Run {idx:2d}: beneficio=  {b:<5d}  costo=  {c:<5d}  t={t:.4f}s")
    log()
    mean_sto = statistics.mean(sto_benefits)
    std_sto = statistics.stdev(sto_benefits) if len(sto_benefits) > 1 else 0
    best_sto = max(sto_benefits)
    worst_sto = min(sto_benefits)
    median_sto = statistics.median(sto_benefits)
    avg_time_sto = statistics.mean(sto_times)
    log("  Estadísticas Greedy Estocástico:")
    log(f"    Media     : {mean_sto:.2f}")
    log(f"    Desv. Est.: {std_sto:.2f}")
    log(f"    Mejor     : {best_sto}")
    log(f"    Peor      : {worst_sto}")
    log(f"    Mediana   : {median_sto:.1f}")
    log(f"    Tiempo prom: {avg_time_sto:.4f}s")
    log()

    # ACO Runs
    log(f"  [ACO x{n_runs} corridas independientes]")
    aco_benefits = []
    aco_times = []
    aco_costs = []
    convergence_rows: List[Dict] = []
    best_run_id = -1
    best_run_benefit = -1

    for run_id in range(n_runs):
        seed = BASE_SEED + run_id * 7
        aco = ACO(inst, config_name=config_name, seed=seed, K_greedy=20, verbose=False)
        t0 = time.perf_counter()
        sol, benefit = aco.run()
        elapsed = time.perf_counter() - t0

        _, cost, _ = evaluate_solution(sol, inst)
        aco_benefits.append(benefit)
        aco_costs.append(cost)
        aco_times.append(elapsed)

        if benefit > best_run_benefit:
            best_run_benefit = benefit
            best_run_id = run_id + 1

        # Guardar historial de convergencia de ESTA corrida
        for row in aco.history:
            convergence_rows.append({
                "instance": config_name,
                "run": run_id + 1,
                "iteration": row["iteration"],
                "best_global": row["best_global"],
                "iter_best": row["iter_best"],
                "iter_mean": row["iter_mean"],
                "iter_worst": row["iter_worst"],
                "time_cum": row["time_cum"],
            })

        log(f"    Run {run_id+1:2d}: beneficio=  {benefit:<5d}  costo=  {cost:<5d}  t={elapsed:.4f}s")

    log()
    mean_b = statistics.mean(aco_benefits)
    std_b = statistics.stdev(aco_benefits) if len(aco_benefits) > 1 else 0
    best_b = max(aco_benefits)
    worst_b = min(aco_benefits)
    median_b = statistics.median(aco_benefits)
    avg_time_aco = statistics.mean(aco_times)

    log("  Estadísticas ACO:")
    log(f"    Media     : {mean_b:.2f}")
    log(f"    Desv. Est.: {std_b:.2f}")
    log(f"    Mejor     : {best_b}")
    log(f"    Peor      : {worst_b}")
    log(f"    Mediana   : {median_b:.1f}")
    log(f"    Corrida con mejor resultado: Run {best_run_id} (usada como referencia")
    log(f"      para los gráficos de convergencia poblacional y FO vs tiempo)")
    log()

    # COMPARACION DE RESULTADOS

    log("  Comparativa:")
    log(f"    Greedy Det.        : {gd_b}")
    log(f"    Greedy Sto. (mejor): {best_sto}")
    log(f"    Greedy Sto. (media): {mean_sto:.2f}")
    log(f"    ACO (mejor)        : {best_b}")
    log(f"    ACO (media)        : {mean_b:.2f}")
    if gd_b > 0:
        gap_aco_vs_det = ((best_b - gd_b) / gd_b) * 100
        log(f"    Gap ACO vs Det.   : {gap_aco_vs_det:+.2f}%")
    if best_sto > 0:
        gap_aco_vs_sto = ((best_b - best_sto) / best_sto) * 100
        log(f"    Gap ACO vs Sto.   : {gap_aco_vs_sto:+.2f}%")
    log()
    total_time = gd_time + sum(sto_times) + sum(aco_times)
    log(f"  Tiempo total instancia: {total_time:.4f}s")
    log()

    stats = {
        "instance": config_name,
        "det": gd_b,
        "sto_best": best_sto,
        "sto_mean": mean_sto,
        "sto_std": std_sto,
        "aco_best": best_b,
        "aco_mean": mean_b,
        "aco_std": std_b,
        "aco_worst": worst_b,
        "aco_t_avg": avg_time_aco,
        "aco_runs": n_runs,
        "best_run_id": best_run_id,
    }

    return stats, "\n".join(lines), convergence_rows

# ══════════════════════════════════════════════════════════════════════════════
#  6B. BARRIDO DE PARÁMETROS (justificación empírica, análogo a tabla de tenure)
# ══════════════════════════════════════════════════════════════════════════════

def run_parameter_sweep(inst: Instance, instance_name: str,
                         rho_values: List[float] = None,
                         n_reps: int = N_REPS_SWEEP,
                         iter_factor: float = SWEEP_ITER_FACTOR
                         ) -> Tuple[List[Dict], str]:
    """
    Corre el ACO variando rho (tasa de evaporación) sobre >=5 valores con
    >=5 repeticiones cada uno, registrando mejor/media/desv.est. por valor.
    Retorna las filas crudas (para CSV) y el texto de tabla (para results.txt)
    """
    if rho_values is None:
        rho_values = RHO_SWEEP_VALUES

    config_name = instance_name.replace(".txt", "")
    rows: List[Dict] = []
    lines = []
    lines.append("")
    lines.append("=" * 70)
    lines.append(f"  CALIBRACIÓN DE PARÁMETROS (rho) - Instancia: {config_name}")
    lines.append(f"  ({n_reps} repeticiones por valor, resto de parámetros fijos según CONFIGS)")
    lines.append("=" * 70)
    lines.append(f"  {'rho':>6} | {'Mejor':>8} | {'Media':>10} | {'Desv.Est':>9} | {'Peor':>8}")
    lines.append("  " + "-" * 52)

    for rho_val in rho_values:
        benefits = []
        for rep in range(n_reps):
            seed = BASE_SEED + rep * 13 + int(rho_val * 1000)
            aco = ACO(inst, config_name=config_name, seed=seed, K_greedy=10,
                      verbose=False, param_overrides={"rho": rho_val},
                      iter_factor=iter_factor)
            _, benefit = aco.run()
            benefits.append(benefit)
            rows.append({
                "instance": config_name,
                "rho": rho_val,
                "rep": rep + 1,
                "benefit": benefit,
            })
            print(f"    [Sweep rho={rho_val:.2f}] rep {rep+1}/{n_reps} -> beneficio={benefit}")

        mean_b = statistics.mean(benefits)
        std_b = statistics.stdev(benefits) if len(benefits) > 1 else 0
        best_b = max(benefits)
        worst_b = min(benefits)
        lines.append(f"  {rho_val:>6.2f} | {best_b:>8} | {mean_b:>10.2f} | {std_b:>9.2f} | {worst_b:>8}")

    base_rho = ACO.CONFIGS.get(config_name, ACO.CONFIGS["medium1"])["rho"]
    lines.append("")
    lines.append(f"  Valor utilizado en la configuración final: rho = {base_rho}")
    lines.append("=" * 70)

    return rows, "\n".join(lines)

# ══════════════════════════════════════════════════════════════════════════════
#  7. EXPORTACIÓN A CSV
# ══════════════════════════════════════════════════════════════════════════════

def write_convergence_csv(rows: List[Dict], output_path: str):
    if not rows:
        return
    fieldnames = ["instance", "run", "iteration", "best_global",
                  "iter_best", "iter_mean", "iter_worst", "time_cum"]
    with open(output_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    print(f"  → Datos de convergencia escritos en: {output_path}")


def write_sweep_csv(rows: List[Dict], output_path: str):
    if not rows:
        return
    fieldnames = ["instance", "rho", "rep", "benefit"]
    with open(output_path, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
    print(f"  → Datos de barrido de parámetros escritos en: {output_path}")

# ══════════════════════════════════════════════════════════════════════════════
#  8. PUNTO DE ENTRADA
# ══════════════════════════════════════════════════════════════════════════════

def main():
    all_stats = []
    all_logs = []
    all_convergence_rows: List[Dict] = []
    all_sweep_rows: List[Dict] = []
    sweep_texts: List[str] = []
    random.seed(BASE_SEED)

    for filename in INSTANCE_FILES:
        filepath = os.path.join(CASES_DIR, filename)
        if not os.path.isfile(filepath):
            print(f"  [ADVERTENCIA] No se encontró: {filepath}  – se omite.")
            continue

        inst = Instance.from_file(filepath)
        stats, log_text, conv_rows = run_experiments(inst, filename, n_runs=10)
        all_stats.append(stats)
        all_logs.append(log_text)
        all_convergence_rows.extend(conv_rows)

        config_name = filename.replace(".txt", "")
        if config_name in SWEEP_INSTANCES:
            sweep_rows, sweep_text = run_parameter_sweep(inst, filename)
            all_sweep_rows.extend(sweep_rows)
            sweep_texts.append(sweep_text)

    if not all_stats:
        print("\n  [ERROR] No se procesó ninguna instancia. Verifica la carpeta 'cases'.")
        return

    summary_lines = []
    summary_lines.append("===========================================================================")
    summary_lines.append("  RESUMEN GLOBAL - ACO")
    summary_lines.append("===========================================================================")
    summary_lines.append("  Inst.       Det.Ini   Sto.Mejor  Sto.Media  ACO.Best  ACO.Mean  ACO.Std")
    summary_lines.append("  ----------------------------------------------------------------------")
    for s in all_stats:
        summary_lines.append(
            f"  {s['instance']:<12} {s['det']:>7}   {s['sto_best']:>8}  {s['sto_mean']:>8.1f}  {s['aco_best']:>8}  {s['aco_mean']:>8.1f}  {s['aco_std']:>7.1f}"
        )
    summary_lines.append("")

    summary_text = "\n".join(summary_lines)
    print(summary_text)

    with open(RESULTS_FILE, "w", encoding="utf-8") as f:
        for log_text in all_logs:
            f.write(log_text + "\n")
        f.write(summary_text)
        if sweep_texts:
            f.write("\n\n")
            f.write("\n\n".join(sweep_texts))
            f.write("\n")

    print(f"  → Resultados escritos en: {RESULTS_FILE}")

    write_convergence_csv(all_convergence_rows, CONVERGENCE_FILE)
    write_sweep_csv(all_sweep_rows, SWEEP_FILE)

if __name__ == "__main__":
    main()