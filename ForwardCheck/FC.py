import sys
import time

# --- Constantes de Color ANSI ---
RESET = "\033[0m"
VERDE = "\033[92m"       # Para resultados finales y mejores soluciones
AMARILLO = "\033[93m"    # Para asignaciones temporales (branching)
ROJO = "\033[91m"        # Para podas (Branch & Bound) y fallos (Forward Checking)
CIAN = "\033[96m"        # Para información general de los nodos
MAGENTA = "\033[95m"     # Para la selección de variable (MRV)

class ALPSolver:
    def __init__(self, filename, debug=False):
        self.filename = filename
        self.debug = debug
        self.D = 0
        self.E = []
        self.L = []
        self.P = []
        self.C_early = []
        self.C_late = []
        self.tau = []
        
        self.best_cost = float('inf')
        self.best_solution = {}
        self.nodes_explored = 0
        
    def leer_instancia(self):
        """Lee el archivo de texto y parsea los datos del problema."""
        try:
            with open(self.filename, 'r') as f:
                tokens = f.read().split()
        except FileNotFoundError:
            print(f"{ROJO}Error: No se encontró el archivo '{self.filename}'{RESET}")
            sys.exit(1)
            
        if not tokens:
            print(f"{ROJO}Error: Archivo vacío.{RESET}")
            sys.exit(1)
            
        self.D = int(tokens[0])
        idx = 1
        
        for k in range(self.D):
            self.E.append(int(tokens[idx]))
            self.P.append(int(tokens[idx+1]))  # Tiempo Preferente
            self.L.append(int(tokens[idx+2]))  # Tiempo Tardío
            self.C_early.append(float(tokens[idx+3]))
            self.C_late.append(float(tokens[idx+4]))
            idx += 5
            
            row = []
            for _ in range(self.D):
                row.append(int(tokens[idx]))
                idx += 1
            self.tau.append(row)

    def calc_penalty(self, k, t):
        """Calcula la penalización del avión k aterrizando en el tiempo t."""
        if t < self.P[k]:
            return self.C_early[k] * (self.P[k] - t)
        elif t > self.P[k]:
            return self.C_late[k] * (t - self.P[k])
        return 0.0

    def forward_checking(self, assigned, unassigned, current_penalty, R_domain, T_domain):
        """
        Función recursiva de Backtracking con Forward Checking y Branch & Bound.
        """
        self.nodes_explored += 1
        
        # Nivel de profundidad para la indentación de los logs
        indent = "  " * len(assigned)
        
        if self.debug:
            print(f"{indent}{CIAN}[Nodo {self.nodes_explored}] Profundidad: {len(assigned)} | Aviones restantes: {len(unassigned)}{RESET}")
        
        # Caso base: Todos los aviones asignados
        if not unassigned:
            if current_penalty < self.best_cost:
                self.best_cost = current_penalty
                self.best_solution = assigned.copy()
                if self.debug:
                    print(f"{indent}{VERDE}★ ¡NUEVA MEJOR SOLUCIÓN ENCONTRADA! Costo actual: {current_penalty:.2f}{RESET}")
            return

        # Branch & Bound: Cálculo del Límite Inferior (Lower Bound)
        lb = current_penalty
        for j in unassigned:
            min_p = float('inf')
            valid_times = set()
            for r in R_domain[j]:
                valid_times.update(T_domain[j][r])
            
            for tj in valid_times:
                p = self.calc_penalty(j, tj)
                if p < min_p:
                    min_p = p
            lb += min_p
            
        # Poda por cota
        if lb >= self.best_cost:
            if self.debug:
                print(f"{indent}{ROJO}✂ Poda B&B: Límite inferior ({lb:.2f}) >= Mejor Costo ({self.best_cost:.2f}){RESET}")
            return

        # Heurística de Selección de Variable (MRV)
        best_k = None
        min_size = float('inf')
        for j in unassigned:
            size = sum(len(T_domain[j][r]) for r in R_domain[j])
            if size < min_size:
                min_size = size
                best_k = j
            elif size == min_size:
                if best_k is None or j < best_k:
                    best_k = j

        if self.debug:
            print(f"{indent}{MAGENTA}► Seleccionado Avión {best_k + 1} (MRV: {min_size} opciones){RESET}")

        # Heurística de Selección de Valores
        values = []
        for r in R_domain[best_k]:
            for t in T_domain[best_k][r]:
                values.append((r, t))
                
        values.sort(key=lambda x: (abs(x[1] - self.P[best_k]), x[1], x[0]))

        unassigned.remove(best_k)
        
        # Branching
        for r, t in values:
            if self.debug:
                print(f"{indent}  {AMARILLO}Probando: Avión {best_k + 1} -> Pista {r}, Tiempo {t}{RESET}")
            
            new_R = {j: R_domain[j].copy() for j in unassigned}
            new_T = {j: {r_j: T_domain[j][r_j].copy() for r_j in R_domain[j]} for j in unassigned}
            
            # Forward Checking
            fail = False
            failed_plane = None
            
            for j in unassigned:
                if r in new_R[j]:
                    valid_times = []
                    for tj in new_T[j][r]:
                        if tj <= t - self.tau[j][best_k] or tj >= t + self.tau[best_k][j]:
                            valid_times.append(tj)
                            
                    new_T[j][r] = valid_times
                    
                    if not valid_times:
                        new_R[j].remove(r)
                        if not new_R[j]:
                            fail = True
                            failed_plane = j + 1
                            break
                            
            if not fail:
                assigned[best_k] = (r, t)
                penalty_k = self.calc_penalty(best_k, t)
                self.forward_checking(assigned, unassigned, current_penalty + penalty_k, new_R, new_T)
                del assigned[best_k]
            else:
                if self.debug:
                    print(f"{indent}    {ROJO}✗ Forward Checking falló: El Avión {failed_plane} se quedó sin dominios válidos.{RESET}")
                
        # Restaurar al hacer backtrack
        unassigned.add(best_k)

    def resolver(self):
        """Inicializa el problema y llama al algoritmo de búsqueda."""
        self.leer_instancia()
        
        R_domain_init = {}
        T_domain_init = {}
        unassigned = set(range(self.D))
        
        for k in range(self.D):
            R_domain_init[k] = [1, 2, 3]
            T_domain_init[k] = {}
            times = list(range(self.E[k], self.L[k] + 1))
            for r in R_domain_init[k]:
                T_domain_init[k][r] = times.copy()

        assigned_init = {}
        
        start_cpu = time.process_time()
        self.forward_checking(assigned_init, unassigned, 0.0, R_domain_init, T_domain_init)
        end_cpu = time.process_time()
        
        self.mostrar_resultados(end_cpu - start_cpu)

    def mostrar_resultados(self, cpu_time):
        """Imprime la solución óptima y las métricas en color verde."""
        print(f"\n{VERDE}--- Resultados para {self.filename} ---")
        if not self.best_solution:
            print(f"{ROJO}No hay solución factible.{RESET}")
        else:
            print(f"{'Avión':<8} {'Tiempo (T_k)':<15} {'Pista (R_k)':<15} {'Penalización'}")
            print("-" * 55)
            for k in range(self.D):
                r, t = self.best_solution[k]
                penalty = self.calc_penalty(k, t)
                print(f"{k+1:<8} {t:<15} {r:<15} {penalty:.2f}")
                
            print("-" * 55)
            print(f"Coste total óptimo : {self.best_cost:.2f}")
            
        print(f"Nodos explorados   : {self.nodes_explored}")
        print(f"Tiempo de CPU      : {cpu_time:.4f} segundos{RESET}\n")


if __name__ == "__main__":
    # --- CONFIGURACIÓN ---
    # Cambia esto a True para ver el árbol de decisiones con colores. 
    # ADVERTENCIA: En casos grandes (como case1 con 21 mil nodos), 
    # la consola se ralentizará significativamente si está en True.
    DEBUG_MODE = False 
    
    #archivo_entrada = "case1.txt"
    archivo_entrada = "Casos/case2.txt"
    if len(sys.argv) > 1:
        archivo_entrada = sys.argv[1]
    
    if len(sys.argv) > 2 and sys.argv[2] == "--debug":
        DEBUG_MODE = True
        
    solucionador = ALPSolver(archivo_entrada, debug=DEBUG_MODE)
    solucionador.resolver()