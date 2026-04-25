import sys
import copy
import time

def parse_instance(filepath):
    """Parsea el archivo de texto y retorna los parámetros del problema."""
    with open(filepath, 'r') as f:
        lines = f.read().split()
    
    if not lines:
        return None
        
    n = int(lines[0])
    idx = 1
    
    E, P, L, C_early, C_late = [], [], [], [], []
    sep_matrix = []
    
    for i in range(n):
        E.append(int(lines[idx]))
        P.append(int(lines[idx+1]))
        L.append(int(lines[idx+2]))
        C_early.append(float(lines[idx+3]))
        C_late.append(float(lines[idx+4]))
        idx += 5
        
        row = []
        for j in range(n):
            row.append(int(lines[idx]))
            idx += 1
        sep_matrix.append(row)
        
    return n, E, P, L, C_early, C_late, sep_matrix

def calculate_penalty(t, p, c_early, c_late):
    """Calcula la penalización por aterrizar fuera del tiempo preferente."""
    if t < p:
        return (p - t) * c_early
    elif t > p:
        return (t - p) * c_late
    return 0.0

def solve_alp_fc(n, num_runways, E, P, L, C_early, C_late, sep_matrix):
    runways = list(range(1, num_runways + 1))
    
    # 1. Inicializar dominios: dict[avión] -> dict[pista] -> set(tiempos_validos)
    domains = {}
    for i in range(n):
        domains[i] = {}
        for r in runways:
            domains[i][r] = set(range(E[i], L[i] + 1))
    
    best_cost = float('inf')
    best_solution = None
    nodes_explored = 0
    
    def backtrack(unassigned, current_cost, current_domains, current_solution):
        nonlocal best_cost, best_solution, nodes_explored
        nodes_explored += 1
        
        # Poda (Branch & Bound): Si el costo actual ya supera o iguala al mejor, no seguimos
        if current_cost >= best_cost:
            return
            
        # Caso base: Todos los aviones asignados
        if not unassigned:
            if current_cost < best_cost:
                best_cost = current_cost
                best_solution = current_solution.copy()
            return
            
        # PASO 1: Seleccionar variable a instanciar
        # Heurística: Minimum Remaining Values (MRV) adaptada a FC para mayor eficiencia.
        # En tu informe se sugiere usar (L_k - E_k), pero en FC, medir el dominio dinámico poda el árbol mucho más rápido.
        plane = min(unassigned, key=lambda p: sum(len(current_domains[p][r]) for r in runways))
        
        # PASO 2: Darle un valor
        # Recopilar todos los valores válidos (pista, tiempo) y ordenarlos por heurística
        possible_values = []
        for r in runways:
            for t in current_domains[plane][r]:
                pen = calculate_penalty(t, P[plane], C_early[plane], C_late[plane])
                possible_values.append((r, t, pen))
        
        # Heurística de valor: El valor que genere la menor penalización (más cercano a Pk)
        possible_values.sort(key=lambda x: x[2])
        
        for r, t, pen in possible_values:
            # Poda anticipada local
            if current_cost + pen >= best_cost:
                continue
                
            # PASO 3: Mirar el resto de variables y actualizar sus dominios (Forward Checking)
            new_domains = {}
            is_valid = True
            
            for other_plane in unassigned:
                if other_plane == plane:
                    continue
                    
                new_domains[other_plane] = {}
                domain_empty = True
                
                for other_r in runways:
                    valid_times = current_domains[other_plane][other_r].copy()
                    
                    # Restricción de separación solo aplica si aterrizan en la misma pista
                    if other_r == r:
                        # Si 'other' aterriza DESPUÉS de 'plane', debe ser >= t + sep_matrix[plane][other]
                        # Si 'other' aterriza ANTES de 'plane', debe ser <= t - sep_matrix[other][plane]
                        lower_bound = t - sep_matrix[other_plane][plane]
                        upper_bound = t + sep_matrix[plane][other_plane]
                        
                        # Tiempos inválidos caen estrictamente dentro de este margen
                        invalid_times = set(range(lower_bound + 1, upper_bound))
                        valid_times -= invalid_times # Operación de conjuntos en O(N) muy eficiente
                        
                    new_domains[other_plane][other_r] = valid_times
                    
                    if valid_times:
                        domain_empty = False
                        
                # PASO 4: Si a un avión futuro no le quedan valores en ninguna pista, podamos esta rama.
                if domain_empty:
                    is_valid = False
                    break
            
            # Si el valor elegido deja dominios válidos para el resto, seguimos bajando en el árbol
            if is_valid:
                new_unassigned = unassigned.copy()
                new_unassigned.remove(plane)
                
                new_solution = current_solution.copy()
                new_solution[plane] = (r, t) # Asignamos (pista, tiempo)
                
                backtrack(new_unassigned, current_cost + pen, new_domains, new_solution)

    # Iniciar recursión
    unassigned_initial = set(range(n))
    start_time = time.time()
    
    backtrack(unassigned_initial, 0.0, domains, {})
    
    exec_time = time.time() - start_time
    return best_cost, best_solution, nodes_explored, exec_time

# --- EJECUCIÓN DEL CASO 1 ---
if __name__ == "__main__":
    # Asegúrate de tener el archivo case1.txt en el mismo directorio
    # Permite pasar el nombre del archivo por argumento, o usar "Casos/case1.txt" por defecto
    archivo_entrada = "Casos/case1.txt"
    if len(sys.argv) > 1:
        archivo_entrada = sys.argv[1]
    
    data = parse_instance(archivo_entrada)
    #archivo_entrada = "Casos/case1.txt"
    if data:
        n, E, P, L, C_early, C_late, sep_matrix = data
        
        # El problema plantea de 1 a 3 pistas. Para este caso de prueba, asumimos 1 pista por defecto.
        # Puedes cambiar este valor según la experimentación que te pida la tarea.
        NUM_PISTAS = 1 
        
        print(f"Resolviendo instancia de {n} aviones con Forward Checking...")
        cost, solution, nodes, t_exec = solve_alp_fc(n, NUM_PISTAS, E, P, L, C_early, C_late, sep_matrix)
        
        print("\n--- RESULTADOS ---")
        print(f"Mejor Costo (Penalización): {cost}")
        print(f"Nodos Explorados: {nodes}")
        print(f"Tiempo de Ejecución: {t_exec:.4f} segundos")
        print("\nAsignaciones (Pista, Tiempo):")
        if solution:
            for plane_id in sorted(solution.keys()):
                pista, tiempo = solution[plane_id]
                print(f"Avión {plane_id + 1}: Pista {pista}, Tiempo {tiempo} (Preferente: {P[plane_id]})")
        else:
            print("No se encontró solución factible.")