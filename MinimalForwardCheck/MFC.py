"""
Aircraft Landing Problem - Minimal Forward Checking con Branch and Bound
=========================================================================
Algoritmo MFC con Branch and Bound para asignar tiempos (T_k) y pistas (R_k).

Diferencia clave entre algoritmos:
- Backtracking: Verifica restricciones SOLO con aviones ya asignados.
- FC (Forward Checking): Propaga restricciones a TODOS los aviones no asignados.
- MFC (Minimal Forward Checking): Propaga SOLO a los aviones 'j' cuyo dominio
  de pistas D(R_j) aún contiene la pista asignada al avión actual.

Optimización de Rendimiento (Trailing):
  Para evitar el inmenso costo computacional de copiar el estado completo
  en cada nodo (deepcopy), se utiliza un mecanismo de "trailing". Solo se 
  guardan referencias a los dominios originales antes de ser filtrados. Al 
  hacer backtrack, se restauran estas referencias en O(1), permitiendo 
  evaluar cientos de miles de nodos por segundo.
"""

import sys
import time as time_module
from typing import List, Dict, Tuple, Optional, Set

# ---------------------------------------------------------------------------
# Estructuras de datos
# ---------------------------------------------------------------------------

class Avion:
    __slots__ = ('idx', 'E', 'L', 'P', 'C', 'Cp', 'tau')

    def __init__(self, idx: int, E: int, L: int, P: int,
                 C: float, Cp: float, tau_row: List[int]):
        self.idx = idx
        self.E = E          # Tiempo más temprano
        self.L = L          # Tiempo más tardío
        self.P = P          # Tiempo preferente
        self.C = C          # Penalización por adelanto
        self.Cp = Cp        # Penalización por retraso
        self.tau = tau_row  # Separación mínima requerida

    def penalizacion(self, t: int) -> float:
        if t < self.P:
            return self.C * (self.P - t)
        else:
            return self.Cp * (t - self.P)

    def pen_lista(self, tiempos: List[int]) -> float:
        if not tiempos:
            return float('inf')
        return min(self.penalizacion(t) for t in tiempos)

# ---------------------------------------------------------------------------
# Lectura de instancia
# ---------------------------------------------------------------------------

def leer_instancia(archivo: str) -> Tuple[int, List[Avion]]:
    with open(archivo, 'r') as f:
        tokens = f.read().split()

    pos = 0
    D = int(tokens[pos]); pos += 1

    aviones = []
    for k in range(D):
        E  = int(tokens[pos]);   pos += 1
        L  = int(tokens[pos]);   pos += 1
        P  = int(tokens[pos]);   pos += 1
        C  = float(tokens[pos]); pos += 1
        Cp = float(tokens[pos]); pos += 1
        tau_row = [int(tokens[pos + j]) for j in range(D)]
        pos += D
        aviones.append(Avion(k, E, L, P, C, Cp, tau_row))

    return D, aviones

# ---------------------------------------------------------------------------
# Estado de dominios
# ---------------------------------------------------------------------------

class Estado:
    def __init__(self, D: int, aviones: List[Avion], pistas: Tuple[int, ...]):
        self.D = D
        self.pistas = pistas

        # dom_t[k][r] = lista de tiempos válidos para el avión k en la pista r
        self.dom_t: List[Dict[int, List[int]]] = [
            {r: list(range(a.E, a.L + 1)) for r in pistas}
            for a in aviones
        ]
        self.dom_r: List[Set[int]] = [set(pistas) for _ in range(D)]

        self.min_pen: List[float] = [
            aviones[k].pen_lista(list(range(aviones[k].E, aviones[k].L + 1)))
            for k in range(D)
        ]
        self.lb_no_asig: float = sum(self.min_pen)
        self.asignados: Dict[int, Tuple[int, int, float]] = {}
        self.coste_acum: float = 0.0

    def no_asignados(self) -> List[int]:
        # Para mayor velocidad en Python, reconstruimos la lista
        return [k for k in range(self.D) if k not in self.asignados]

    def tamanio_dominio(self, k: int) -> int:
        return sum(len(self.dom_t[k][r]) for r in self.dom_r[k])

    def actualizar_min_pen(self, j: int, avion: Avion, modificaciones: dict) -> None:
        """Recalcula la mínima penalización de 'j' y anota el cambio en el trail."""
        nueva = float('inf')
        for r in self.dom_r[j]:
            if self.dom_t[j][r]:
                nueva = min(nueva, avion.pen_lista(self.dom_t[j][r]))
        
        if nueva != self.min_pen[j]:
            if j not in modificaciones['min_pen']:
                modificaciones['min_pen'][j] = self.min_pen[j]
            self.lb_no_asig += nueva - self.min_pen[j]
            self.min_pen[j] = nueva

# ---------------------------------------------------------------------------
# Heurísticas
# ---------------------------------------------------------------------------

def seleccionar_variable(estado: Estado) -> int:
    """MRV: avión no asignado con menor dominio combinado. Desempate: menor índice."""
    candidatos = estado.no_asignados()
    return min(candidatos, key=lambda k: estado.tamanio_dominio(k))

def ordenar_valores(k: int, avion: Avion, estado: Estado) -> List[Tuple[int, int]]:
    """Ordena combinaciones (pista, tiempo) por proximidad a P_k."""
    combinaciones = []
    for r in sorted(estado.dom_r[k]):
        for t in estado.dom_t[k][r]:
            combinaciones.append((r, t))
    combinaciones.sort(key=lambda rt: (abs(rt[1] - avion.P), rt[1]))
    return combinaciones

# ---------------------------------------------------------------------------
# Propagación MFC
# ---------------------------------------------------------------------------

def propagar_mfc(k: int, r_asig: int, t_asig: int,
                 estado: Estado, aviones: List[Avion], modificaciones: dict) -> bool:
    """
    Propagación Minimal Forward Checking con Trailing.
    Registra cualquier dominio modificado en 'modificaciones' para restaurarlo rápido.
    """
    tau_k = aviones[k].tau

    for j in estado.no_asignados():
        # --- Núcleo MFC ---
        if r_asig not in estado.dom_r[j]:
            continue

        tau_j_k = aviones[j].tau[k]
        tau_k_j = tau_k[j]
        lista = estado.dom_t[j][r_asig]

        nuevos = [
            t_j for t_j in lista
            if t_j <= t_asig - tau_j_k or t_j >= t_asig + tau_k_j
        ]

        if len(nuevos) < len(lista):
            # Trailing: Guardar la referencia a la lista antigua
            modificaciones['dom_t'][(j, r_asig)] = lista
            estado.dom_t[j][r_asig] = nuevos

            if not nuevos:
                # Trailing: Guardar copia del set de pistas
                modificaciones['dom_r'][j] = estado.dom_r[j].copy()
                estado.dom_r[j].discard(r_asig)
                if not estado.dom_r[j]:
                    return False  # Dominio vacío

            estado.actualizar_min_pen(j, aviones[j], modificaciones)

    return True

# ---------------------------------------------------------------------------
# Algoritmo MFC con Branch and Bound
# ---------------------------------------------------------------------------

class Buscador:
    def __init__(self, D: int, aviones: List[Avion]):
        self.D = D
        self.aviones = aviones
        self.mejor_coste: float = float('inf')
        self.mejor_asignacion: Optional[Dict[int, Tuple[int, int, float]]] = None
        self.nodos_explorados: int = 0

    def mfc(self, estado: Estado) -> None:
        self.nodos_explorados += 1

        if len(estado.asignados) == self.D:
            if estado.coste_acum < self.mejor_coste:
                self.mejor_coste = estado.coste_acum
                self.mejor_asignacion = dict(estado.asignados)
            return

        k = seleccionar_variable(estado)
        avion_k = self.aviones[k]

        # Extraer k de la cota inferior global
        lb_sin_k = estado.lb_no_asig - estado.min_pen[k]

        combinaciones = ordenar_valores(k, avion_k, estado)

        for (r, t) in combinaciones:
            # Validar que r y t no hayan sido podados previamente en este bucle
            if r not in estado.dom_r[k] or t not in estado.dom_t[k].get(r, []):
                continue

            pen_k = avion_k.penalizacion(t)
            nuevo_coste = estado.coste_acum + pen_k

            # Poda Branch & Bound
            if nuevo_coste + lb_sin_k >= self.mejor_coste:
                continue

            # --- PREPARACIÓN DEL TRAILING (Deshacer en O(1)) ---
            old_dom_r_k = estado.dom_r[k]
            old_dom_t_k = estado.dom_t[k]
            old_min_pen_k = estado.min_pen[k]
            old_lb_no_asig = estado.lb_no_asig

            estado.asignados[k] = (r, t, pen_k)
            estado.coste_acum = nuevo_coste
            estado.lb_no_asig = lb_sin_k
            estado.min_pen[k] = 0.0
            estado.dom_r[k] = {r}
            estado.dom_t[k] = {r: [t]}

            modificaciones = {'dom_t': {}, 'dom_r': {}, 'min_pen': {}}

            # Propagación
            exito = propagar_mfc(k, r, t, estado, self.aviones, modificaciones)

            if exito:
                self.mfc(estado)

            # --- RESTAURACIÓN RÁPIDA (Backtrack) ---
            for (j, r_mod), old_lst in modificaciones['dom_t'].items():
                estado.dom_t[j][r_mod] = old_lst
            for j, old_set in modificaciones['dom_r'].items():
                estado.dom_r[j] = old_set
            for j, old_pen in modificaciones['min_pen'].items():
                estado.min_pen[j] = old_pen

            del estado.asignados[k]
            estado.coste_acum -= pen_k
            estado.lb_no_asig = old_lb_no_asig
            estado.dom_r[k] = old_dom_r_k
            estado.dom_t[k] = old_dom_t_k
            estado.min_pen[k] = old_min_pen_k

# ---------------------------------------------------------------------------
# Programa principal
# ---------------------------------------------------------------------------

def main():
    if len(sys.argv) >= 2:
        archivo = sys.argv[1]
    else:
        archivo = input("Archivo de instancia [case1.txt]: ").strip() or "case1.txt"

    while True:
        try:
            n_pistas_str = input("Número de pistas disponibles (1-3) [3]: ").strip()
            n_pistas = int(n_pistas_str) if n_pistas_str else 3
            if 1 <= n_pistas <= 3:
                break
            print("  Por favor ingresa un valor entre 1 y 3.")
        except ValueError:
            print("  Entrada inválida. Ingresa un número entero.")

    pistas: Tuple[int, ...] = tuple(range(1, n_pistas + 1))

    print(f"\n{'='*68}")
    print(f"  Aircraft Landing Problem — MFC + Branch & Bound")
    print(f"  Archivo : {archivo}")
    print(f"  Pistas  : {list(pistas)}")
    print(f"{'='*68}\n")

    try:
        D, aviones = leer_instancia(archivo)
    except FileNotFoundError:
        print(f"ERROR: No se encontró el archivo '{archivo}'")
        sys.exit(1)
    except Exception as e:
        print(f"ERROR al leer instancia: {e}")
        sys.exit(1)

    print(f"Número de aviones: {D}")

    estado_inicial = Estado(D, aviones, pistas)
    buscador = Buscador(D, aviones)

    t_inicio = time_module.perf_counter()
    buscador.mfc(estado_inicial)
    t_fin = time_module.perf_counter()
    tiempo_cpu = t_fin - t_inicio

    print(f"\n{'='*68}")
    if buscador.mejor_asignacion is None:
        print("  No hay solución factible.")
    else:
        print("  SOLUCIÓN ÓPTIMA ENCONTRADA")
        print(f"{'='*68}")

        h = (f"  {'Avión':>5}  {'Pista':>5}  {'E':>6}  {'P':>6}  "
             f"{'L':>6}  {'T asig.':>8}  {'Costo':>10}")
        sep = (f"  {'-'*5}  {'-'*5}  {'-'*6}  {'-'*6}  "
               f"{'-'*6}  {'-'*8}  {'-'*10}")
        print(h)
        print(sep)

        for k in range(D):
            r, t, pen = buscador.mejor_asignacion[k]
            a = aviones[k]
            print(f"  {k+1:>5}  {r:>5}  {a.E:>6}  {a.P:>6}  "
                  f"{a.L:>6}  {t:>8}  {pen:>10.2f}")

        print(sep)
        print(f"  {'TOTAL':>5}  {'':>5}  {'':>6}  {'':>6}  "
              f"{'':>6}  {'':>8}  {buscador.mejor_coste:>10.2f}")

    print(f"\n  Nodos explorados : {buscador.nodos_explorados}")
    print(f"  Tiempo CPU       : {tiempo_cpu:.4f} s")
    print(f"{'='*68}\n")

if __name__ == "__main__":
    main()