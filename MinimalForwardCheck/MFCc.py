"""
Aircraft Landing Problem - Minimal Forward Checking con Branch and Bound
=========================================================================
Algoritmo MFC (Minimal Forward Checking) con Branch and Bound para asignar
tiempos (T_k) y pistas (R_k) a D aviones minimizando penalizaciones.

Diferencia clave entre algoritmos:
- Backtracking: asigna y comprueba restricciones SOLO con aviones ya asignados.
  No hay propagación hacia adelante.
- Forward Checking (FC) estándar: al asignar (r, t) al avión k, propaga
  restricciones hacia TODOS los aviones no asignados.
- MFC (Minimal Forward Checking): al asignar (r, t) al avión k, SOLO propaga
  hacia los aviones j cuyo dominio de pistas D(R_j) aún contiene r.
  Los aviones que ya descartaron r no pueden coincidir con k → se omiten,
  reduciendo el número de verificaciones por nodo.

Diseño de dominios (per-pista):
  dom_t[k][r] = lista de tiempos válidos para el avión k en la pista r
  dom_r[k]    = conjunto de pistas aún viables para el avión k

Esta representación per-pista es necesaria para la corrección: un tiempo t_j
puede ser incompatible con k en la pista r, pero perfectamente válido en la
pista r'≠r. Filtrar dom_t[j] globalmente causaría sobre-poda incorrecta.

Uso:
    python mfc_aircraft_landing.py [archivo_instancia]
    Si no se pasa argumento, se usa "case1.txt" por defecto.
"""

import sys
import time as time_module
from typing import List, Dict, Tuple, Optional


PISTAS = (1, 2, 3)   # pistas disponibles


# ---------------------------------------------------------------------------
# Estructuras de datos
# ---------------------------------------------------------------------------

class Avion:
    """Parámetros de un avión (índice 0-based internamente)."""
    __slots__ = ('idx', 'E', 'L', 'P', 'C', 'Cp', 'tau')

    def __init__(self, idx: int, E: int, L: int, P: int,
                 C: float, Cp: float, tau_row: List[int]):
        self.idx = idx
        self.E = E          # earliest landing time
        self.L = L          # latest landing time
        self.P = P          # preferred landing time (puede ser > L)
        self.C = C          # penalización por unidad de adelanto
        self.Cp = Cp        # penalización por unidad de retraso
        self.tau = tau_row  # tau[j] = separación mínima si self aterriza antes que j

    def penalizacion(self, t: int) -> float:
        """Coste de aterrizar en el tiempo t."""
        if t < self.P:
            return self.C * (self.P - t)
        else:
            return self.Cp * (t - self.P)


# ---------------------------------------------------------------------------
# Lectura de instancia
# ---------------------------------------------------------------------------

def leer_instancia(archivo: str) -> Tuple[int, List[Avion]]:
    """
    Formato del archivo:
      Línea 1: D (número de aviones)
      Por cada avión k (1..D), 3 líneas:
        1. E_k L_k P_k C_k C'_k
        2. primeros 8 valores de τ (fila k)
        3. restantes D-8 valores de τ (fila k)
    Retorna (D, lista_de_Avion).
    """
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
    """
    Encapsula los dominios y la asignación parcial.

    dom_t[k][r]  : lista de tiempos válidos para el avión k en la pista r
    dom_r[k]     : conjunto de pistas aún viables para el avión k
    asignados    : dict k → (pista, tiempo, penalización)
    coste_acum   : suma de penalizaciones de los aviones ya asignados
    """

    def __init__(self, D: int, aviones: List[Avion]):
        self.D = D
        # Dominio inicial: todas las pistas con todos los tiempos [E_k .. L_k]
        self.dom_t: List[Dict[int, List[int]]] = [
            {r: list(range(a.E, a.L + 1)) for r in PISTAS}
            for a in aviones
        ]
        self.dom_r: List[set] = [{1, 2, 3} for _ in range(D)]
        self.asignados: Dict[int, Tuple[int, int, float]] = {}
        self.coste_acum: float = 0.0

    def copia(self) -> 'Estado':
        """Copia profunda eficiente de los dominios."""
        nuevo = Estado.__new__(Estado)
        nuevo.D = self.D
        nuevo.dom_t = [{r: lst[:] for r, lst in d.items()} for d in self.dom_t]
        nuevo.dom_r = [s.copy() for s in self.dom_r]
        nuevo.asignados = dict(self.asignados)
        nuevo.coste_acum = self.coste_acum
        return nuevo

    def no_asignados(self) -> List[int]:
        return [k for k in range(self.D) if k not in self.asignados]

    def tamanio_dominio(self, k: int) -> int:
        """
        Tamaño del dominio combinado ≈ |D(T_k)| × |D(R_k)|:
        suma de tiempos disponibles por pista viable.
        """
        return sum(len(self.dom_t[k][r]) for r in self.dom_r[k])

    def tiempos_disponibles(self, k: int) -> List[int]:
        """Unión de tiempos disponibles en todas las pistas viables de k."""
        tiempos: set = set()
        for r in self.dom_r[k]:
            tiempos.update(self.dom_t[k][r])
        return sorted(tiempos)


# ---------------------------------------------------------------------------
# Heurísticas
# ---------------------------------------------------------------------------

def seleccionar_variable(estado: Estado) -> int:
    """
    MRV (Minimum Remaining Values): elige el avión no asignado con menor
    dominio combinado (|D(T_k)| × |D(R_k)|, aproximado como suma de
    tiempos por pista). Desempate: menor índice.
    """
    candidatos = estado.no_asignados()
    return min(candidatos, key=lambda k: estado.tamanio_dominio(k))


def ordenar_valores(k: int, avion: Avion, estado: Estado) -> List[Tuple[int, int]]:
    """
    Genera todas las combinaciones (pista, tiempo) del dominio actual del avión k
    y las ordena por proximidad a P_k (menor |t - P_k| primero).
    Desempate: tiempo más temprano.
    """
    combinaciones = []
    for r in sorted(estado.dom_r[k]):
        for t in estado.dom_t[k][r]:
            combinaciones.append((r, t))
    combinaciones.sort(key=lambda rt: (abs(rt[1] - avion.P), rt[1]))
    return combinaciones


# ---------------------------------------------------------------------------
# Cota inferior para Branch & Bound
# ---------------------------------------------------------------------------

def pen_minima_avion(avion: Avion, tiempos: List[int]) -> float:
    """Penalización mínima posible dada la lista de tiempos disponibles."""
    if not tiempos:
        return float('inf')
    return min(avion.penalizacion(t) for t in tiempos)


# ---------------------------------------------------------------------------
# Propagación MFC
# ---------------------------------------------------------------------------

def propagar_mfc(k: int, r_asig: int, t_asig: int,
                 estado: Estado, aviones: List[Avion]) -> bool:
    """
    Propagación Minimal Forward Checking tras asignar (r_asig, t_asig) al avión k.

    MFC solo recorre los aviones j con r_asig ∈ dom_r[j]:
      - Si r_asig ∉ dom_r[j]: j ya no puede compartir pista con k → omitir (MFC).
      - Si r_asig ∈ dom_r[j]: j podría aterrizar en r_asig junto a k.
          Filtrar dom_t[j][r_asig] manteniendo solo los t_j que satisfacen:
              t_j ≤ t_asig - τ_{j,k}   (j aterriza antes que k)  O
              t_j ≥ t_asig + τ_{k,j}   (k aterriza antes que j)
          Si dom_t[j][r_asig] queda vacío:
              → r_asig ya no es viable para j → eliminar de dom_r[j].
              → Si dom_r[j] queda vacío → fallo (retorna False).

    NOTA SOBRE CORRECCIÓN:
    Filtramos POR PISTA (dom_t[j][r_asig]) y no el dominio plano global.
    Un tiempo t_j incompatible en r_asig puede ser perfectamente válido en
    otra pista r'≠r_asig → no debe eliminarse globalmente.

    Retorna True si la propagación es exitosa, False si hay fallo.
    """
    tau_k = aviones[k].tau  # fila de τ del avión k

    for j in estado.no_asignados():
        # --- Núcleo MFC: solo aviones con r_asig en su dominio de pistas ---
        if r_asig not in estado.dom_r[j]:
            continue  # j descartó r_asig → no compartirá pista con k → omitir

        tau_j_k = aviones[j].tau[k]  # separación si j aterriza antes que k
        tau_k_j = tau_k[j]           # separación si k aterriza antes que j

        # Filtrado POR PISTA: solo dom_t[j][r_asig]
        nuevos = [
            t_j for t_j in estado.dom_t[j][r_asig]
            if t_j <= t_asig - tau_j_k or t_j >= t_asig + tau_k_j
        ]

        if len(nuevos) < len(estado.dom_t[j][r_asig]):
            estado.dom_t[j][r_asig] = nuevos

            if not nuevos:
                # Ningún tiempo compatible en r_asig → r_asig no viable
                estado.dom_r[j].discard(r_asig)
                if not estado.dom_r[j]:
                    return False  # j se quedó sin pistas → fallo

    return True


# ---------------------------------------------------------------------------
# Algoritmo MFC con Branch and Bound
# ---------------------------------------------------------------------------

class Buscador:
    """Encapsula el estado global de la búsqueda."""

    def __init__(self, D: int, aviones: List[Avion]):
        self.D = D
        self.aviones = aviones
        self.mejor_coste: float = float('inf')
        self.mejor_asignacion: Optional[Dict[int, Tuple[int, int, float]]] = None
        self.nodos_explorados: int = 0

    def mfc(self, estado: Estado) -> None:
        """
        Búsqueda recursiva MFC con Branch & Bound.

        Correcto y completo: los dominios per-pista garantizan que no se
        pierden soluciones válidas, y la cota inferior admisible asegura
        que el óptimo global es encontrado si existe.
        """
        self.nodos_explorados += 1

        # --- Caso base: todos los aviones asignados ---
        if len(estado.asignados) == self.D:
            if estado.coste_acum < self.mejor_coste:
                self.mejor_coste = estado.coste_acum
                self.mejor_asignacion = dict(estado.asignados)
            return

        # --- Selección de variable (MRV) ---
        k = seleccionar_variable(estado)
        avion_k = self.aviones[k]

        # --- Generación y ordenación de valores ---
        combinaciones = ordenar_valores(k, avion_k, estado)

        for (r, t) in combinaciones:
            # Verificar validez (puede haber cambiado en iteraciones anteriores)
            if r not in estado.dom_r[k] or t not in estado.dom_t[k].get(r, []):
                continue

            # --- Penalización parcial ---
            pen_k = avion_k.penalizacion(t)
            nuevo_coste = estado.coste_acum + pen_k

            # --- Poda Branch & Bound ANTES de propagar ---
            # Cota inferior admisible: coste acumulado + mín. penalización de
            # cada avión no asignado (ignorando restricciones de separación →
            # cota optimista).
            cota = nuevo_coste
            poda = False
            for j in estado.no_asignados():
                if j == k:
                    continue
                tiempos_j = estado.tiempos_disponibles(j)
                if not tiempos_j:
                    poda = True
                    break
                cota += pen_minima_avion(self.aviones[j], tiempos_j)
                if cota >= self.mejor_coste:
                    poda = True
                    break

            if poda:
                continue  # Rama podada por B&B

            # --- Guardar estado ANTES de modificar ---
            estado_guardado = estado.copia()

            # --- Asignación provisional ---
            estado.asignados[k] = (r, t, pen_k)
            estado.coste_acum = nuevo_coste
            # Fijar dominio de k a la elección actual
            estado.dom_r[k] = {r}
            estado.dom_t[k] = {r: [t]}

            # --- Propagación MFC ---
            exito = propagar_mfc(k, r, t, estado, self.aviones)

            if exito:
                self.mfc(estado)  # llamada recursiva

            # --- Restaurar estado (backtrack) ---
            estado.dom_t      = estado_guardado.dom_t
            estado.dom_r      = estado_guardado.dom_r
            estado.asignados  = estado_guardado.asignados
            estado.coste_acum = estado_guardado.coste_acum


# ---------------------------------------------------------------------------
# Programa principal
# ---------------------------------------------------------------------------

def main():
    if len(sys.argv) >= 2:
        archivo = sys.argv[1]
    else:
        archivo = input("Archivo de instancia [case1.txt]: ").strip() or "case1.txt"

    print(f"\n{'='*64}")
    print(f"  Aircraft Landing Problem — MFC + Branch & Bound")
    print(f"  Archivo: {archivo}")
    print(f"{'='*64}\n")


    try:
        D, aviones = leer_instancia(archivo)
    except FileNotFoundError:
        print(f"ERROR: No se encontró el archivo '{archivo}'")
        sys.exit(1)
    except Exception as e:
        print(f"ERROR al leer instancia: {e}")
        sys.exit(1)

    print(f"Número de aviones: {D}")

    estado_inicial = Estado(D, aviones)
    buscador = Buscador(D, aviones)

    t_inicio = time_module.perf_counter()
    buscador.mfc(estado_inicial)
    t_fin = time_module.perf_counter()
    tiempo_cpu = t_fin - t_inicio

    print(f"\n{'='*64}")
    if buscador.mejor_asignacion is None:
        print("  No hay solución factible.")
    else:
        print("  SOLUCIÓN ÓPTIMA ENCONTRADA")
        print(f"{'='*64}")
        print(f"  {'Avión':>5}  {'Pista':>5}  {'Tiempo':>7}  "
              f"{'Preferente':>10}  {'Penalización':>12}")
        print(f"  {'-'*5}  {'-'*5}  {'-'*7}  {'-'*10}  {'-'*12}")

        for k in range(D):
            r, t, pen = buscador.mejor_asignacion[k]
            print(f"  {k+1:>5}  {r:>5}  {t:>7}  {aviones[k].P:>10}  {pen:>12.2f}")

        print(f"\n  Coste total óptimo : {buscador.mejor_coste:.2f}")

    print(f"\n  Nodos explorados   : {buscador.nodos_explorados}")
    print(f"  Tiempo CPU         : {tiempo_cpu:.4f} s")
    print(f"{'='*64}\n")


if __name__ == "__main__":
    main()