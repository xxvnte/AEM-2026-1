# Documentación Algoritmo Propuesto: EH-SA/TS

---

## Motivación: Desafíos de SA/TS en el EVRP sin Ventanas de Tiempo

La hibridación entre Tabu Search (TS) y Simulated Annealing (SA) ha demostrado resultados sólidos en variantes del EVRP con restricciones adicionales, como el trabajo de Küçükoğlu et al. (2019) para el problema del vehículo eléctrico con ventanas de tiempo y tasas de recarga mixtas (ETSPTW-MCR). Sin embargo, su aplicación directa al EVRP sin ventanas de tiempo presenta desafíos fundamentales derivados del **dinamismo del coste energético**, donde al mover un nodo en la ruta, la carga transportada $m_{ij}$ cambia en todos los arcos sucesivos, propagando un recálculo en cascada que modifica el paisaje de optimización en cada iteración. Este fenómeno, reconocido explícitamente por Zhang et al. (2018) en la motivación de sus funciones sustitutas, afecta directamente a los dos mecanismos centrales de la hibridación SA/TS, generando los tres problemas descritos a continuación:

| Problema                      | Descripción                                                                                                                                                                            |
| ----------------------------- | -------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **Lista Tabú desactualizada** | El costo energético cambia en cada iteración, haciendo que los movimientos prohibidos queden desactualizados. La lista tabú pierde su utilidad como memoria de ciclos.                 |
| **SA con datos incorrectos**  | SA fue diseñado para operar sobre un paisaje estable. Al cambiar el costo en cada iteración, acepta y rechaza movimientos con información incorrecta.                                  |
| **Cálculo en cascada**        | Al mover un nodo, el costo de todos los arcos siguientes cambia, obligando a recalcular toda la cadena en cada movimiento evaluado, lo que hace al algoritmo computacionalmente lento. |

---

## Descripción General del EH-SA/TS

Para abordar los tres problemas identificados, se propone el algoritmo **Enhanced Hybrid SA/TS (EH-SA/TS)**, una metaheurística híbrida en la que el Simulated Annealing actúa como criterio de aceptación del operador de Tabu Search, siguiendo el esquema de hibridación propuesto por Küçükoğlu et al. (2019). En este diseño, ambos algoritmos operan en un único bucle de optimización, donde TS actúa como generador del vecindario guiado por la memoria estructural, mientras que SA reemplaza la lógica de aceptación estricta del TS, permitiendo aceptar movimientos peores con probabilidad $e^{-\Delta E / T}$. Esto habilita la exploración y la explotación simultáneamente en cada iteración, previniendo el estancamiento sin necesidad de transiciones secuenciales entre algoritmos.

Sobre esta base híbrida, el EH-SA/TS incorpora tres capas de mejora que atacan directamente los problemas identificados:

| Capa | Componente                                | Problema que resuelve                        |
| ---- | ----------------------------------------- | -------------------------------------------- |
| 1    | Criterio tabú estructural (TABUROUTE)     | Tabú desactualizada por dinamismo            |
| 2a   | Función sustituta $f'_{approx}$           | Cálculo en cascada y datos incorrectos en SA |
| 2b   | Vecindario restringido (_candidate list_) | Cálculo en cascada                           |
| 3    | Programación Dinámica adaptada            | Variabilidad del subproblema de recarga      |

---

## Procedimiento de Inicialización

La solución inicial se construye en dos fases. En la primera, se aplica un procedimiento _greedy_ aleatorio que asigna clientes a vehículos construyendo rutas sin considerar las restricciones de batería. En la segunda, la Programación Dinámica de la Capa 3 inserta el plan de recarga sobre cada ruta construida. El objetivo es entregar al algoritmo principal una solución **lo más factible posible** (todos los clientes visitados, capacidad y batería respetadas); si el DP no encuentra un plan completo, se aplican mecanismos de respaldo (greedy de estaciones y, en última instancia, retorno al depósito para recarga completa).

```
Fase 1: greedy aleatorio → construir rutas asignando clientes a vehículos
        (sin considerar batería aún)
Fase 2: DP inserta plan de recarga inicial sobre cada ruta
        (con fallback greedy + retorno al depósito si hace falta)
→ SA parte con un plan de recarga coherente para la secuencia inicial
  (factibilidad estricta verificada al reportar resultados)
```

---

## Pilares del EH-SA/TS

### Capa 1: Criterio Tabú Estructural

El criterio tabú estándar prohíbe movimientos basándose en el valor de la función objetivo, lo cual resulta ineficaz en el EVRP sin ventanas de tiempo, ya que el coste energético cambia en cada iteración invalidando continuamente las prohibiciones registradas. La Capa 1 reemplaza este criterio por una **memoria basada en atributos estructurales**, siguiendo el modelo clásico TABUROUTE propuesto por Gendreau, Hertz y Laporte (1994).

El movimiento prohibido es el par **(cliente, ruta de origen)**, donde una vez que un cliente es removido de su ruta actual e insertado en otra, queda prohibido volver a insertarlo en su ruta de origen durante $K_{tabú}$ iteraciones, independientemente de cómo evolucione el coste energético. La lista tabú nunca queda desactualizada porque su contenido no depende de ningún coste dinámico, sino únicamente de la estructura de asignación cliente-ruta.

El cliente candidato a remover se selecciona en base a su **potencial de mejora**, definido por dos criterios independientes del coste dinámico:

- **Mayor demanda $e_i$**: su reubicación tiene el mayor impacto potencial en $m_{ij}$ y, por tanto, en el consumo energético de la ruta.
- **Mayor distancia al centroide de su ruta actual**: criterio puramente estructural y geométrico, independiente del coste energético.

La evaluación del impacto de la reubicación se realiza mediante la función sustituta $f'_{approx}$ de la Capa 2, evitando el recálculo exacto durante la selección.

**Mecanismo de protección ante el dinamismo:**

| Componente                  | Cómo protege ante el dinamismo                                                                              |
| --------------------------- | ----------------------------------------------------------------------------------------------------------- |
| Capa 2 ($f'_{approx}$)      | La selección no usa el costo exacto dinámico sino uno consistentemente aproximado                           |
| Capa 1 (TABUROUTE)          | Aunque la selección fue subóptima, el algoritmo no puede quedar atrapado en un ciclo por esa suboptimalidad |
| SA (criterio de aceptación) | Si el dinamismo hizo que una decisión "buena" resulte mala, SA permite escapar en vez de quedar bloqueado   |

---

### Capa 2: Reducción del Coste Computacional

La Capa 2 ataca el problema del cálculo en cascada mediante dos mecanismos complementarios.

#### Función sustituta $f'_{approx}$

Adaptada a partir de las funciones sustitutas $f'*e(S)$ y $L'*{batt}(S)$ propuestas por Zhang et al. (2018), esta función evita el recálculo energético en cascada asumiendo que los arcos anteriores al punto de cambio mantienen su valor energético previo. Formalmente:

$$f'*{approx}(\text{movimiento}) = f*{actual} - \sum E_{\text{arcos eliminados}} + \sum E_{\text{arcos nuevos creados}}$$

donde todos los arcos anteriores al punto de cambio quedan congelados. La función se aplica en dos momentos del loop híbrido:

- **En TS**: para evaluar cada movimiento candidato del vecindario.
- **En SA**: para calcular el $\Delta E$ que determina la probabilidad de aceptación según la distribución de Boltzmann $P = e^{-\Delta E / T}$.

La adaptación de $f'_{approx}$ para cada uno de los cuatro movimientos de búsqueda local es la siguiente:

**Relocate** (mover cliente $X$ de ruta 1 a ruta 2):

```
Δf' = - E(A→X) - E(X→B) + E(A→B)
      - E(C→D) + E(C→X) + E(X→D)
(arcos anteriores a A y C: congelados)
```

**Exchange** (intercambiar clientes $X$ e $Y$):

```
Δf' = aplicar lógica de Relocate dos veces simultáneamente:
      primero X → posición de Y
      luego  Y → posición de X
```

**2-opt** (reconectar arcos entre dos rutas):

```
Δf' = - E(arco 1) - E(arco 2)
      + E(nuevo arco 1) + E(nuevo arco 2)
(segmentos internos: congelados)
```

**StationInRe** (insertar o eliminar estación de recarga):

```
Δf' = misma lógica de Relocate con vértice de estación
```

El recálculo exacto se realiza una única vez por movimiento aceptado, disparado inmediatamente después de la aceptación por SA.

#### Vecindario restringido (_candidate list_)

Para reducir el número de movimientos evaluados, se restringe el vecindario a los clientes con mayor potencial de mejora (mayor demanda o mayor distancia al centroide de su ruta), eliminando el ruido computacional de evaluar movimientos de bajo impacto potencial sin sacrificar diversidad.

---

### Capa 3: Programación Dinámica Adaptada

Inspirada en el procedimiento _station_insertion_ de Küçükoğlu et al. (2019), la Capa 3 incorpora un módulo de Programación Dinámica (DP) que resuelve el subproblema de inserción óptima de estaciones de recarga para una secuencia de clientes fija. La adaptación al EVRP de Zhang et al. (2018) elimina el componente de ventanas de tiempo de las etiquetas y sustituye el objetivo de minimización de distancia por el de minimización de energía.

Dado que el modelo asume recarga completa al visitar una estación o el depósito, la DP no decide cuánta energía cargar sino **dónde y cuándo visitar las estaciones** a lo largo de la ruta. Las etiquetas DP tienen la forma $\{q,\ c\}$, donde $q$ es el nivel de batería al llegar a un nodo y $c$ es el coste energético acumulado.

Para cada par de nodos consecutivos en la secuencia $(N_{i-1}, N_i)$, la DP no elige únicamente el camino de **menor energía local**: conserva un conjunto **Pareto** de etiquetas no dominadas en $(q, c)$, porque un tramo barato en energía puede dejar batería insuficiente para continuar hacia $N_{i+1}$. Concretamente:

- Se generan todos los caminos factibles entre $N_{i-1}$ y $N_i$ (directo o vía estaciones/depósito).
- Solo se acepta una etiqueta con batería $q'$ en $N_i$ si $q' \geq q_{\min}(N_i \to N_{i+1})$, es decir, si alcanza la batería mínima necesaria para el tramo siguiente (**lookahead**).
- Entre las etiquetas válidas se mantiene el frente Pareto en $(q, c)$ antes de pasar al siguiente índice.

Si en algún paso no quedan etiquetas válidas, se activa un **greedy de respaldo** con la misma lógica de lookahead; si aun así no hay camino factible hacia el siguiente cliente, se permite un **retorno al depósito** (recarga completa) y se continúa la ruta.

```
Entrada: secuencia de clientes de UNA ruta
         R = (depot → C1 → C2 → ... → Cn → depot)

Etiquetas DP: {q, c}
  q → batería al llegar al nodo
  c → coste energético acumulado
  (sin componente de tiempo: no hay ventanas de tiempo)

Para cada par consecutivo (N(i-1), Ni):
  Para cada etiqueta {q, c} en L(i-1):
    Generar caminos factibles (directo / vía estación / vía depósito)
    Calcular {q', c'} al llegar a Ni
    Aceptar solo si q' ≥ q_min(Ni → N(i+1))   // lookahead
  Mantener etiquetas Pareto no dominadas en L(i)

Fallback si L(i) queda vacío:
  greedy de estaciones → retorno al depósito si hace falta

Salida: plan de recarga + coste energético f_e(R)
```

La DP se invoca sobre cada ruta modificada **antes** de que SA decida aceptar o rechazar el movimiento de TS, aplicándose únicamente sobre las rutas afectadas. Si SA rechaza el movimiento, el resultado del DP se descarta. Esto favorece evaluar $f_{gen}$ sobre un plan de recarga coherente.

---

## Funciones de Evaluación

El EH-SA/TS opera con tres niveles de evaluación con distintos propósitos y costes computacionales:

| Función                                                            | Cuándo se usa                                                     | Propósito                                                            |
| ------------------------------------------------------------------ | ----------------------------------------------------------------- | -------------------------------------------------------------------- |
| $f'_{approx}(S)$                                                   | Evaluación de cada vecino en TS                                   | Evaluación rápida, evita recálculo en cascada                        |
| $f_{gen}(S) = f_e + \gamma_{cap} L_{cap} + \gamma_{batt} L_{batt}$ | Criterio de aceptación SA sobre el movimiento elegido **post-DP** | Guía la búsqueda; permite penalizar violaciones de capacidad/batería |
| $f_e$ exacta (con penalizaciones)                                  | Dentro de $f_{gen}$ durante el loop híbrido                       | Coste energético simulado tras reoptimizar el plan de recarga        |
| **Validación estricta**                                            | Al finalizar y en comparaciones con MIP (`-small`)                | Comprueba factibilidad real antes de reportar energía                |

**Búsqueda vs reporte.** Durante SA/TS, $f_{gen}$ puede explorar soluciones con penalizaciones ($L_{cap}$, $L_{batt}$) para no bloquear la exploración. En cambio, la **energía EH-SA/TS que se compara con OR-Tools** solo se reporta si la solución cumple simultáneamente:

- todos los clientes visitados **exactamente una vez**;
- capacidad del vehículo respetada en cada ruta;
- batería $\geq 0$ en **cada arco** (simulación estricta, sin “resetear” violaciones).

Si alguna condición falla, la instancia se marca como **`INFACTIBLE`** y no se calcula Absolute Gap frente al MIP.

**Nota:** En la implementación, el DP se invoca sobre el movimiento elegido por TS antes de que SA tome la decisión de aceptar o rechazar. Si SA rechaza el movimiento, el trabajo del DP de esa iteración se descarta. Dado que el DP se aplica únicamente sobre las rutas afectadas por el movimiento, el costo computacional adicional es acotado. La ventaja de esta estrategia es que $f_{gen}$ opera sobre un plan de recarga reoptimizado en lugar de uno aproximado, mejorando la calidad de la comparación en SA.

La función sustituta se construye como:

$$f'*{approx}(S) = f'e(S) + \gamma{cap} \cdot L*{cap}(S) + \gamma_{batt} \cdot L'_{batt}(S)$$

donde $L_{cap}(S)$ no requiere versión sustituta porque la capacidad total del vehículo no depende del orden de recargas y se calcula directamente.

---

## Pseudocódigo del EH-SA/TS

### Algoritmo 1: DP_StationInsertion

```
Entrada: Secuencia de clientes de una ruta
         R = (depot, C1, C2, ..., Cn, depot)
         Conjunto de estaciones S
         Capacidad de batería T

Salida:  Plan de recarga y coste energético f_e(R)

1: Inicializar etiqueta L(0) ← {q = T, c = 0}    // batería llena en depósito
2: Para i = 1 hasta n+1:
3:   Para cada etiqueta {q, c} en L(i-1):
4:     Para cada camino factible entre N(i-1) y Ni
        (directo, vía estación o vía depósito):
5:       Si q ≥ energía requerida por el camino:
6:         Calcular {q', c'} al llegar a Ni
7:         Si Ni no es el depósito final:
8:           Exigir q' ≥ q_min(Ni → N(i+1))       // lookahead hacia el siguiente nodo
9:         Si {q', c'} no está dominado en L(i):
10:          Agregar {q', c'} a L(i)
11:   Eliminar etiquetas dominadas de L(i)
12:   Si L(i) está vacío:
13:     Aplicar greedy de respaldo; si falla, permitir retorno al depósito
14: Retornar etiqueta de menor c en L(n+1) y plan de recarga asociado
```

---

### Algoritmo 2: EH-SA/TS

```
Entrada: Instancia EVRP
         Parámetros: T0, α, T_min, K_tabú, γ_cap, γ_batt, umbral

Salida:  Mejor solución encontrada S*

//--- Inicialización ---
1:  S ← construcción greedy aleatoria de rutas de clientes
2:  Para cada ruta R en S:
3:    R ← DP_StationInsertion(R)
4:  S* ← S
5:  T ← T0
6:  iter_sin_mejora ← 0
7:  Lista_tabú ← ∅

//--- Loop principal híbrido ---
8:  Mientras (T > T_min) Y (iter_sin_mejora < umbral):

      // Generación y filtrado del vecindario (TS)
9:    N ← generar vecindario con movimientos:
             Relocate, Exchange, 2-opt*, StationInRe
10:   Filtrar N por candidate list
             (clientes de mayor demanda o más alejados de su ruta (centroide))

      // Evaluación con función sustituta
11:   Para cada movimiento m en N:
12:     Δf' ← f'_approx(m)
13:     Si par (cliente, ruta_origen) de m está en Lista_tabú:
14:       Marcar m como tabú

// Evaluación SA con plan de recarga óptimo (decisión de implementación)
15:   m* ← movimiento no tabú con menor Δf' en N

16:   S_trial ← aplicar m* a S (solo secuencia de clientes)
17:   Para cada ruta R modificada en S_trial:
18:     R ← DP_StationInsertion(R)     // DP sobre rutas afectadas
19:   ΔE ← f_gen(S_trial) - f_gen(S)  // comparación sobre plan DP consistente

20:   Si ΔE ≤ 0:
21:     aceptado ← verdadero
22:   Si no:
23:     aceptado ← verdadero con probabilidad e^(-ΔE/T)
24:              ← falso en caso contrario

      // Aplicación del movimiento aceptado
25:   Si aceptado:
26:     S ← S_trial
27:     Actualizar Lista_tabú: agregar par (cliente, ruta_origen) de m*
28:     Si f_gen(S) < f_gen(S*):
29:       S* ← S
30:       iter_sin_mejora ← 0
31:     Si no:
32:       iter_sin_mejora ← iter_sin_mejora + 1
33:   Si no aceptado:
34:     Descartar S_trial
35:     iter_sin_mejora ← iter_sin_mejora + 1

36:   T ← α · T

37: E_reporte ← f_e estricta(S*) si S* es factible; INFACTIBLE en caso contrario
38: Retornar S*, E_reporte
```

---

## Configuración de Parámetros

Los valores iniciales de los parámetros se establecen en base a estándares de la literatura para algoritmos híbridos SA/TS en problemas de enrutamiento de vehículos.

| Parámetro       | Descripción                                            | Valor inicial |
| --------------- | ------------------------------------------------------ | ------------- |
| $T_0$           | Temperatura inicial                                    | 20            |
| $\alpha$        | Tasa de enfriamiento geométrico                        | 0.97          |
| $T_{min}$       | Temperatura mínima de parada                           | 0.01          |
| $K_{tabú}$      | Tenencia tabú (iteraciones de prohibición)             | 12            |
| $\gamma_{cap}$  | Factor de penalización por violación de capacidad      | 200           |
| $\gamma_{batt}$ | Factor de penalización por violación de batería        | 100           |
| umbral          | Iteraciones consecutivas sin mejora global para parada | 50–100        |

---

## Requisitos

El proyecto usa Python 3. Antes de ejecutar `main.py`, instala las dependencias desde la carpeta `project`:

```bash
cd project
python -m pip install --user -r requirements.txt
```

---

## Ejecución

### Comandos principales

```bash
cd project
python main.py -small
python main.py -large --runs 10 --seed 42
python main.py -all
python main.py --help
```

Con `-all` se ejecutan en secuencia: `-small` → `-large` → `recharge-stations` → `battery-reserve` → `energy-vs-distance` (mismo protocolo experimental que el paper (Zhang et al.), más los tres análisis de sensibilidad del paper).

### Ejemplo: una instancia concreta

```bash
cd project
python main.py single C25R2-1
python main.py single C14R2 --seed 123
```

Salida en consola (y en `logs/run_NNN_single_C25R2-1.txt`): energía EH-SA/TS (kWh) si la solución es factible (`INFACTIBLE` en caso contrario), `f_gen`, tiempo, número de rutas y visitas a estaciones. No compara con el solver MIP; sirve para depurar o repetir un caso puntual.

### Modos disponibles

| Modo                 | Comando                             | Log típico                            |
| -------------------- | ----------------------------------- | ------------------------------------- |
| Pequeñas             | `python main.py -small`             | `logs/run_001_small.txt`              |
| Grandes              | `python main.py -large`             | `logs/run_001_large.txt`              |
| Todo                 | `python main.py -all`               | `logs/run_001_all.txt`                |
| Estaciones           | `python main.py recharge-stations`  | `logs/run_001_recharge-stations.txt`  |
| Reserva batería      | `python main.py battery-reserve`    | `logs/run_001_battery-reserve.txt`    |
| Energía vs distancia | `python main.py energy-vs-distance` | `logs/run_001_energy-vs-distance.txt` |
| Una instancia        | `python main.py single <nombre>`    | `logs/run_001_single_<nombre>.txt`    |

También aceptan forma sin guión (`small`, `large`, `all`) o con guión (`-recharge-stations`, etc.).

Modos auxiliares (no entran en `-all`): `extended` (C10R2, C11R2) y `bank` (55 instancias, solo EH-SA/TS).

### Opciones

| Opción           | Efecto                                                            | Default |
| ---------------- | ----------------------------------------------------------------- | ------- |
| `--seed N`       | Semilla base de EH-SA/TS; en `-large` deriva semillas por corrida | `42`    |
| `--runs N`       | Número de corridas independientes en `-large`                     | `10`    |
| `--time-limit N` | Límite en segundos del MIP OR-Tools por instancia en `-small`     | `300`   |
| `--help` / `-h`  | Resumen de modos en consola (no crea log)                         | —       |

En `-large`, las semillas son `base_seed + i × 9973` para `i = 0 … N-1` (reproducibilidad entre corridas).

---

## Escenarios de prueba y métricas

Las métricas siguen la notación de Zhang et al. (2018). En este proyecto, el solver de referencia en instancias pequeñas es **OR-Tools (MIP)** en lugar de CPLEX; en instancias grandes el paper compara varios métodos (p. ej. AC y ALNS), pero aquí solo se ejecuta **EH-SA/TS** varias veces, de modo que el RPD se calcula **entre runs del mismo algoritmo** (variabilidad y robustez), no frente a un segundo método.

### Instancias pequeñas (`-small`)

**Qué se ejecuta:** las 13 instancias `C12R2` … `C24R2`. Por cada una se corre EH-SA/TS y, si OR-Tools está instalado vía `requirements.txt`, un modelo MIP que actúa como referencia de optimalidad.

**Qué debe aparecer en el log** (`logs/run_NNN_small.txt`):

- Energía del MIP (kWh), tiempo (`t_MIP`) y estado (óptimo / factible con límite de tiempo / infactible).
- Energía de EH-SA/TS (kWh) y tiempo (`t_EH`), o la etiqueta **`INFACTIBLE`** si la solución no cumple todas las restricciones.
- **Absolute Gap (EH)** respecto al MIP (solo si EH es factible y el MIP entregó referencia):

$$\text{Gap} = \frac{E_{\text{EH}} - E_{\text{ref}}}{E_{\text{ref}}} \times 100\%$$

donde \(E\_{\text{ref}}\) es la energía del MIP. Valores **positivos** indican que EH-SA/TS consume más energía que la referencia (comportamiento esperado frente a un óptimo), un gap negativo con EH factible sería inconsistente y suele indicar un error de validación.

- Si EH es infactible, línea de diagnóstico con clientes servidos, faltantes y violaciones de batería/capacidad.
- Tabla resumen por instancia y promedios al final (`--- Resumen comparativo (modo -small) ---`), incluyendo conteo `EH factibles: X/13`.

Si OR-Tools no está instalado (`python -m pip install --user -r requirements.txt`), EH-SA/TS igual corre; la columna Gap muestra `---` y el resumen indica la instalación faltante.

### Instancias grandes (`-large`)

**Qué se ejecuta:** el banco grande `C25` … `C150` (variantes R2/R4/R6/R8 y sufijos `-1`/`-2` cuando aplica). Por defecto **10 corridas** de EH-SA/TS con **semillas distintas** por instancia.

**Qué debe aparecer en el log** (`logs/run_NNN_large.txt`):

| Métrica                 | Definición (solo runs EH-SA/TS)                                                                                                                         |
| ----------------------- | ------------------------------------------------------------------------------------------------------------------------------------------------------- |
| **B\***                 | Mejor energía entre las N corridas: \(\min_i E_i\)                                                                                                      |
| **Media**               | Promedio de energía (kWh) sobre las N corridas                                                                                                          |
| **Desviación estándar** | Dispersión de energía entre corridas                                                                                                                    |
| **RPD por corrida**     | \(\text{RPD}\_i = (E*i - B^*) / B^\_ \times 100\%\) (misma fórmula relativa que en el paper; aquí la referencia es la mejor corrida propia, no AC/ALNS) |
| **RPD media / RPD máx** | Promedio y máximo de los RPD por instancia                                                                                                              |
| Detalle                 | Energías `R1=… R2=…` y RPD `%` por corrida                                                                                                              |

### Pipeline completo (`-all`)

Un solo log (`run_NNN_all.txt`) con los cinco bloques anteriores en orden, útil para reproducir el experimento de una vez.

### `recharge-stations`

EH-SA/TS sobre todas las instancias grandes, agrupadas por número de estaciones **R2, R4, R6, R8**.

**Métricas:** energía media (kWh) y visitas medias a estaciones por grupo, con conteo `n` de instancias por grupo.

### `battery-reserve`

Mismo banco grande con tres niveles de **reserva mínima de batería**: 0 %, 10 % y 20 % (capacidad usable 110 / 99 / 88 kWh).

**Métricas:** energía media y visitas medias a estaciones por nivel de reserva.

### `energy-vs-distance`

Por instancia: minimización de **energía** (EH-SA/TS estándar) frente a una variante que prioriza **distancia** y luego evalúa energía.

**Métricas:** \(E*{\min}\), \(E*{\text{dist}}\) y **% de incremento** de energía al usar la solución orientada a distancia:

$$\% = \frac{E_{\text{dist}} - E_{\min}}{E_{\min}} \times 100\%$$

---
