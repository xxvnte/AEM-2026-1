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

La solución inicial se construye en dos fases. En la primera, se aplica un procedimiento _greedy_ aleatorio que asigna clientes a vehículos construyendo rutas sin considerar las restricciones de batería. En la segunda, la Programación Dinámica de la Capa 3 inserta el plan de recarga óptimo inicial sobre cada ruta construida, entregando al algoritmo principal una solución estructuralmente válida. Este mecanismo conecta directamente con la Capa 3, asegurando que el EH-SA/TS parta siempre de una solución con un plan de recarga consistente y óptimo para la secuencia inicial de clientes.

```
Fase 1: greedy aleatorio → construir rutas asignando clientes a vehículos
        (sin considerar batería aún)
Fase 2: DP inserta plan de recarga inicial sobre cada ruta
→ SA parte de una solución estructuralmente válida
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

Dado que el modelo asume recarga completa al visitar una estación, la DP no decide cuánta energía cargar sino **dónde y cuándo visitar las estaciones** a lo largo de la ruta. Las etiquetas DP tienen la forma ${q,\ c}$, donde $q$ es el nivel de batería actual y $c$ es el coste energético acumulado. Para cada par de clientes consecutivos $(C_i, C_{i+1})$, la DP evalúa todos los caminos posibles: directo sin recargar y vía una o más estaciones disponibles, seleccionando el camino de menor coste energético factible.

```
Entrada: secuencia de clientes de UNA ruta
         R = (depot → C1 → C2 → ... → Cn → depot)

Etiquetas DP: {q, c}
  q → nivel de batería actual
  c → coste energético acumulado
  (sin componente de tiempo: no hay ventanas de tiempo)

Para cada par consecutivo (Ci, Ci+1):
  Evaluar todos los caminos posibles:
  - directo: ¿alcanza la batería? → calcular energía
  - vía estación F: ¿alcanza a F? ¿de F alcanza Ci+1? → calcular energía
  Seleccionar camino de menor coste energético factible

Salida: plan de recarga óptimo + coste energético f_e(R)
```

La DP se invoca sobre cada ruta modificada inmediatamente después de que SA acepta un movimiento, garantizando que el plan de recarga permanezca consistente y óptimo para la configuración actual. De esta forma, SA evalúa sus movimientos sobre una solución con el subproblema de recarga ya resuelto de forma óptima, reduciendo la variabilidad del coste entre iteraciones y estabilizando el paisaje de optimización.

En la implementación concreta, el DP se ejecuta sobre el movimiento elegido por TS inmediatamente antes de la decisión de aceptación de SA, aplicándose únicamente sobre las rutas afectadas por dicho movimiento. Si SA rechaza el movimiento, el resultado del DP se descarta y la solución actual permanece inalterada. Esta estrategia representa una decisión de implementación que favorece la calidad de la evaluación SA a expensas de un costo computacional adicional acotado por iteración.

---

## Funciones de Evaluación

El EH-SA/TS opera con tres niveles de evaluación con distintos propósitos y costes computacionales:

| Función                                                            | Cuándo se usa                                                     | Propósito                                                             |
| ------------------------------------------------------------------ | ----------------------------------------------------------------- | --------------------------------------------------------------------- |
| $f'_{approx}(S)$                                                   | Evaluación de cada vecino en TS                                   | Evaluación rápida, evita recálculo en cascada                         |
| $f_{gen}(S) = f_e + \gamma_{cap} L_{cap} + \gamma_{batt} L_{batt}$ | Criterio de aceptación SA sobre el movimiento elegido **post-DP** | Permite explorar regiones infactibles con plan de recarga consistente |
| $f_e$ exacta                                                       | Después de que DP re-optimiza cada ruta del movimiento elegido    | Coste energético real tras optimizar el plan de recarga               |

**Nota:** En la implementación, el DP se invoca sobre el movimiento elegido por TS antes de que SA tome la decisión de aceptar o rechazar. Esto implica que si SA rechaza el movimiento, el trabajo del DP de esa iteración se descarta. Sin embargo, dado que DP se aplica únicamente sobre las rutas afectadas por el movimiento (no sobre la solución completa), el costo computacional adicional es acotado. La ventaja de esta estrategia es que $f_{gen}$​ opera sobre un plan de recarga óptimo en lugar de uno aproximado, mejorando la calidad de la comparación y reduciendo aceptaciones erróneas debidas a violaciones de batería artificiales.

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

Salida:  Plan de recarga óptimo y coste energético f_e(R)

1: Inicializar etiqueta L(0) ← {q = T, c = 0}    // batería llena en depósito
2: Para i = 1 hasta n+1:
3:   Para cada etiqueta {q, c} en L(i-1):
4:     Para cada camino posible entre C(i-1) y Ci
        (directo o vía estaciones de S):
5:       Si q ≥ energía mínima requerida por el camino:
6:         Calcular {q', c'} resultante en nodo Ci
7:         Si {q', c'} no está dominado por ninguna etiqueta en L(i):
8:           Agregar {q', c'} a L(i)
9:   Eliminar etiquetas dominadas de L(i)
10: Retornar etiqueta de menor c en L(n+1) y plan de recarga asociado
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

37: Retornar S*
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

## Ejecución

Desde `project/`:

```bash
python main.py [modo] [--seed N]
```

Las instancias se leen de `instances/small/` y `instances/large/` (archivos `.txt`).

### Modos

| Modo          | Qué ejecuta                                                           |
| ------------- | --------------------------------------------------------------------- |
| `all`         | Benchmark principal + escenarios extra del paper (ver abajo)          |
| `small`       | Instancias pequeñas `C12R2` … `C24R2`                                 |
| `large`       | Instancias grandes `C25` … `C150` (variantes R2/R4/R6/R8 y `-1`/`-2`) |
| `extended`    | `C10R2`, `C11R2`                                                      |
| `bank`        | Banco completo: `C10R2` … `C24R2` + 40 grandes (55 instancias)        |
| `single NAME` | Una sola instancia (ej. `C25R2-1`)                                    |

### Escenarios extras

Complementan el análisis con los últimos tres experimentos del artículo (Zhang et al.), para su respectiva comparación. Cada modo tiene un nombre descriptivo:

| Modo                 | Qué mide                                                                                                                    |
| -------------------- | --------------------------------------------------------------------------------------------------------------------------- |
| `recharge-stations`  | Instancias con distinto número de estaciones de recarga (R2, R4, R6, R8): energía media (kWh) y visitas medias a estaciones |
| `battery-reserve`    | Mismo banco grande con reserva de batería 0 %, 10 % y 20 %: energía media y visitas medias                                  |
| `energy-vs-distance` | Mismo banco con objetivo de minimizar energía vs minimizar distancia: consumo (kWh) y % de incremento energético            |

### Logs automáticos

Cada ejecución guarda la salida de consola en `logs/`, con numeración por modo:

| Ejecución                     | 1.ª vez                         | 2.ª vez                         |
| ----------------------------- | ------------------------------- | ------------------------------- |
| `python main.py all`          | `logs/run_001_all.txt`          | `logs/run_002_all.txt`          |
| `python main.py small`        | `logs/run_001_small.txt`        | `logs/run_002_small.txt`        |
| `python main.py single C12R2` | `logs/run_001_single_C12R2.txt` | `logs/run_002_single_C12R2.txt` |

El contador es independiente por modo (`all`, `small`, `battery-reserve`, etc.).

### Opciones

- `--seed N` - semilla aleatoria (por defecto: `42`)
- `--help` / `-h` - muestra la ayuda en consola (no genera log)

Los resultados quedan en `logs/run_NNN_<modo>.txt` (salida completa de consola, numerada por modo).

### Ejemplos

```bash
# Todo el benchmark
python main.py all

# Una instancia
python main.py single C12R2

# Solo instancias pequeñas
python main.py small

# Benchmark completo con otra semilla
python main.py all --seed 123

# Escenarios extra del paper
python main.py recharge-stations
python main.py battery-reserve
python main.py energy-vs-distance
```

En modo `single`, la salida incluye energía (kWh), `f_gen`, tiempo, número de rutas y visitas a estaciones.
