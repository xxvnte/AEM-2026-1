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

La solución inicial se construye en dos fases. En la primera, se aplica un procedimiento _greedy_ aleatorio que asigna **todos** los clientes a rutas (solo capacidad; sin batería). En la segunda, la Programación Dinámica de la Capa 3 inserta el plan de recarga sobre cada ruta. El objetivo es entregar al algoritmo principal una solución con **todos los clientes presentes en la secuencia** y un plan de recarga coherente; la factibilidad estricta (batería en cada arco, capacidad, visita única) se verifica al reportar resultados.

```
Fase 1: greedy aleatorio → todas las rutas de clientes (capacidad sí, batería no)
Fase 2: DP + greedy de estaciones por ruta
        → nunca se omite un cliente de la secuencia
        → si un tramo no es alcanzable localmente: retorno al depósito
          y continuación del sufijo (recarga completa)
→ SA parte con la misma cobertura de clientes que la fase 1
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

Si en algún paso no quedan etiquetas válidas, se activa un **greedy de respaldo** con la misma lógica de lookahead (Pareto en tramos, estaciones en dos saltos, detours y retorno al depósito). Si el greedy no puede enlazar $N_{i-1} \rightarrow N_i$ con ningún mecanismo local, **no se salta $N_i$**, se cierra el prefijo construido, se reinicia en el depósito con batería llena y se procesa recursivamente el **sufijo** $depot \rightarrow N_i \rightarrow \ldots \rightarrow depot$.

Reglas de implementación (Capa 3):

| Regla            | Descripción                                                                                                                  |
| ---------------- | ---------------------------------------------------------------------------------------------------------------------------- |
| Cobertura        | Toda secuencia de entrada conserva el mismo conjunto de clientes en la salida                                                |
| Avance           | Solo se avanza al siguiente cliente si el camino construido **termina** en ese nodo                                          |
| Retorno depósito | Tramo local $N_{i-1} \to N_i$ con recarga en depósito/estaciones, si falla, **re-enganche por depósito** del sufijo restante |
| Último recurso   | Devolver la secuencia de clientes sin estaciones solo si el re-enganche también falla (penalizado en $f_{gen}$)              |

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
  greedy de estaciones (mismas reglas de cobertura)
  Si falla el enlace local: re-enganche depot + sufijo restante

Salida: plan de recarga + coste energético f_e(R)  (mismos clientes que la entrada)
```

La DP se invoca sobre cada ruta modificada **antes** de que SA decida aceptar o rechazar el movimiento de TS, aplicándose únicamente sobre las rutas afectadas. Si SA rechaza el movimiento, el resultado del DP se descarta. Esto favorece evaluar $f_{gen}$ sobre un plan de recarga coherente.

---

## Funciones de Evaluación

El EH-SA/TS opera con tres niveles de evaluación con distintos propósitos y costes computacionales:

| Función                                                                                     | Cuándo se usa                                                     | Propósito                                                                                |
| ------------------------------------------------------------------------------------------- | ----------------------------------------------------------------- | ---------------------------------------------------------------------------------------- |
| $f'_{approx}(S)$                                                                            | Evaluación de cada vecino en TS                                   | Evaluación rápida, evita recálculo en cascada                                            |
| $f_{gen}(S) = f_e + \gamma_{cap} L_{cap} + \gamma_{batt} L_{batt} + \gamma_{miss} L_{miss}$ | Criterio de aceptación SA sobre el movimiento elegido **post-DP** | Guía la búsqueda y penaliza violaciones de capacidad/batería y **clientes no visitados** |
| $f_e$ exacta (con penalizaciones)                                                           | Dentro de $f_{gen}$ durante el loop híbrido                       | Coste energético simulado tras reoptimizar el plan de recarga                            |
| **Validación estricta**                                                                     | Al finalizar (`-small`, `-large`, etc.)                           | Comprueba factibilidad real antes de reportar energía                                    |

**Búsqueda vs reporte.** Durante SA/TS, $f_{gen}$ puede explorar soluciones con penalizaciones ($L_{cap}$, $L_{batt}$, $L_{miss}$) para no bloquear la exploración, $L_{miss}$ cuenta clientes faltantes y visitas duplicadas con un peso alto ($\gamma_{miss}$) para que la búsqueda no prefiera rutas incompletas. La **energía EH-SA/TS reportada** en logs solo se muestra si la solución cumple simultáneamente:

- todos los clientes visitados **exactamente una vez**;
- capacidad del vehículo respetada en cada ruta;
- batería $\geq 0$ en **cada arco** (simulación estricta, sin “resetear” violaciones).

Si alguna condición falla, la instancia se marca como **`INFACTIBLE`**. La comparación con el óptimo MIP (CPLEX) se hace aparte leyendo `logs/run_NNN_small_cplex.txt`.

**Nota:** En la implementación, el DP se invoca sobre el movimiento elegido por TS antes de que SA tome la decisión de aceptar o rechazar. Si SA rechaza el movimiento, el trabajo del DP de esa iteración se descarta. Dado que el DP se aplica únicamente sobre las rutas afectadas por el movimiento, el costo computacional adicional es acotado. La ventaja de esta estrategia es que $f_{gen}$ opera sobre un plan de recarga reoptimizado en lugar de uno aproximado, mejorando la calidad de la comparación en SA.

La función sustituta se construye como:

$$f'*{approx}(S) = f'e(S) + \gamma{cap} \cdot L*{cap}(S) + \gamma_{batt} \cdot L'_{batt}(S)$$

$$f_{gen}(S) = f_e(S) + \gamma_{cap} L_{cap}(S) + \gamma_{batt} L_{batt}(S) + \gamma_{miss} L_{miss}(S)$$

donde $L_{cap}(S)$ no requiere versión sustituta porque la capacidad total del vehículo no depende del orden de recargas y se calcula directamente, y $L_{miss}(S)$ penaliza clientes no incluidos en las rutas (la función sustituta $f'_{approx}$ no recalcula $L_{miss}$, los movimientos Relocate/Exchange conservan el conjunto de clientes).

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
13:     Greedy de respaldo (sin omitir clientes)
14:     Si falla enlace local Ni-1 → Ni: re-enganche depot + sufijo restante
15: Retornar etiqueta de menor c en L(n+1) y plan de recarga asociado
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

Los valores de los parámetros se calibran con base en la literatura y en el comportamiento observado sobre el banco de instancias del paper (Zhang et al., 2018).

| Parámetro                    | Descripción                                                        | Valor  |
| ---------------------------- | ------------------------------------------------------------------ | ------ |
| $T_0$                        | Temperatura inicial SA                                             | 20     |
| $\alpha$                     | Tasa de enfriamiento geométrico                                    | 0.97   |
| $T_{min}$                    | Temperatura mínima de parada                                       | 0.01   |
| $K_{tabú}$                   | Tenencia tabú (iteraciones de prohibición)                         | 12     |
| `MAX_LABELS_DP`              | Máximo de etiquetas Pareto por paso en la DP                       | 20     |
| `MAX_NEIGHBORHOOD_MOVES`     | Cap duro del vecindario por iteración SA/TS                        | 2000   |
| `MAX_DIJKSTRA_POPS`          | Cota dura de expansiones por búsqueda Dijkstra                     | 200000 |
| `--time-limit-run` (default) | Segundos máximos por corrida EH en `-large` y extras               | 300    |
| `INIT_TIME_MAX_S`            | Tope absoluto de tiempo para la fase inicial (greedy + DP)         | 90     |
| `INIT_TIME_FRACTION`         | Fracción del límite por corrida reservada solo a la inicialización | 0.30   |
| $\gamma_{cap}$               | Factor de penalización por violación de capacidad                  | 200    |
| $\gamma_{batt}$              | Factor de penalización por violación de batería                    | 100    |
| $\gamma_{miss}$              | Penalización por cliente no visitado (o duplicado)                 | 5000   |
| umbral                       | Iteraciones consecutivas sin mejora global para parada             | 100    |

### Justificación de $T_0 = 20$ y $\alpha = 0.97$

La **temperatura inicial** $T_0 = 20$ se elige de modo que, en la primera iteración, movimientos que empeoran la función objetivo en ~20 kWh se acepten con probabilidad $e^{-20/20} \approx 0.37$. Dado que la escala de las soluciones factibles del banco de instancias oscila entre 500 y 3000 kWh, esto representa un umbral de aceptación del orden del 0.5–4 % de la energía total, lo que equilibra exploración inicial con evitar aceptar soluciones muy malas.

La **tasa de enfriamiento** $\alpha = 0.97$ produce el siguiente horizonte de iteraciones antes de que la temperatura sea inferior a $T_{min} = 0.01$:

$$N_{iter} = \left\lfloor \frac{\ln(T_{min}/T_0)}{\ln \alpha} \right\rfloor = \left\lfloor \frac{\ln(0.0005)}{\ln(0.97)} \right\rfloor \approx 248 \text{ iteraciones}$$

Esto otorga ~250 iteraciones de búsqueda (sin contar el criterio de parada por `umbral`). Para instancias pequeñas (C25, n≤25) este horizonte resulta suficiente para converger; para instancias grandes (C75–C150) la condición de `umbral=100` (sin mejora) suele activarse antes de que la temperatura llegue a $T_{min}$, actuando como criterio de parada anticipada. Este diseño es consistente con el rango de 200–300 iteraciones reportado en Küçükoğlu et al. (2019) para tamaños de problema similares.

### Justificación de $K_{tabú} = 12$

La **tenencia tabú** determina cuántas iteraciones un par (cliente, ruta de origen) permanece prohibido después de un movimiento. En TABUROUTE (Gendreau et al., 1994), el rango típico es $K_{tabú} \in [5, 15]$ con $K=7$ como referencia base. Para el EVRP el espacio de soluciones es mayor debido a la dimensión de las decisiones de recarga, lo que incrementa el riesgo de ciclado. El valor $K_{tabú} = 12$ se ubica en el cuartil superior de ese rango, lo que:

- Prolonga suficientemente la prohibición para evitar ciclos de corto período.
- Sigue siendo corto en comparación con el horizonte total de iteraciones (~250), de modo que la lista tabú no bloquea indefinidamente opciones buenas.
- Es consistente con los valores de tenencia de 10–15 reportados por Küçükoğlu et al. (2019) para problemas EVRP de tamaño comparable.

### `MAX_LABELS_DP = 20` y su impacto en velocidad

El frente Pareto de la DP crece en el peor caso como $O(k \times f)$ por paso, donde $k$ es el número de etiquetas anteriores y $f$ el número de opciones de segmento. Sin la cota, para instancias C75–C150 la verificación de dominancia entre etiquetas puede alcanzar $O(k^2)$ comparaciones por paso con $k$ grande. Con `MAX_LABELS_DP=20` se pre-filtra a $40$ candidatos antes del chequeo Pareto y luego se recorta a las $20$ mejores por coste, limitando el tiempo por paso a $O(1600)$ comparaciones. El límite se redujo de 30 a 20 tras confirmar que las rutas de C75–C150 tienen más clientes por ruta y la precomputación de `_segment_need_end_battery` (24 llamadas Dijkstra por paso) hace que cada paso de DP sea costoso; con 20 etiquetas la calidad de recarga es prácticamente idéntica ya que el espacio Pareto relevante en R2 es pequeño (solo 2 estaciones). Adicionalmente, el DP ahora precomputa los valores `need_end` y `seg_nodes` fuera del bucle principal de etiquetas, y verifica el deadline en cada paso del precomputo y del DP; si el tiempo se agota se cae automáticamente al greedy de inserción rápida.

### `MAX_NEIGHBORHOOD_MOVES = 2000` - cap duro del vecindario

Con 75 clientes, el vecindario generado (RELOCATE + EXCHANGE + 2-OPT\*) puede superar los 15.000 movimientos. Sin cap, la evaluación de deltas por sí sola consume segundos por iteración. El cap duro de 2000 movimientos (tras el shuffle aleatorio que ya existía al 15 %) garantiza que la evaluación del vecindario no domina el presupuesto de tiempo por corrida, dejando margen para las llamadas DP de la fase de aceptación.

### Límite de tiempo por corrida (`--time-limit-run`, default 300 s)

El valor por defecto pasó de **120 s a 300 s** (5 min) para dar margen suficiente a instancias C75–C150 con varias estaciones de recarga (R4/R6/R8), donde en pruebas internas se obtienen soluciones **factibles** dentro de ese presupuesto (p. ej. C75R8-1 en ~40 s, C50R8-2 en ~27 s). Instancias C25/C50 suelen terminar mucho antes y no consumen los 300 s completos.

**Reparto del presupuesto:** la fase inicial (greedy + DP de estaciones) usa como máximo `min(90 s, 30 % del límite)`; el resto queda para el bucle SA/TS hasta el límite total. Esto evita que la inicialización monopolice todo el tiempo en instancias difíciles (C75R2 con 2 estaciones lejanas).

| Modo / caso                        | Máximo teórico por instancia (default 300 s)          |
| ---------------------------------- | ----------------------------------------------------- |
| `-large` (1 run)                   | ~300 s                                                |
| `-large` (10 runs, p. ej. C75R2-1) | ~50 min                                               |
| `recharge-stations`                | ~300 s                                                |
| `battery-reserve` (3 niveles)      | ~15 min                                               |
| `energy-vs-distance`               | ~300 s (solo EH; el solver de distancia es adicional) |

Peor caso del banco `-large` completo (50 instancias × 10 runs × 300 s): ~42 h si todas llegaran al límite; en la práctica C25/C50 terminan antes y el total es menor.

### Deadline global e interrupción dura de la búsqueda

El límite se aplica mediante un **deadline global** verificado en `segment_path_options`, punto único por el que pasan todas las búsquedas Dijkstra costosas (DP, `min_start_battery_for_segment`, reparación y greedy). Al agotarse el tiempo se lanza `_DeadlineReached`, que desenrolla la pila al instante y devuelve la mejor solución hasta ese momento.

Como defensa adicional, `_segment_path_search_core` limita cada búsqueda a `MAX_DIJKSTRA_POPS = 200 000` expansiones.

> **C75R2 / C100R2 / C150R2 (solo 2 estaciones):** siguen siendo las más difíciles; con semilla fija pueden quedar como `INF` incluso con 300 s. El límite garantiza que **no se cuelgan** y que el banco avanza. Para forzar más exploración en esos casos: `--time-limit-run 0` (sin límite) o variar `--seed`.

---

## Requisitos

- **Python 3**
- Dependencias del proyecto (`requirements.txt`)
- **CPLEX vía amplpy** (solo necesario para el modo `-small`, como referencia óptima frente a EH-SA/TS; ver [Instancias pequeñas](#instancias-pequeñas--small))

### 1. Dependencias Python

Desde la carpeta `project`:

```bash
cd project
python -m pip install --user -r requirements.txt
```

### 2. Solver CPLEX (referencia para `-small`)

CPLEX resuelve el MIP de energía en instancias pequeñas y sirve como **punto de referencia** para comparar EH-SA/TS (Gap, mapas, `comparative_NNN_small.txt`). No se ejecuta desde `main.py` sino que se usa `solve_cplex.py` por separado.

```bash
# 1. Instalar la librería para Python
python -m pip install amplpy --upgrade

# 2. Instalar el módulo de CPLEX
python -m amplpy.modules install cplex

# 3. Activar licencia (UUID de ampl.com/ce o ampl.com/courses)
python -m amplpy.modules activate <id-licencia>
```

Comprobar instalación:

```bash
python -c "from amplpy import AMPL; ampl = AMPL(); print('AMPL OK')"
```

`requirements.txt` ya incluye `amplpy`, los pasos 2 y 3 configuran el solver y la licencia.

---

## Ejecución

Todos los comandos se ejecutan desde `project/`. Cada corrida guarda log y JSON en `logs/run_NNN_<modo>.txt` y `logs/run_NNN_<modo>.json` (NNN = 001, 002, …).

### Ejecución de experimentos

Ejecutar en este orden (o por separado):

```bash
cd project
python main.py -small
python main.py -large
python main.py recharge-stations
python main.py battery-reserve
python main.py energy-vs-distance
```

| Modo                 | Comando                             | Qué hace                                               |
| -------------------- | ----------------------------------- | ------------------------------------------------------ |
| Pequeñas             | `python main.py -small`             | 15 instancias C10R2–C24R2 con EH-SA/TS                 |
| Grandes              | `python main.py -large`             | 50 instancias C25–C150, 10 runs EH-SA/TS por instancia |
| Estaciones           | `python main.py recharge-stations`  | Análisis por número de estaciones (R2/R4/R6/R8)        |
| Reserva batería      | `python main.py battery-reserve`    | Análisis con reserva 0 % / 10 % / 20 %                 |
| Energía vs distancia | `python main.py energy-vs-distance` | Comparación minimización energía vs distancia          |

Opciones útiles: `--seed N` (default 42), `--runs N` (solo `-large`, default 10), `--time-limit-run N` (default 300 s). Ver tabla completa en [Opciones de línea de comandos](#opciones-de-línea-de-comandos).

Para ejecutar los cinco modos seguidos: `python main.py -all`.

### Gráficos (`stats.py`)

Tras cada corrida, genera los gráficos con `stats.py` a partir del JSON guardado (en `-small` también puede leer el log `.txt`). Sustituye `001` por el número de corrida (`run_001_…`).

```bash
python stats.py -small 001
python stats.py -large 001
python stats.py recharge-stations 001
python stats.py battery-reserve 001
python stats.py energy-vs-distance 001
```

Si se omite el número, `stats.py` usa el último run disponible.

#### Gráficos generados por modo

##### `-small` → `python stats.py -small 001`

| Archivo                               | Contenido                                                                |
| ------------------------------------- | ------------------------------------------------------------------------ |
| `run_001_small_bars.png`              | Energía EH-SA/TS vs OR-Tools por instancia                               |
| `run_001_small_gaps.png`              | Gap (%) EH vs CPLEX por instancia                                        |
| `run_001_small_time_bars.png`         | Tiempo de ejecución por instancia                                        |
| `run_001_small_iter_bars.png`         | Iteraciones SA/TS por instancia                                          |
| `run_001_small_evo_iter_g1/g2/g3.png` | Evolución FO (best + current) vs iteraciones, grupos de 5                |
| `run_001_small_evo_time_g1/g2/g3.png` | Evolución FO vs tiempo, grupos de 5                                      |
| `run_001_small_map_g1/g2/g3.png`      | Mapas de rutas agrupados (5 instancias/PNG)                              |
| `comparative_001_small.txt`           | Tabla comparativa EH-SA/TS vs CPLEX (requiere `run_001_small_cplex.txt`) |

**Interpretación:** barras y gaps miden calidad y distancia al óptimo MIP; evolución muestra convergencia de SA/TS; mapas comparan rutas EH (naranja) vs referencia (morada).

##### `-large` → `python stats.py -large 001`

| Archivo                            | Contenido                                                        |
| ---------------------------------- | ---------------------------------------------------------------- |
| `run_001_large_rpd_bars.png`       | RPD medio (%) por instancia vs B\*                               |
| `run_001_large_energy_bars.png`    | B\* y energía media por instancia                                |
| `run_001_large_evo_iter_g<N>.png`  | Evolución FO vs iteraciones, grupos de 5                         |
| `run_001_large_evo_time_g<N>.png`  | Evolución FO vs tiempo, grupos de 5                              |
| `run_001_large_map_3runs_g<N>.png` | Mapas agrupados: mejor / representativo / peor run por instancia |

**Interpretación:** RPD indica dispersión entre las 10 corridas; energía resume B\* y media; evolución y mapas muestran estabilidad y calidad de rutas en instancias grandes.

##### `recharge-stations` → `python stats.py recharge-stations 001`

| Archivo                                              | Contenido                           |
| ---------------------------------------------------- | ----------------------------------- |
| `run_001_recharge-stations_recharge_energy_bars.png` | Energía media por grupo R2/R4/R6/R8 |
| `run_001_recharge-stations_recharge_visits_bars.png` | Visitas a estaciones por grupo      |
| `run_001_recharge-stations_recharge_bars.png`        | Ambas series combinadas             |

**Interpretación:** más estaciones suele reducir energía y visitas forzadas; barras `inf` o vacías indican instancias sin solución factible en ese grupo.

##### `battery-reserve` → `python stats.py battery-reserve 001`

| Archivo                                           | Contenido                                     |
| ------------------------------------------------- | --------------------------------------------- |
| `run_001_battery-reserve_battery_energy_bars.png` | Energía media por nivel de reserva 0%/10%/20% |
| `run_001_battery-reserve_battery_visits_bars.png` | Visitas a estaciones por nivel de reserva     |
| `run_001_battery-reserve_battery_bars.png`        | Ambas series combinadas                       |

**Interpretación:** mayor reserva implica menos batería usable y, en general, más energía y más paradas en estaciones.

##### `energy-vs-distance` → `python stats.py energy-vs-distance 001`

| Archivo                                         | Contenido                                   |
| ----------------------------------------------- | ------------------------------------------- |
| `run_001_energy-vs-distance_evd_bars_g1–g4.png` | E_min vs E_dist por grupo de 10 instancias  |
| `run_001_energy-vs-distance_evd_pct_g1–g4.png`  | % incremento energía al minimizar distancia |
| `run_001_energy-vs-distance_evd_time_g1–g4.png` | Tiempo de ejecución EH por grupo            |

**Interpretación:** si E_dist > E_min, minimizar distancia empeora la energía; el % positivo cuantifica ese sobrecoste. Valores `nan` o barras vacías = al menos una corrida infactible.

### Opciones de línea de comandos

| Opción               | Efecto                                                                         | Default |
| -------------------- | ------------------------------------------------------------------------------ | ------- |
| `--seed N`           | Semilla base de EH-SA/TS; en `-large` deriva semillas por corrida              | `42`    |
| `--runs N`           | Número de corridas independientes en `-large`                                  | `10`    |
| `--time-limit-run N` | Segundos máximos por corrida en `-large` y experimentos extra (0 = sin límite) | `300`   |
| `--help` / `-h`      | Resumen de modos en consola (no crea log)                                      | —       |

En `-large`, las semillas son `base_seed + i × 9973` para `i = 0 … N-1`.

---

## Escenarios de prueba y métricas

Las métricas siguen la notación de Zhang et al. (2018).

- **Pequeñas:** `main.py -small` solo ejecuta **EH-SA/TS** (`logs/run_NNN_small.txt`). La referencia MIP óptima es **CPLEX** vía `solve_cplex.py` (`logs/run_NNN_small_cplex.txt`). El análisis comparativo (p. ej. Absolute Gap) se hace cruzando ambos logs.
- **Grandes:** solo **EH-SA/TS** con varias corridas (runs); el RPD se calcula **entre runs del mismo algoritmo**.
- **Estaciones de recarga:** una corrida EH-SA/TS por instancia grande, agregada por grupos R2/R4/R6/R8.
- **Reserva de batería:** el banco grande repetido con reserva 0 % / 10 % / 20 %.
- **Energía vs distancia:** por instancia, EH minimizando energía frente a una variante greedy minimizando distancia.

### Instancias pequeñas (`-small`)

**Qué se ejecuta:** las 15 instancias `C10R2` … `C24R2` con EH-SA/TS únicamente (incluye las dos primeras del banco pequeño del paper).

**Log EH-SA/TS** (`logs/run_NNN_small.txt`):

- Energía EH-SA/TS (kWh), tiempo, rutas, visitas a estaciones, o **`INFACTIBLE`** con diagnóstico (clientes faltantes, violaciones).
- Resumen al final (`--- Resumen (modo -small) ---`) y `EH factibles: X/15`.

**Log referencia CPLEX** (`logs/run_NNN_small_cplex.txt`, script aparte):

- Energía MIP (kWh), tiempo de solver, estado (óptimo / factible / infactible) por instancia.
- Línea `Rutas CPLEX <inst>: [...]` por instancia (para mapas en `stats.py -small`).
- Tabla resumen al final del batch CPLEX.

**Gap (análisis manual):** con energía factible de EH y referencia CPLEX,

$$\text{Gap} = \frac{E_{\text{EH}} - E_{\text{ref}}}{E_{\text{ref}}} \times 100\%$$

Valores positivos indican que EH-SA/TS consume más que el óptimo MIP.

**Ejecutar CPLEX** (tras instalar según [Requisitos](#requisitos)):

```bash
cd project
python solve_cplex.py --build-small-dat   # genera instances/small_dat/*.dat (una vez)
python solve_cplex.py                     # batch C10R2–C24R2 → logs/run_NNN_small_cplex.txt
```

CPLEX usa `mipgap=0.0001` (0,01 %) sin límite de tiempo. Archivos: `model.mod`, `solve_cplex.py`, `instances/small_dat/<nombre>.dat`.

Los gráficos y su interpretación están en [Gráficos (`stats.py`)](#gráficos-statspy) (sección Ejecución). Leyenda en mapas: depósito (cuadrado rojo), estaciones (triángulo verde), clientes (círculo azul); referencia CPLEX/OR-Tools (morada continua); EH-SA/TS (naranja discontinua).

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

### Estaciones de recarga (`recharge-stations`)

**Qué se ejecuta:** las **40 instancias grandes** (`C25` … `C150`, variantes R2/R4/R6/R8 y sufijos `-1`/`-2`). Una corrida EH-SA/TS por instancia (semilla base, límite por defecto 300 s). Al final se agregan los resultados en cuatro grupos según el número de estaciones de recarga: **R2, R4, R6, R8** (10 instancias por grupo).

**Qué debe aparecer en el log** (`logs/run_NNN_recharge-stations.txt`):

- Detalle por instancia: energía EH-SA/TS (kWh) o **`INFACTIBLE`**, tiempo, visitas a estaciones; marca `(TL)` si alcanza el límite de tiempo.
- Tabla resumen al final con una fila por grupo:

| Métrica     | Definición                                                                    |
| ----------- | ----------------------------------------------------------------------------- |
| **Energía** | Media de energía (kWh) en el grupo; instancias infactibles cuentan como `inf` |
| **VisEst**  | Media de visitas a estaciones de recarga en el grupo                          |
| **n**       | Número de instancias del grupo (10 por R2/R4/R6/R8)                           |

**Interpretación:** compara cómo cambia el consumo y la frecuencia de recarga al variar la densidad de estaciones. Gráficos en [Gráficos (`stats.py`)](#gráficos-statspy) → `recharge-stations`.

### Reserva de batería (`battery-reserve`)

**Qué se ejecuta:** el mismo banco de **40 instancias grandes**, repetido para **tres niveles de reserva mínima** de batería al salir de cada estación. En total son 120 corridas EH-SA/TS (40 instancias × 3 niveles), con la misma semilla base y límite de 300 s por corrida.

| Nivel | Reserva                      | Capacidad usable |
| ----- | ---------------------------- | ---------------- |
| `0%`  | Sin reserva extra            | 110 kWh          |
| `10%` | 10 % de la batería reservada | 99 kWh           |
| `20%` | 20 % reservada               | 88 kWh           |

**Qué debe aparecer en el log** (`logs/run_NNN_battery-reserve.txt`):

- Detalle por instancia con prefijo de nivel (`[0%]`, `[10%]`, `[20%]`): energía o **`INFACTIBLE`**, tiempo, visitas a estaciones.
- Tabla resumen con una fila por nivel:

| Métrica     | Definición                                                                        |
| ----------- | --------------------------------------------------------------------------------- |
| **Usable**  | Capacidad efectiva (kWh) tras aplicar la reserva                                  |
| **Energía** | Media de energía (kWh) sobre las 40 instancias; solo se promedian valores finitos |
| **VisEst**  | Media de visitas a estaciones en ese nivel                                        |

**Interpretación:** cuantifica el coste energético de exigir más margen de batería (más paradas y mayor consumo). Gráficos en [Gráficos (`stats.py`)](#gráficos-statspy) → `battery-reserve`.

### Energía vs distancia (`energy-vs-distance`)

**Qué se ejecuta:** las **40 instancias grandes**, una por una. Por instancia se comparan dos enfoques:

1. **E_min** — EH-SA/TS estándar minimizando energía (semilla base, límite 300 s).
2. **E_dist** — construcción greedy orientada a **distancia** + inserción DP de estaciones; se evalúa la energía resultante (semilla `base + 1`, sin límite de tiempo adicional).

**Qué debe aparecer en el log** (`logs/run_NNN_energy-vs-distance.txt`):

| Métrica    | Definición                                                               |
| ---------- | ------------------------------------------------------------------------ |
| **E_min**  | Mejor energía factible de EH-SA/TS (kWh), o `inf` si no hay solución     |
| **E_dist** | Energía (kWh) de la ruta construida minimizando distancia                |
| **% inc**  | Incremento relativo de energía al usar la solución orientada a distancia |
| **t(s)**   | Tiempo total de ambas fases por instancia                                |

**% de incremento:**

$$\% = \frac{E_{\text{dist}} - E_{\min}}{E_{\min}} \times 100\%$$

Valores **positivos** indican que minimizar distancia consume más energía que minimizarla directamente. Valores **negativos** o `nan%` aparecen cuando \(E\_{\min}\) es infactible o la comparación no es válida. Al final del log, fila **Promedio** con la media del % sobre las 40 instancias.

**Interpretación:** mide el sobrecoste energético de priorizar distancia en lugar de energía. Gráficos agrupados (10 instancias/PNG) en [Gráficos (`stats.py`)](#gráficos-statspy) → `energy-vs-distance`.

### Pipeline completo (`-all`)

Un solo log (`run_NNN_all.txt`) con los cinco bloques anteriores en orden (`-small` → `-large` → `recharge-stations` → `battery-reserve` → `energy-vs-distance`), para reproducir el experimento completo de una vez.
