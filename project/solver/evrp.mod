# ===================================================
# evrp.mod — EVRP Exact MILP Model
# ===================================================
# Electric Vehicle Routing Problem with Recharging Stations
# Minimize total energy consumption (kWh)
#
# Based on: Zhang, Gajpal, Appadoo & Abdulkader (2018)
# Int. J. Production Economics, 203, 404-413
#
# Linearized energy formula (EXACT decomposition):
#   E_ij,k = c_fixed[i,j] * x[i,j,k] + c_load[i,j] * f[i,j,k]
# where:
#   c_fixed[i,j] = eta_d * eta_m * (alpha * W_kg * d_m + beta * s_ms^2 * d_m) / 3600000
#   c_load[i,j]  = eta_d * eta_m * alpha * 1000 * d_m / 3600000
#
# Station copies allow each physical station to be visited
# multiple times (standard EVRP literature technique).
# ===================================================

# ==================== SETS ====================

set NODES;                              # All node indices
set CUSTOMERS within NODES;             # Customer nodes
set STATIONS within NODES;              # Station copy nodes (recharging)
set VEHICLES ordered;                   # Vehicle indices (ordered for symmetry breaking)

param depot_start integer;              # Depot origin node
param depot_end integer;                # Depot destination node (virtual)

# Derived sets
set INTERMEDIATE = CUSTOMERS union STATIONS;

# Valid arcs: no self-loops, no outgoing from depot_end, no incoming to depot_start
set ARCS = {i in NODES, j in NODES :
    i != j and i != depot_end and j != depot_start};

# ==================== PARAMETERS ====================

param Q >= 0;                                   # Vehicle capacity (tons)
param B >= 0;                                   # Battery capacity (kWh)
param demand {NODES} >= 0 default 0;            # Node demand (tons)
param c_fixed {NODES, NODES} >= 0 default 0;    # Fixed energy cost per arc (kWh)
param c_load  {NODES, NODES} >= 0 default 0;    # Variable energy cost per arc (kWh/ton)
param orig_id {NODES} >= 0 default 0;           # Mapping to C++ internal node ID (for display)

# Auxiliary
param N_inter = card(INTERMEDIATE);

# ==================== VARIABLES ====================

var x {ARCS, VEHICLES} binary;                          # 1 if vehicle k traverses arc (i,j)
var f {ARCS, VEHICLES} >= 0;                             # Cargo load on arc (i,j) by vehicle k (tons)
var y {NODES, VEHICLES} >= 0, <= B;                      # Battery level upon departure (kWh)
var u {INTERMEDIATE, VEHICLES} >= 1, <= N_inter;         # MTZ position variable

# ==================== OBJECTIVE ====================

minimize TotalEnergy:
    sum {(i,j) in ARCS, k in VEHICLES}
        (c_fixed[i,j] * x[i,j,k] + c_load[i,j] * f[i,j,k]);

# ==================== CONSTRAINTS ====================

# ---------- Routing ----------

# C1: Each customer is visited exactly once (by exactly one vehicle)
subject to Visit {c in CUSTOMERS}:
    sum {(i,c) in ARCS, k in VEHICLES} x[i,c,k] = 1;

# C2: Flow conservation at intermediate nodes (customers and stations)
subject to FlowCons {n in INTERMEDIATE, k in VEHICLES}:
    sum {(i,n) in ARCS} x[i,n,k] = sum {(n,j) in ARCS} x[n,j,k];

# C3: Each vehicle departs from depot_start at most once
subject to DepotOut {k in VEHICLES}:
    sum {(depot_start,j) in ARCS} x[depot_start,j,k] <= 1;

# C4: Vehicle returns to depot_end iff it departed from depot_start
subject to DepotIn {k in VEHICLES}:
    sum {(i,depot_end) in ARCS} x[i,depot_end,k] =
    sum {(depot_start,j) in ARCS} x[depot_start,j,k];

# C5: Each station copy is visited at most once (across all vehicles)
#     Multiple copies of the same physical station allow repeated visits
subject to StationOnce {s in STATIONS}:
    sum {(i,s) in ARCS, k in VEHICLES} x[i,s,k] <= 1;

# ---------- Load (Cargo) ----------

# C6: Load on arc cannot exceed vehicle capacity (and is 0 if arc not used)
subject to LoadCap {(i,j) in ARCS, k in VEHICLES}:
    f[i,j,k] <= Q * x[i,j,k];

# C7: Load balance at customers — delivery model
#     Incoming load - outgoing load = demand delivered (if visited)
subject to LoadCust {c in CUSTOMERS, k in VEHICLES}:
    sum {(i,c) in ARCS} f[i,c,k] - sum {(c,j) in ARCS} f[c,j,k]
    = demand[c] * sum {(c,j) in ARCS} x[c,j,k];

# C8: Load balance at stations — cargo passes through unchanged
subject to LoadStat {s in STATIONS, k in VEHICLES}:
    sum {(i,s) in ARCS} f[i,s,k] = sum {(s,j) in ARCS} f[s,j,k];

# ---------- Battery ----------

# C9: Battery at depot_start — full charge if vehicle is used, 0 otherwise
subject to BatDepot {k in VEHICLES}:
    y[depot_start,k] = B * sum {(depot_start,j) in ARCS} x[depot_start,j,k];

# C10: Battery at stations — full recharge upon visit
subject to BatRecharge {s in STATIONS, k in VEHICLES}:
    y[s,k] = B * sum {(i,s) in ARCS} x[i,s,k];

# C11: Battery propagation to customers and depot_end
#      When x[i,j,k]=1: y[j,k] <= y[i,k] - energy(i,j,k)
#      When x[i,j,k]=0: constraint is trivially satisfied (Big-M = B)
subject to BatProp {(i,j) in ARCS, k in VEHICLES : j in CUSTOMERS union {depot_end}}:
    y[j,k] <= y[i,k]
              - c_fixed[i,j] * x[i,j,k]
              - c_load[i,j]  * f[i,j,k]
              + B * (1 - x[i,j,k]);

# C12: Battery arrival check at stations
#      Vehicle must have enough battery to REACH the station
#      When x[i,s,k]=1: y[i,k] >= energy(i,s,k)
subject to BatArrive {(i,s) in ARCS, k in VEHICLES : s in STATIONS}:
    y[i,k] >= c_fixed[i,s] * x[i,s,k]
              + c_load[i,s] * f[i,s,k]
              - B * (1 - x[i,s,k]);

# ---------- Subtour Elimination ----------

# C13: Miller-Tucker-Zemlin constraints
#      When x[i,j,k]=1: u[j,k] >= u[i,k] + 1 (position increases along route)
subject to NoSubtour {(i,j) in ARCS, k in VEHICLES :
    i in INTERMEDIATE and j in INTERMEDIATE}:
    u[i,k] - u[j,k] + N_inter * x[i,j,k] <= N_inter - 1;

# ---------- Symmetry Breaking ----------

# C14: Vehicle ordering — vehicle k is used before vehicle k+1
#      Since vehicles are identical, this eliminates symmetric solutions
subject to SymBreak {k in VEHICLES : ord(k) < card(VEHICLES)}:
    sum {(depot_start,j) in ARCS} x[depot_start,j,k] >=
    sum {(depot_start,j) in ARCS} x[depot_start,j,next(k)];
