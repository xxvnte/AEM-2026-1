set NODES;
set CUSTOMERS within NODES;
set STATIONS within NODES;
set RECHARGE within NODES;
set ARCS within NODES cross NODES;

param depot_start;
param depot_end;
param cap_kg;
param bat_cap;
param big_m_load;
param big_m_batt;
param max_vehicles;

param demand {NODES} default 0;
param energy {ARCS};

var x {ARCS} binary;
var load {NODES} >= 0, <= cap_kg;
var batt {NODES} >= 0, <= bat_cap;

minimize total_energy:
    sum {(i, j) in ARCS} energy[i, j] * x[i, j];

s.t. customer_inflow {j in CUSTOMERS}:
    sum {(i, j) in ARCS} x[i, j] = 1;

s.t. depot_end_inflow:
    sum {(i, depot_end) in ARCS} x[i, depot_end] <= max_vehicles;

s.t. station_inflow {j in STATIONS}:
    sum {(i, j) in ARCS} x[i, j] <= 1;

s.t. customer_outflow {i in CUSTOMERS}:
    sum {(i, j) in ARCS} x[i, j] = 1;

s.t. depot_start_outflow:
    sum {(depot_start, j) in ARCS} x[depot_start, j] <= max_vehicles;

s.t. station_balance {j in STATIONS}:
    sum {(i, j) in ARCS} x[i, j] = sum {(j, k) in ARCS} x[j, k];

s.t. load_start:
    load[depot_start] = 0;

s.t. batt_start:
    batt[depot_start] = bat_cap;

s.t. load_propagate {(i, j) in ARCS}:
    load[j] >= load[i] + demand[j] - big_m_load * (1 - x[i, j]);

s.t. load_propagate_ub {(i, j) in ARCS}:
    load[j] <= load[i] + demand[j] + big_m_load * (1 - x[i, j]);

s.t. batt_propagate {(i, j) in ARCS: j not in RECHARGE}:
    batt[j] >= batt[i] - energy[i, j] - big_m_batt * (1 - x[i, j]);

s.t. batt_recharge_lb {(i, j) in ARCS: j in RECHARGE}:
    batt[j] >= bat_cap - big_m_batt * (1 - x[i, j]);

s.t. batt_recharge_ub {(i, j) in ARCS: j in RECHARGE}:
    batt[j] <= bat_cap + big_m_batt * (1 - x[i, j]);
