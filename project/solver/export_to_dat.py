#!/usr/bin/env python3
"""
export_to_dat.py — Convert EVRP .txt instances to AMPL .dat files.

Usage:
    python export_to_dat.py <instance.txt>          # Single file
    python export_to_dat.py <directory>              # All .txt files in directory
    python export_to_dat.py <dir1> <dir2> ...        # Multiple directories

Output goes to ampl/dat/ relative to the project root.

Station copies: each physical recharging station is duplicated N_COPIES times
(default 2) to allow multiple visits in the MILP model.
"""

import os
import sys
import math
import glob

# Number of copies per physical recharging station
N_COPIES = 2


def parse_instance(filepath):
    """Parse an EVRP .txt instance file."""
    params = {}
    nodes = []
    dist_matrix = []
    speed_matrix = []

    with open(filepath, "r", encoding="utf-8") as f:
        lines = f.readlines()

    section = "header"
    for line in lines:
        line = line.strip()
        if not line or line.startswith("#"):
            continue

        if line == "NODE_COORD_SECTION":
            section = "nodes"
            continue
        elif line == "DISTANCE_MATRIX_MILES":
            section = "distance"
            continue
        elif line == "SPEED_MATRIX_KMH":
            section = "speed"
            continue
        elif line == "EOF":
            break

        if section == "header":
            if ":" in line:
                key, value = line.split(":", 1)
                key = key.strip()
                value = value.strip()
                try:
                    if "." in value:
                        params[key] = float(value)
                    else:
                        params[key] = int(value)
                except ValueError:
                    params[key] = value

        elif section == "nodes":
            parts = line.split()
            if len(parts) >= 5:
                nodes.append(
                    {
                        "id": int(parts[0]),
                        "type": parts[1],  # D, C, or S
                        "x": float(parts[2]),
                        "y": float(parts[3]),
                        "demand": float(parts[4]),
                    }
                )

        elif section == "distance":
            row = [float(v) for v in line.split()]
            dist_matrix.append(row)

        elif section == "speed":
            row = [float(v) for v in line.split()]
            speed_matrix.append(row)

    return params, nodes, dist_matrix, speed_matrix


def build_ampl_data(params, nodes, dist_matrix, speed_matrix):
    """
    Build expanded AMPL data with:
    - depot_start (node 0) and depot_end (node 1) sharing depot coordinates
      (matches C++ internal: DEPOT_START=0, DEPOT_END=1)
    - Customers from node 2 onwards
    - N_COPIES copies of each physical recharging station after customers
    - Precomputed c_fixed and c_load energy parameters
    """
    n_customers = int(params["CUSTOMERS"])
    n_stations = int(params["STATIONS"])

    # Original node indices
    depot_orig = 0
    cust_orig = list(range(1, n_customers + 1))
    station_orig = list(range(n_customers + 1, n_customers + n_stations + 1))

    # Build mapping: ampl_index -> original_index
    # Matches C++ internal representation:
    #   0 = DEPOT_START, 1 = DEPOT_END, 2..C+1 = Customers, C+2.. = Station copies
    mapping = []
    ampl_customers = []
    ampl_stations = []

    # Node 0: depot_start
    mapping.append(depot_orig)

    # Node 1: depot_end (same location as depot)
    mapping.append(depot_orig)
    depot_start_id = 0
    depot_end_id = 1

    # Nodes 2..C+1: customers (same order as original)
    for c in cust_orig:
        ampl_customers.append(len(mapping))
        mapping.append(c)

    # Nodes C+2..C+1+S*N_COPIES: station copies
    for s in station_orig:
        for _ in range(N_COPIES):
            ampl_stations.append(len(mapping))
            mapping.append(s)

    n_ampl = len(mapping)

    # --- Expand distance matrix ---
    ampl_dist = [[0.0] * n_ampl for _ in range(n_ampl)]
    for i in range(n_ampl):
        for j in range(n_ampl):
            oi, oj = mapping[i], mapping[j]
            if oi == oj:
                ampl_dist[i][j] = 0.0
            else:
                ampl_dist[i][j] = dist_matrix[oi][oj]

    # --- Expand speed matrix ---
    ampl_speed = [[0.0] * n_ampl for _ in range(n_ampl)]
    for i in range(n_ampl):
        for j in range(n_ampl):
            oi, oj = mapping[i], mapping[j]
            if oi == oj:
                ampl_speed[i][j] = 0.0
            else:
                ampl_speed[i][j] = speed_matrix[oi][oj]

    # --- Physical parameters ---
    alpha = params["ALPHA"]
    beta = params["BETA"]
    eff_d = params["EFF_BATTERY_DISCHARGE"]
    eff_m = params["EFF_MOTOR"]
    curb_w = params["CURB_WEIGHT_KG"]
    miles_to_m = params["MILES_TO_METERS"]
    B = params["BATTERY_CAPACITY_KWH"]
    Q = params["VEHICLE_CAPACITY_TONS"]
    max_veh = int(params["MAX_VEHICLES"])

    # --- Compute c_fixed and c_load for each arc ---
    # c_fixed[i,j] = eff_d * eff_m * (alpha * curb_w * d_m + beta * s_ms^2 * d_m) / 3600000
    # c_load[i,j]  = eff_d * eff_m * alpha * 1000 * d_m / 3600000
    c_fixed = [[0.0] * n_ampl for _ in range(n_ampl)]
    c_load = [[0.0] * n_ampl for _ in range(n_ampl)]

    for i in range(n_ampl):
        for j in range(n_ampl):
            if i == j:
                continue
            d_miles = ampl_dist[i][j]
            s_kmh = ampl_speed[i][j]
            if d_miles <= 0.0:
                continue

            d_m = d_miles * miles_to_m
            s_ms = s_kmh * 1000.0 / 3600.0

            c_fixed[i][j] = (
                eff_d
                * eff_m
                * (alpha * curb_w * d_m + beta * s_ms * s_ms * d_m)
                / 3_600_000.0
            )
            c_load[i][j] = eff_d * eff_m * alpha * 1000.0 * d_m / 3_600_000.0

    # --- Demand vector ---
    ampl_demand = [0.0] * n_ampl
    for aid in ampl_customers:
        orig_id = mapping[aid]
        ampl_demand[aid] = nodes[orig_id]["demand"]

    # --- orig_id: map each AMPL node to its C++ internal ID ---
    # depot_start=0, depot_end=1, customers keep same ID
    # station copies map to the C++ ID of the FIRST copy of that physical station
    # C++ layout: 0=depot_start, 1=depot_end, 2..C+1=customers, C+2..C+S+1=stations
    cpp_station_ids = {}  # physical_orig_idx -> C++ internal ID
    for idx, s in enumerate(station_orig):
        cpp_station_ids[s] = n_customers + 2 + idx  # C+2+idx

    ampl_orig_id = [0] * n_ampl
    ampl_orig_id[0] = 0  # depot_start
    ampl_orig_id[1] = 1  # depot_end
    for aid in ampl_customers:
        ampl_orig_id[aid] = aid  # customers keep same ID (2..C+1)
    for aid in ampl_stations:
        phys_orig = mapping[aid]  # original index in instance file
        ampl_orig_id[aid] = cpp_station_ids[phys_orig]

    return {
        "n_ampl": n_ampl,
        "depot_start": depot_start_id,
        "depot_end": depot_end_id,
        "customers": ampl_customers,
        "stations": ampl_stations,
        "vehicles": list(range(1, max_veh + 1)),
        "Q": Q,
        "B": B,
        "demand": ampl_demand,
        "c_fixed": c_fixed,
        "c_load": c_load,
        "orig_id": ampl_orig_id,
        "mapping": mapping,
    }


def write_dat(ampl_data, output_path, instance_name=""):
    """Write a .dat file in AMPL format."""
    n = ampl_data["n_ampl"]
    all_nodes = list(range(n))

    os.makedirs(os.path.dirname(output_path), exist_ok=True)

    with open(output_path, "w", encoding="utf-8") as f:
        f.write(f"# AMPL data file for EVRP instance: {instance_name}\n")
        f.write(f"# Auto-generated by export_to_dat.py\n")
        f.write(
            f"# Nodes: {n} (depot_start + {len(ampl_data['customers'])} customers"
            f" + {len(ampl_data['stations'])} station copies + depot_end)\n\n"
        )

        # --- Sets ---
        f.write(f"set NODES := {' '.join(str(i) for i in all_nodes)};\n\n")
        f.write(
            f"set CUSTOMERS := {' '.join(str(i) for i in ampl_data['customers'])};\n\n"
        )
        f.write(
            f"set STATIONS := {' '.join(str(i) for i in ampl_data['stations'])};\n\n"
        )
        f.write(
            f"set VEHICLES := {' '.join(str(i) for i in ampl_data['vehicles'])};\n\n"
        )

        # --- Scalar parameters ---
        f.write(f"param depot_start := {ampl_data['depot_start']};\n")
        f.write(f"param depot_end   := {ampl_data['depot_end']};\n")
        f.write(f"param Q := {ampl_data['Q']};\n")
        f.write(f"param B := {ampl_data['B']};\n\n")

        # --- Demand ---
        f.write("param demand :=\n")
        for i in all_nodes:
            f.write(f"  {i}  {ampl_data['demand'][i]:.6f}\n")
        f.write(";\n\n")

        # --- orig_id (maps AMPL node -> C++ internal ID for display) ---
        f.write("param orig_id :=\n")
        for i in all_nodes:
            f.write(f"  {i}  {ampl_data['orig_id'][i]}\n")
        f.write(";\n\n")

        # --- c_fixed (2D matrix) ---
        header = "  ".join(f"{j:>12}" for j in all_nodes)
        f.write(f"param c_fixed :\n  {header} :=\n")
        for i in all_nodes:
            row = "  ".join(f"{ampl_data['c_fixed'][i][j]:12.8f}" for j in all_nodes)
            f.write(f"  {i:>3}  {row}\n")
        f.write(";\n\n")

        # --- c_load (2D matrix) ---
        f.write(f"param c_load :\n  {header} :=\n")
        for i in all_nodes:
            row = "  ".join(f"{ampl_data['c_load'][i][j]:12.8f}" for j in all_nodes)
            f.write(f"  {i:>3}  {row}\n")
        f.write(";\n\n")

    print(f"  [OK] {output_path}")


def process_file(txt_path, dat_dir):
    """Convert a single .txt instance to .dat."""
    basename = os.path.splitext(os.path.basename(txt_path))[0]
    dat_path = os.path.join(dat_dir, f"{basename}.dat")

    params, nodes, dist_matrix, speed_matrix = parse_instance(txt_path)
    ampl_data = build_ampl_data(params, nodes, dist_matrix, speed_matrix)
    write_dat(ampl_data, dat_path, instance_name=basename)

    return dat_path


def main():
    if len(sys.argv) < 2:
        print("Usage:")
        print("  python export_to_dat.py <instance.txt>")
        print("  python export_to_dat.py <directory>")
        print("  python export_to_dat.py <dir1> <dir2> ...")
        sys.exit(1)

    # Determine output directory: ampl/dat/ relative to this script
    script_dir = os.path.dirname(os.path.abspath(__file__))
    dat_dir = os.path.join(script_dir, "dat")
    os.makedirs(dat_dir, exist_ok=True)

    txt_files = []
    for arg in sys.argv[1:]:
        if os.path.isfile(arg) and arg.endswith(".txt"):
            txt_files.append(arg)
        elif os.path.isdir(arg):
            txt_files.extend(sorted(glob.glob(os.path.join(arg, "*.txt"))))
        else:
            print(f"  [SKIP] {arg} (not a .txt file or directory)")

    if not txt_files:
        print("No .txt instance files found.")
        sys.exit(1)

    print(f"Converting {len(txt_files)} instance(s) to AMPL .dat format")
    print(f"Output directory: {dat_dir}\n")

    for txt_path in txt_files:
        try:
            process_file(txt_path, dat_dir)
        except Exception as e:
            print(f"  [ERROR] {txt_path}: {e}")

    print(f"\nDone. {len(txt_files)} file(s) processed.")


if __name__ == "__main__":
    main()
