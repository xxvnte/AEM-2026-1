# ===================================================
# run_all.ps1 — Batch solve EVRP instances with AMPL/CPLEX
# ===================================================
# Converts .txt instances to .dat, solves each with CPLEX,
# and collects results in output/CPLEX_solutions/
# ===================================================

$ErrorActionPreference = "Continue"

# --- Paths ---
$AMPL_EXE    = "C:\Users\FORM0\AMPL\ampl.exe"
$SCRIPT_DIR  = Split-Path -Parent $MyInvocation.MyCommand.Path
$PROJECT_DIR = Split-Path -Parent $SCRIPT_DIR
$MODEL_FILE  = Join-Path $SCRIPT_DIR "evrp.mod"
$DAT_DIR     = Join-Path $SCRIPT_DIR "dat"
$OUTPUT_DIR  = Join-Path $PROJECT_DIR "output\CPLEX_solutions"
$INSTANCES_SMALL = Join-Path $PROJECT_DIR "instances\small"
$INSTANCES_LARGE = Join-Path $PROJECT_DIR "instances\large"

# Per-instance CPLEX overrides
$INSTANCE_TIME_LIMITS = @{
    "C23R2" = 900
}
$INSTANCE_MIP_GAPS = @{
    "C23R2" = 0.05
}

# --- Configuration ---
# Time limits and gap tolerances by instance size
$CONFIG = @{
    "small" = @{ timelimit = 300;  mipgap = 0.01; dir = $INSTANCES_SMALL }
    # Uncomment to also run large instances:
    # "large_small" = @{ timelimit = 600;  mipgap = 0.05; dir = $INSTANCES_LARGE; filter = "C25*","C50*" }
    # "large_big"   = @{ timelimit = 1800; mipgap = 0.05; dir = $INSTANCES_LARGE; filter = "C75*","C100*","C150*" }
}

# --- Create output directories ---
New-Item -ItemType Directory -Force -Path $OUTPUT_DIR | Out-Null
New-Item -ItemType Directory -Force -Path $DAT_DIR    | Out-Null

# --- Step 1: Convert instances to .dat ---
Write-Host "=" * 60
Write-Host "STEP 1: Converting instances to AMPL .dat format"
Write-Host "=" * 60

foreach ($setName in $CONFIG.Keys) {
    $cfg = $CONFIG[$setName]
    $instanceDir = $cfg.dir

    if (-not (Test-Path $instanceDir)) {
        Write-Host "  [SKIP] Directory not found: $instanceDir"
        continue
    }

    Write-Host "`nConverting instances from: $instanceDir"
    python (Join-Path $SCRIPT_DIR "export_to_dat.py") $instanceDir

    if ($LASTEXITCODE -ne 0) {
        Write-Host "  [ERROR] Conversion failed for $setName"
        exit 1
    }
}

# --- Step 2: Solve each instance ---
Write-Host "`n" + ("=" * 60)
Write-Host "STEP 2: Solving instances with AMPL/CPLEX"
Write-Host "=" * 60

# CSV header
$csvPath = Join-Path $OUTPUT_DIR "cplex_results.csv"
"instance,status,objective_kwh,solve_result_num,vehicles_used,time_limit,mip_gap" | Out-File -FilePath $csvPath -Encoding utf8

foreach ($setName in $CONFIG.Keys) {
    $cfg = $CONFIG[$setName]
    $instanceDir = $cfg.dir
    $timelimit = $cfg.timelimit
    $mipgap = $cfg.mipgap

    if (-not (Test-Path $instanceDir)) { continue }

    # Get list of .dat files for this set
    $txtFiles = Get-ChildItem (Join-Path $instanceDir "*.txt") | Sort-Object Name

    foreach ($txtFile in $txtFiles) {
        $instanceName = $txtFile.BaseName
        $datFile = Join-Path $DAT_DIR "$instanceName.dat"

        $timelimit = $cfg.timelimit
        if ($INSTANCE_TIME_LIMITS.ContainsKey($instanceName)) {
            $timelimit = $INSTANCE_TIME_LIMITS[$instanceName]
        }
        $mipgap = $cfg.mipgap
        if ($INSTANCE_MIP_GAPS.ContainsKey($instanceName)) {
            $mipgap = $INSTANCE_MIP_GAPS[$instanceName]
        }

        if (-not (Test-Path $datFile)) {
            Write-Host "  [SKIP] No .dat file for $instanceName"
            continue
        }

        Write-Host "`n--- Solving: $instanceName (timelimit=${timelimit}s, gap=${mipgap}) ---"

        # Build AMPL commands
        $datFileUnix = $datFile -replace '\\', '/'
        $modelFileUnix = $MODEL_FILE -replace '\\', '/'

        $amplCommands = @"
reset;
model '$modelFileUnix';
data '$datFileUnix';

option solver cplex;
option cplex_options 'timelimit=$timelimit mipgap=$mipgap threads=0 mipdisplay=1';

solve;

# Declare scripting params once
param nv default 0;
param cur integer default 0;
param found integer default 0;

printf "RESULT_START\n";
printf "instance=%s\n", "$instanceName";
printf "status=%s\n", solve_result;
printf "status_num=%d\n", solve_result_num;
printf "objective=%.8f\n", TotalEnergy;

let nv := sum {k in VEHICLES}
    (if sum {(depot_start,j) in ARCS} x[depot_start,j,k] > 0.5 then 1 else 0);
printf "vehicles_used=%d\n", nv;
printf "RESULT_END\n";

printf "\n--- ROUTES ---\n";
for {k in VEHICLES: sum{(depot_start,j) in ARCS} x[depot_start,j,k] > 0.5} {
    printf "Vehicle %d:", k;
    let cur := depot_start;
    printf " %d", orig_id[cur];
    repeat {
        let found := 0;
        for {(i,j) in ARCS: i == cur and x[i,j,k] > 0.5} {
            printf " -> %d", orig_id[j];
            let cur := j;
            let found := 1;
        }
        if found == 0 then break;
    } until (cur == depot_end);
    printf "\n";
}
printf "END_ROUTES\n";

printf "\n--- ACTIVE ARCS ---\n";
printf "from,to,vehicle,load_tons\n";
for {(i,j) in ARCS, k in VEHICLES: x[i,j,k] > 0.5} {
    printf "%d,%d,%d,%.6f\n", orig_id[i], orig_id[j], k, f[i,j,k];
}
printf "END_ARCS\n";
"@

        # Save solution output
        $solutionFile = Join-Path $OUTPUT_DIR "${instanceName}_solution.txt"

        try {
            $output = $amplCommands | & $AMPL_EXE 2>&1
            $output | Out-File -FilePath $solutionFile -Encoding utf8
            Write-Host "  Solution saved to: $solutionFile"

            # Parse results from output
            $outputStr = $output -join "`n"
            $status = "unknown"
            $objective = "N/A"
            $statusNum = "-1"
            $vehiclesUsed = "0"

            if ($outputStr -match "status=(\S+)") { $status = $Matches[1] }
            if ($outputStr -match "objective=([\d.]+)") { $objective = $Matches[1] }
            if ($outputStr -match "status_num=(\d+)") { $statusNum = $Matches[1] }
            if ($outputStr -match "vehicles_used=(\d+)") { $vehiclesUsed = $Matches[1] }

            # Append to CSV
            "$instanceName,$status,$objective,$statusNum,$vehiclesUsed,$timelimit,$mipgap" |
                Out-File -FilePath $csvPath -Append -Encoding utf8

            Write-Host "  Status: $status | Objective: $objective kWh | Vehicles: $vehiclesUsed"
        }
        catch {
            Write-Host "  [ERROR] Failed to solve $instanceName : $_"
            "$instanceName,error,N/A,-1,0,$timelimit,$mipgap" |
                Out-File -FilePath $csvPath -Append -Encoding utf8
        }
    }
}

# --- Summary ---
Write-Host "`n" + ("=" * 60)
Write-Host "BATCH EXECUTION COMPLETE"
Write-Host "=" * 60
Write-Host "Results CSV:  $csvPath"
Write-Host "Solutions in: $OUTPUT_DIR"
Write-Host ""

# Read and display CSV summary
if (Test-Path $csvPath) {
    Write-Host "--- Results Summary ---"
    Get-Content $csvPath | Format-Table
}
