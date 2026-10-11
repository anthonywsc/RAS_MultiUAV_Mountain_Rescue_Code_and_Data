# Terrain-Informed Multi-UAV Mountain Rescue: Code and Data

This repository contains the MATLAB implementation and experimental data associated with the manuscript:

> Terrain-Informed Two-Stage Task Assignment and Dynamic Reallocation for Multi-UAV Mountain Rescue Missions

## Contents

- Core MATLAB implementation of terrain-aware path planning, pre-mission task assignment, communication-constrained dynamic reallocation, and lazy exact evaluation.
- Final data from 450 paired six-UAV Monte Carlo and sensitivity samples.
- Final data from the 30-scenario pre-mission baseline comparison.
- Final data from the 50-scenario comparison with the CBBA-Euclidean adaptation.
- Fleet-size preview data for three to six UAVs.
- Recorded three-UAV hardware-in-the-loop telemetry and the corresponding validation figures.
- Publication figures derived from the included results.

## Software requirements

- MATLAB R2022b or later is recommended.
- The statistical experiments use MATLAB code and the packaged DEM/data files in this repository.
- Re-executing the hardware-in-the-loop experiment additionally requires PX4/RflySim and three compatible flight-control units. The recorded HIL data can be inspected without that hardware.

No absolute user-specific path is required for the MATLAB statistical experiments. Run MATLAB from the repository root so that `data/` and `output/` resolve correctly.

## Repository structure

```text
.
|-- *.m                              Core and experiment MATLAB files
|-- data/
|   |-- map10.txt                    DEM used by the statistical experiments
|   `-- StaticData_original.mat      Packaged task data
|-- output/
|   |-- InitialData_refactored.mat   Six-UAV initial assignment
|   |-- StaticCostLibrary.mat        Directed terrain-cost library
|   |-- experiment_results/          Final CSV and MAT results
|   `-- run_results/                 Deterministic pruning comparison
|-- figures/                         Publication figures
`-- hil_validation/
    |-- tracking.csv                 Reported 442.6-s, 29,193-sample HIL trial
    |-- plotted_trial_tracking.csv   Completed trial used for attitude/XYZ plots
    |-- assets/                      Terrain and planned mission data
    |-- code/                        HIL execution and post-processing code
    `-- figures/                     HIL trajectory, position, and attitude plots
```

## Main reproduction commands

Run the following from the repository root in MATLAB.

### Pre-mission baseline comparison

```matlab
S = runAllocationBaselineComparison( ...
    'NumScenarios', 30, ...
    'RandomSeed', 20261010, ...
    'Verbose', true);
```

### Monte Carlo, ablation, and communication/resource sensitivity

```matlab
S = runReallocationSensitivity( ...
    'NumScenarios', 50, ...
    'DynamicTaskCounts', [1 2 3], ...
    'CommDistancesM', [900 1350 1800], ...
    'RandomSeed', 20260923, ...
    'Verbose', false);
```

This produces 450 paired scenario-configuration samples. Each realization is evaluated with exhaustive exact candidate evaluation and with the proposed lazy branch-and-bound policy using the same initial state and dynamic tasks.

### Published CBBA comparison

```matlab
S = runPublishedCBBAComparison( ...
    'NumScenarios', 50, ...
    'NumDynamicTasks', 2, ...
    'CommDistanceM', 1350, ...
    'RandomSeed', 20261011, ...
    'Verbose', false);
```

On machines where many repeated terrain-planning calls place excessive pressure on MATLAB memory, run isolated batches with `ScenarioOffset` and `OutputTag`, then call `combinePublishedCBBAComparison`.

### Deterministic exhaustive-versus-lazy ablation

```matlab
C = compareLazyExactEvaluation();
```

### Fleet-size preview

```matlab
runFleetScalabilityPreview();
```

## Final result files

- `output/experiment_results/latest_reallocation_sensitivity.csv`: 450 paired online samples.
- `output/experiment_results/allocation_baseline_comparison_30.csv`: 30 paired pre-mission scenarios and three methods.
- `output/experiment_results/allocation_baseline_summary.csv`: aggregate pre-mission statistics.
- `output/experiment_results/published_cbba_comparison_50.csv`: 50 paired published-method comparison scenarios.
- `output/experiment_results/fleet_scalability_*uav.csv`: fleet-size preview data.
- `hil_validation/tracking.csv`: the 442.6-s HIL trial containing 29,193 recorded telemetry samples reported in the manuscript.
- `hil_validation/plotted_trial_tracking.csv`: the subsequent completed HIL trial used to produce the archived attitude and XYZ plots.

CSV files are provided for direct inspection. MAT files retain MATLAB tables, options, and supporting structures used in the analysis.

## Data notes

The statistical experiments use the original DEM-based terrain packaged in `data/map10.txt`. The HIL validation uses the separate Zhashui terrain asset under `hil_validation/assets/`; it is not used in the Monte Carlo, baseline, ablation, or CBBA comparisons.

The `CBBA-Euclidean` method is a single-task adaptation of the consensus-based bundle-allocation principle. At each event, reachable UAVs submit Euclidean marginal insertion bids and the winning insertion is validated with the common terrain and resource model.

## Citation

Please cite the associated manuscript when using this code or data. Citation metadata is also provided in `CITATION.cff`.
