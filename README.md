# Optimization-Based Motion Planning for Autonomous Cable-Towing Vehicles in Confined Warehouses

MATLAB/AMPL code and numerical evidence for optimization-based planning of an autonomous tractor with passive cable-towed carts. The cable has fixed material length and can become slack. All compared strategies share first-order explicit Euler kinematics, polygonal body geometry, OBCA constraints, initial controls, and physical acceptance rules.

## Results and scope

The primary comparison contains nine 16 m × 16 m warehouse tasks. The proposed feasible-seed refinement method returns 9/9 new validated outputs with 4.36% mean per-case travel reduction; direct MAX returns 8/9 with 0.97%. Median backend times are 31.03 s and 30.97 s. MAX with recovery also returns 9/9, with 1.07% mean travel reduction. Proposed has lower objectives than MAX in eight cases; MAX has the lower objective on W09.

The separate 18-case strip study pairs a geometric rigid-drawbar obstruction certificate with discrete cable trajectories. Extensions include six initialization/reserve combinations, paired recovery ablations, four runs on each of three tasks, computation allowances, five convex cart shapes, and a constructive three-cart formation.

The quasistatic model assumes rapid stopping of an unpulled cart and hitch motion along the pulling cable direction. Acceptance includes independent Euler replay and per-node geometric cable routes. The intended application is trajectory preparation before low-speed warehouse transfers. Actual finer-replay terminal errors are in [results/terminal_errors.csv](results/terminal_errors.csv): maximum Euclidean position error 0.155 m and maximum cart heading error 1.26 degrees.

## Inspect and reproduce

1. Open MATLAB R2024a at the repository root and run `RunStudy('results')`.
2. Run `python tools/summarize.py` to regenerate numerical paper tables and summary CSV/JSON from archived records. Dependencies are in `requirements.txt`.
3. Run `RunStudy('verify')` to recompute recursive Euler motion, primal polygon distances, cable routes, and acceptance of all 214 optimization records. This needs no optimizer license.
4. Run `RunStudy('figures')` for experimental vector PDFs, PNGs, and editable MATLAB FIG files; run `python tools/draw_schematics.py` for the two geometric schematics. Text uses Times New Roman.
5. Run `python tools/check_release.py` to check distributed-file integrity and numerical summaries.

`RunStudy('main')`, `RunStudy('extensions')`, and `RunStudy('all')` skip existing records. For independent optimization, create a fresh workspace with `RunStudy('workspace','reruns/my_run')`, change into it, configure the runtime, and run `RunStudy('all')`. Keep archived published records for comparison. Timing and numerical termination can vary between machines and repetitions.

### Solver runtime

Reoptimization uses AMPL, Ipopt 3.12.13, BONMIN 1.8.9, and HSL MA57 on Windows. Use one MATLAB computation thread (`maxNumCompThreads(1)`) and one optimization job at a time. Run `RunStudy('setup','path/to/licensed/runtime')` with locally licensed `ampl.exe`, `ampl.lic`, `ipopt.exe`, `bonmin.exe`, `libipoptfort.dll`, and `libhsl.dll`. Proprietary binaries and licenses are not distributed. The PowerShell process wrapper records solver termination overhead. The nominal allowance and measured total backend time are reported separately.

## Evidence map

| Paper evidence | Records |
|---|---|
| Primary comparison and per-case objectives | `results/warehouse_v6/main`, `confirm`, `unique` |
| Common maps and initial controls | `results/warehouse_v6/seeds`, `layouts` |
| Recovery ablations | `results/warehouse_v6/ablations`, `paired`, and `prefix` fields in `confirm` |
| Six initialization/reserve combinations | `final` (F1), `matched` (F2–F4), `factorial` (F5–F6) under `results/warehouse_v6` |
| Rigid numerical comparison | `results/warehouse_v6/rigid` |
| Three additional runs per fixed task/method | `results/warehouse_v6/repeat1`, `repeat2`, `repeat3` |
| LSE and allowance tests | `results/warehouse_v6/smoothing`, `budgets` |
| Five cart shapes | `results/warehouse_v6/shape_seeds`, `shapes` |
| Three-cart construction | `results/warehouse_v6/train` |
| Eighteen strip cases and cable audit | `results/warehouse_v2/mechanism`, `results/euler/mechanism` |
| Exact solver inputs and logs | `results/warehouse_v2/run_inputs`, `logs` |

Each run has an inspectable JSON record and a MAT record with the selected trajectory, stage candidates, and model parameters. `results/record_index.json` indexes all distributed optimization runs; `results/warehouse_v6/verification.json` stores the independent replay audit. Directory suffixes remain stable identifiers used by the code. `research/warehouse_v6/final_method.json` specifies the single proposed configuration.

## Method and comparisons

The proposed method uses relaxed initialization, feasible control restoration, measured replay reserves, direct-max refinement, and final recovery. See `WarehouseFeasibleStartAblation.m` with `no_cap` and Algorithm 1. MAX is the direct maximum baseline defined in this study. LOGIC, COMP, and MINLP encode the same relation. FIX and PORT transfer published fixed-mode and candidate-mode strategies into the shared model. RGR and RRP isolate reserve guidance in a solve-only workflow. BONMIN's nonconvex subproblems provide local numerical candidates.

`New` counts newly accepted outputs separately from retaining a common seed. `NLP` counts accepted solver candidates before recovery. Costs are recomputed from controls. Failed and seed-retaining outcomes remain in the records. The strip theorem certifies rigid geometric obstruction; rigid warehouse solver failures are numerical outcomes of their stated initialization.

## Manuscript and integrity

The final LaTeX source is in `paper/`; compile `main.tex` with IEEEtran using LaTeX/BibTeX or Tectonic. Figure PDFs and 25 bibliography entries are included. `MANIFEST.json` records SHA-256 hashes. Local account paths were sanitized in release copies while preserving numeric arrays, model parameters, and outcome flags.
