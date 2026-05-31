# 🐒 MonkeyBot: Adhesion-Free Wall Climbing Framework

| Status | Language | License |
| :--- | :--- | :--- |
| **Active Development** | Python (100%) | *Unspecified (Contact Authors)* |

## Project Overview

**MonkeyBot** is a simulation and planning framework for a **multi-legged wall-climbing robot** that operates without relying on any adhesive mechanisms (magnets, suction, or microspines).

The system models climbing as a **numeric planning problem** that integrates discrete grip selection with continuous body motions, including dynamic jumps. It combines a 2D Pymunk physics simulation with UPF-based planning.

---

## Repository Structure

| Path | Description |
| :--- | :--- |
| **`monkey_bot/`** | Core package: planner, simulator, procedure-based controller, trajectory finder |
| **`instances/`** | Climbing scenario JSON files |
| **`cache/plans/`** | Cached UPF motion plans (one file per instance) |
| **`cache/transition_links/`** | Cached jump launch instructions (precomputed ballistic trajectories) |
| **`experiments/`** | Batch runners and analysis scripts |
| **`experiments/results/`** | Experiment outputs (CSVs, plots); gitignored |
| **`tests/`** | Unit and integration tests |
| **`docs/`** | Design notes including `controller_improvement_plan.md` |

**Cache vs experiment data:** `cache/` holds planner artifacts reused across runs (plans and transition links). `experiments/results/` holds metrics and plots from tuning batches — regenerate as needed.

---

## Installation

```bash
git clone https://github.com/Ev713/MonkeyBot.git
cd MonkeyBot
python3 -m venv venv
source venv/bin/activate
pip install pygame pymunk numpy pandas unified-planning
```

---

## Running a Simulation

From the repository root:

```bash
python tests/test_sim_runner.py
```

Or programmatically:

```python
from monkey_bot.monkey_bot_problem_instance import load_instance
from monkey_bot.sim_runner import SimRunner
from monkey_bot.config import SimConfig, RobotConfig

instance = load_instance("GPT1", "instances")
sim_runner = SimRunner(instance, sim_config=SimConfig(...), robot_config=RobotConfig(...))
sim_runner.execute_simulation()  # reads cache/plans and cache/transition_links
```

Plans and transition links are written to `cache/` on first run for each instance, then reused.

---

## Experiments

```bash
python experiments/limb_angle_sweep.py --instance TestAngle
python experiments/actual_sim_batch.py --pairs 10 --out-dir experiments/results/actual_sim_batch
python experiments/experiment_metrics.py
python experiments/pruning_analyzer.py
```

---

## Further Reading

- **`docs/controller_improvement_plan.md`** — Proposed fixes for jerky limb motion and low jump success rate
- **`docs/control_system_rework_plan.md`** — Archived MR-style control stack design (not in use)
