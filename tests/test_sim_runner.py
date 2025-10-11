import time
import optuna

from monkey_bot.monkey_bot_problem_instance import load_instance
from monkey_bot.sim_runner import SimRunner
from monkey_bot.simulation_config import SimConfig

INSTANCES_FOLDER = "instances"
INSTANCE_NAME = "5StepsLadder"
C = 9999

def test_efficiency(sim_config, timeout=30.0, test=False):
    """Run one simulation and return a numeric score for Optuna to optimize."""
    instance = load_instance(INSTANCE_NAME, INSTANCES_FOLDER)
    sim_runner = SimRunner(instance, sim_config=sim_config)

    sim_runner.start_simulator()
    sim_runner.start_controller(test=test)
    sim_runner.controller.load_plan("plan_example.txt")

    start = time.perf_counter()
    while sim_runner.simulator.run:
        if sim_runner.controller.longest_action_time > timeout or not sim_runner.simulator.run:
            print("Too slow")
            return C - sim_runner.controller.actions_finished  # too slow → penalize
        try:
            state = sim_runner.simulator.get_state()
            sim_runner.controller.update(state)
            sig = sim_runner.controller.get_sig()
            sim_runner.simulator.apply_signal(sig)
            sim_runner.simulator.step()
        except Exception as e:
            print(f"Failed with: {e}")
            return C - sim_runner.controller.actions_finished
        if sim_runner.controller.finished_plan:
            if hasattr(sim_runner.simulator, "writer"):
                sim_runner.simulator.writer.close()
            duration = time.perf_counter() - start
            print(f"Returned {duration}")
            return duration  # smaller = faster = better
    return C - sim_runner.controller.actions_finished


def objective(trial):
    """Define the Optuna objective function."""

    # fixed parameters
    static = {
        "screen_height": 1000,
        "screen_width": 800,
        "epsilon": 5,
        "fps": 60,
    }

    # hyperparameter search space
    sim_config = SimConfig(
        **static,
        extension_speed=trial.suggest_float("extension_speed", 50, 200),
        twist_speed=trial.suggest_float("twist_speed", 0.5, 3.0),
        move_center_speed=trial.suggest_float("move_center_speed", 50, 500),
        body_mass=trial.suggest_float("body_mass", 0.1, 10.0),
        foot_mass=trial.suggest_float("foot_mass", 0.1, 10.0),
        leg_mass=trial.suggest_float("leg_mass", 0.1, 10.0),
        leg_spring_stiffness=trial.suggest_int("leg_spring_stiffness", 1000, 20000),
        leg_spring_damping=trial.suggest_int("leg_spring_damping", 10, 500),
    )

    score = test_efficiency(sim_config, timeout=10.0)
    return score  # minimize duration


def find_best_sim_config(n_trials=50):
    """Run Optuna optimization."""
    study = optuna.create_study(direction="minimize")
    study.optimize(objective, n_trials=n_trials)

    print("\n=== Best Trial ===")
    best = study.best_trial
    print(f"Score: {best.value:.4f} sec")
    print("Parameters:")
    for k, v in best.params.items():
        print(f"  {k}: {v}")

    return best


if __name__ == "__main__":
    best_trial = find_best_sim_config(1000)  # try 25 trials to start
