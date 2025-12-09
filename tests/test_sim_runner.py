
import pandas as pd
import os
import traceback

from monkey_bot.monkey_bot_problem_instance import load_instance
from monkey_bot.sim_runner import SimRunner
from monkey_bot.config import SimConfig, RobotConfig
from pruning_analyzer import analyze_pruning_combinations

INSTANCES_FOLDER = "instances"
save_simulation=True
sim_config = SimConfig(
        screen_height= 1000,
        screen_width= 1000,
        fps=120,
        gravity=1,
        cell_size=20
        )

robot_config = RobotConfig(
    epsilon=0.1,
    extension_speed=4,
    rotation_speed=4,
    move_center_speed=4,
    body_mass=5,
    body_radius=0.4,
    foot_mass=0.2,
    leg_mass=0.2,
    simplified_problem=True,
    leg_spring_stiffness=500,
    leg_spring_damping=1.5,
    min_extension=0.2,
    max_takeoff_speed=20,
    max_jump_dist=15,
    prune_short_jumps=True,
    prune_in_clique_jumps=True,
    prune_similar_jumps=True,
)

def test_simple_climbing():
    # hyperparameter search space
    instance = load_instance("GPT1", INSTANCES_FOLDER)

    sim_runner = SimRunner(instance, sim_config=sim_config, robot_config=robot_config)

    sim_runner.execute_simulation(save=save_simulation)

def test_jump_procedure_sequence():
    instance = load_instance("JumpTest", INSTANCES_FOLDER)
    sim_runner = SimRunner(instance, sim_config=sim_config, robot_config=robot_config)
    sim_runner.execute_manual_simulation(save=save_simulation)

def test_angle_adjuster():
    instance = load_instance("Random1", INSTANCES_FOLDER)
    sim_runner = SimRunner(instance, sim_config=sim_config, robot_config=robot_config)
    sim_runner.log_rotation_motors = [0]
    sim_runner.execute_simulation(save=False)


def append_to_csv(df, filename):
    """Appends a DataFrame to a CSV file."""
    # Check if the file exists to decide whether to write the header
    header_needed = not os.path.exists(filename) or os.path.getsize(filename) == 0

    # Use 'a' for append mode
    df.to_csv(filename, mode='a', header=header_needed, index=False)

    print(f"Successfully appended results to **{filename}**.")


def test_pruning():
    """Main function to run the analysis and handle I/O."""
    METRICS_SUCCESS_FILE = "metrics_analysis_results.csv"
    PRUNING_SUCCESS_FILE = "pruning_analysis_results.csv"
    ERROR_FILE = "pruning_analysis_errors.csv"

    instances = [
#        'random_19',
        #'inst_22_8228',
        #'inst_23_3180',
        #'inst_23_8413',
        #'inst_25_1057',
        #'inst_27_471',
        #'inst_28_8223',
        #'inst_31_8999',
        #'inst_35_8160',
        #'inst_38_1586',
        #'inst_39_1267',
        #'inst_40_390',
        #'inst_42_3753',
        #'inst_47_9234',
        'inst_55_5932',
        'inst_56_6952',
        'inst_61_5846',
        'inst_89_5877',
        'inst_101_4814',
        'inst_128_5584',
        'inst_180_3831',
    ]

    print(f"Starting analysis for {len(instances)} instances...")
    print("-" * 40)

    for inst_name in instances:
        print(f"\nProcessing instance: **{inst_name}**")

        try:
            # ASSUMPTION: The original function returned two DFs,
            # but for logging to a single CSV, they should be merged
            # or returned as a single combined DF.
            # I have modified the placeholder to return a single combined DF:
            df_metrics, df_pruning = analyze_pruning_combinations(inst_name, "instances")

            # --- SUCCESS HANDLING ---
            print("Run successful.")
            #print(df_metrics.to_string(index=False))  # Display the result

            # Add results to the success file
            append_to_csv(df_metrics, METRICS_SUCCESS_FILE)

            # --- SUCCESS HANDLING ---
            #print("Run successful. Merged Data:")
            #print(df_pruning.to_string(index=False))  # Display the result

            # Add results to the success file
            append_to_csv(df_pruning, PRUNING_SUCCESS_FILE)

        except Exception as e:
            # --- ERROR HANDLING ---
            error_type = type(e).__name__
            error_message = str(e).replace('\n', ' | ')  # Clean up message for CSV

            print(f"**FAILURE**: An error occurred ({error_type}): {error_message}")

            # Create a DataFrame for the error log
            error_data = pd.DataFrame({
                'Instance_Name': [inst_name],
                'Error_Type': [error_type],
                'Error_Message': [error_message],
                'Traceback_Snippet': [
                    traceback.format_exc().splitlines()[-3].strip() if traceback.format_exc() else 'N/A']
            })

            # Add error log to the error file
            append_to_csv(error_data, ERROR_FILE)

    print("-" * 40)
    print("Analysis complete.")

def test_complex_actions_problem(manual=False):
    instance = load_instance("inst_22_8228", INSTANCES_FOLDER)
    sim_runner = SimRunner(instance, sim_config=sim_config, robot_config=robot_config)
    if manual:
        sim_runner.execute_manual_simulation(save=True)
    else:
        sim_runner.execute_simulation(save=True)



if __name__ == "__main__":
    test_pruning()
    #test_complex_actions_problem()
