
import pandas as pd
import matplotlib.pyplot as plt
import json
import numpy as np

# --- Data Definition (Simulating data loaded from JSON) ---
# NOTE: Replace this dictionary content with data loaded from your actual JSON file
data_for_json = {
    "P1=0, P2=0, P3=0": [
        {"Time 1 (Total)": 235.10, "Status": "SOLVED_SATISFICING"},
        {"Time 1 (Total)": 527.58, "Status": "SOLVED_SATISFICING"},
        {"Time 1 (Total)": 363.28, "Status": "SOLVED_SATISFICING"},
        {"Time 1 (Total)": 179.39, "Status": "SOLVED_SATISFICING"},
        {"Time 1 (Total)": 397.69, "Status": "SOLVED_SATISFICING"},
        {"Time 1 (Total)": 371.11, "Status": "UNSOLVABLE_INCOMPLETELY"},
        {"Time 1 (Total)": 605.31, "Status": "SOLVED_SATISFICING"},
        {"Time 1 (Total)": 826.76, "Status": "TIMEOUT"},
        {"Time 1 (Total)": 1151.80, "Status": "TIMEOUT"},
        {"Time 1 (Total)": 899.31, "Status": "TIMEOUT"}
    ],
    "P1=1, P2=0, P3=0": [
        {"Time 1 (Total)": 230.77, "Status": "SOLVED_SATISFICING"},
        {"Time 1 (Total)": 474.41, "Status": "SOLVED_SATISFICING"},
        {"Time 1 (Total)": 352.48, "Status": "SOLVED_SATISFICING"},
        {"Time 1 (Total)": 171.22, "Status": "SOLVED_SATISFICING"},
        {"Time 1 (Total)": 386.43, "Status": "SOLVED_SATISFICING"},
        {"Time 1 (Total)": 363.58, "Status": "UNSOLVABLE_INCOMPLETELY"},
        {"Time 1 (Total)": 574.23, "Status": "SOLVED_SATISFICING"},
        {"Time 1 (Total)": 816.40, "Status": "TIMEOUT"},
        {"Time 1 (Total)": 1110.84, "Status": "TIMEOUT"},
        {"Time 1 (Total)": 893.54, "Status": "TIMEOUT"}
    ],
    "P1=0, P2=1, P3=0": [
        {"Time 1 (Total)": 164.66, "Status": "SOLVED_SATISFICING"},
        {"Time 1 (Total)": 175.76, "Status": "SOLVED_SATISFICING"},
        {"Time 1 (Total)": 109.73, "Status": "SOLVED_SATISFICING"},
        {"Time 1 (Total)": 135.39, "Status": "SOLVED_SATISFICING"},
        {"Time 1 (Total)": 252.88, "Status": "SOLVED_SATISFICING"},
        {"Time 1 (Total)": 297.96, "Status": "UNSOLVABLE_INCOMPLETELY"},
        {"Time 1 (Total)": 314.05, "Status": "SOLVED_SATISFICING"},
        {"Time 1 (Total)": 304.36, "Status": "SOLVED_SATISFICING"},
        {"Time 1 (Total)": 453.41, "Status": "SOLVED_SATISFICING"},
        {"Time 1 (Total)": 803.63, "Status": "TIMEOUT"}
    ],
    "P1=0, P2=0, P3=1": [
        {"Time 1 (Total)": 209.01, "Status": "SOLVED_SATISFICING"},
        {"Time 1 (Total)": 537.18, "Status": "SOLVED_SATISFICING"},
        {"Time 1 (Total)": 310.73, "Status": "SOLVED_SATISFICING"},
        {"Time 1 (Total)": 169.86, "Status": "SOLVED_SATISFICING"},
        {"Time 1 (Total)": 384.56, "Status": "SOLVED_SATISFICING"},
        {"Time 1 (Total)": 328.05, "Status": "UNSOLVABLE_INCOMPLETELY"},
        {"Time 1 (Total)": 587.82, "Status": "SOLVED_SATISFICING"},
        {"Time 1 (Total)": 814.97, "Status": "TIMEOUT"},
        {"Time 1 (Total)": 1104.00, "Status": "TIMEOUT"},
        {"Time 1 (Total)": 898.15, "Status": "TIMEOUT"}
    ],
    "P1=1, P2=1, P3=1": [
        {"Time 1 (Total)": 144.25, "Status": "SOLVED_SATISFICING"},
        {"Time 1 (Total)": 176.30, "Status": "SOLVED_SATISFICING"},
        {"Time 1 (Total)": 86.61, "Status": "SOLVED_SATISFICING"},
        {"Time 1 (Total)": 128.98, "Status": "SOLVED_SATISFICING"},
        {"Time 1 (Total)": 244.89, "Status": "SOLVED_SATISFICING"},
        {"Time 1 (Total)": 262.38, "Status": "UNSOLVABLE_INCOMPLETELY"},
        {"Time 1 (Total)": 307.24, "Status": "SOLVED_SATISFICING"},
        {"Time 1 (Total)": 293.44, "Status": "SOLVED_SATISFICING"},
        {"Time 1 (Total)": 428.19, "Status": "SOLVED_SATISFICING"},
        {"Time 1 (Total)": 777.20, "Status": "TIMEOUT"}
    ]
}

# --- Data Processing ---

target_configs = [
    'P1=0, P2=0, P3=0',
    'P1=1, P2=0, P3=0',
    'P1=0, P2=1, P3=0',
    'P1=0, P2=0, P3=1',
    'P1=1, P2=1, P3=1'
]

# Flatten the data structure
long_data = []
for config, problems in data_for_json.items():
    if config in target_configs:
        for p in problems:
            long_data.append({
                'Configuration': config,
                'Time 1 (Total)': p['Time 1 (Total)'],
                'Status': p['Status']
            })

df_long = pd.DataFrame(long_data)

# Filter for "solved" problems (SOLVED_SATISFICING or TIMEOUT)
solved_statuses = ['SOLVED_SATISFICING', 'TIMEOUT']
df_solved = df_long[df_long['Status'].isin(solved_statuses)]

# --- Matplotlib Plotting ---

plt.figure(figsize=(10, 6))
max_time = 1500  # Set a maximum time limit for the plot
max_problems = 10

for config in target_configs:
    # Get sorted times for the current configuration
    config_times = df_solved[df_solved['Configuration'] == config]['Time 1 (Total)'].sort_values().tolist()

    # Define the X and Y coordinates for the step plot
    # Y-axis: Cumulative count (0 to N)
    y_values = np.arange(0, len(config_times) + 1)

    # X-axis: Times (starting at 0, followed by sorted solution times)
    x_values = np.insert(config_times, 0, 0)

    # Plot the step function
    plt.step(x_values, y_values, where='post', label=config)

# Add chart aesthetics
plt.title('Cumulative Solved Problems vs. Total Solution Time (eCDF)', fontsize=14)
plt.xlabel('Total Solution Time (s)', fontsize=12)
plt.ylabel('Cumulative Problems Solved (Count)', fontsize=12)
plt.xlim(0, max_time)
plt.ylim(0, max_problems + 0.5)
plt.yticks(np.arange(0, max_problems + 1, 1))
plt.grid(axis='y', linestyle='--', alpha=0.7)
plt.legend(title='Configuration')
plt.tight_layout()

# Save the plot as a static image
plt.savefig('cumulative_solved_problems_ecdf.png')

print("\nMatplotlib plot generated and saved as cumulative_solved_problems_ecdf.png")