import json
import os
import random

grid_size_x = 10
grid_size_y = 10

goal_point = [grid_size_x-2, grid_size_y-2]
init_center = [2, 2]

init_feet = [[1, 1], [3, 1], [1, 3]]

name = 'Random'
instance = {
  "name": name,
  "leg_extension": 3,
  "grid_size_x": grid_size_x,
  "grid_size_y": grid_size_y,
  "gripping_points": init_feet.copy(),
  "goal_point": goal_point,
  "init_center": init_center,
  "init_feet": init_feet
}

all_points = [[x+1, y+1] for x in range(grid_size_x) for y in range(grid_size_y) if [x+1, y+1] not in init_feet and x%2==0 and y %2==0]

new_points = all_points
print(f"{len(new_points)} possible grip points")
new_points = random.sample(all_points, 16)

instance['gripping_points'] += new_points


out_dir = "instances"
os.makedirs(out_dir, exist_ok=True)
file_path = os.path.join(out_dir, f"{name}.json")

with open(file_path, "w") as f:
    json.dump(instance, f, indent=2)

print(f"✅ Instance saved to {file_path}")