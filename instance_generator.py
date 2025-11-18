import json
import os
import random


grid_size_x = random.randint(10, 20)
grid_size_y = random.randint(10, 20)
density = random.random()*0.5+0.2

goal_point = [grid_size_x-2, grid_size_y-2]
init_center = [2, 2]

init_feet = [[1, 1], [3, 1], [1, 3]]

all_points = [[x+1, y+1] for x in range(grid_size_x) for y in range(grid_size_y) if [x+1, y+1] not in init_feet and x%2==0 and y %2==0]

print(f"{len(all_points)} possible grip points")
new_points = random.sample(all_points, int(len(all_points)*density))
print(f"{len(new_points)} grip points chosen")

name = f"inst_{len(init_feet)+len(new_points)}_{random.randint(0, 9999)}"

instance = {
  "name": name,
  "leg_extension": 3,
  "grid_size_x": grid_size_x,
  "grid_size_y": grid_size_y,
  "gripping_points": init_feet.copy()+new_points,
  "goal_point": goal_point,
  "init_center": init_center,
  "init_feet": init_feet
}

out_dir = "instances"
os.makedirs(out_dir, exist_ok=True)
file_path = os.path.join(out_dir, f"{name}.json")

with open(file_path, "w") as f:
    json.dump(instance, f, indent=2)

print(f"✅ Instance saved to {file_path}")