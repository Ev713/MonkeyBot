#!/usr/bin/env python3
"""Generate a multi-island MonkeyBot instance JSON file."""

import argparse
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from monkey_bot.island_instance_generator import (
    IslandGeneratorConfig,
    generate_island_instance,
    generate_island_points,
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--name", default="InfiniteIslands")
    parser.add_argument("--instances-folder", default="instances")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--grid-size", type=float, default=100.0)
    parser.add_argument("--islands", type=int, default=14)
    parser.add_argument("--points-per-island-min", type=int, default=12)
    parser.add_argument("--points-per-island-max", type=int, default=28)
    args = parser.parse_args()

    lo = min(args.points_per_island_min, args.points_per_island_max)
    hi = max(args.points_per_island_min, args.points_per_island_max)
    config = IslandGeneratorConfig(
        name=args.name,
        grid_size_x=args.grid_size,
        grid_size_y=args.grid_size,
        num_islands=args.islands,
        target_points_per_island_min=lo,
        target_points_per_island_max=hi,
    )
    instance = generate_island_instance(config, seed=args.seed)
    islands = generate_island_points(config, __import__("random").Random(args.seed))
    sizes = sorted(len(isl) for isl in islands)
    out = Path(args.instances_folder) / f"{args.name}.json"
    instance.to_json_file(out)
    print(
        f"Wrote {out} — {len(instance.gripping_points)} gripping points "
        f"on {len(islands)} islands (sizes {sizes}, seed={args.seed})",
        flush=True,
    )


if __name__ == "__main__":
    main()
