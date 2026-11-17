"""Visualize driven solutions and compare recorded and simulated traffic."""

from pathlib import Path

import numpy as np
from commonroad.common.file_reader import CommonRoadFileReader
from IPython.display import HTML
from tools.CR_Visualizer import CRVisualizer

from ..runtime.execution import RunResult


def compare_traffic(
    source: str | Path, result: RunResult, step: int = 20
) -> list[dict[str, object]]:
    """Show how SUMO's driven actors differ from the recorded XML at one time."""
    recorded, _ = CommonRoadFileReader(str(source)).open()
    simulated = {actor.obstacle_id: actor for actor in result.scenario.dynamic_obstacles}
    rows = []
    for actor in recorded.dynamic_obstacles:
        before = actor.state_at_time(step)
        observed = simulated.get(actor.obstacle_id)
        after = observed.state_at_time(step) if observed else None
        rows.append(
            {
                "actor": actor.obstacle_id,
                "recorded_position": None if before is None else before.position.round(2).tolist(),
                "interactive_position": None if after is None else after.position.round(2).tolist(),
                "distance_m": (
                    None
                    if before is None or after is None
                    else round(float(np.linalg.norm(before.position - after.position)), 3)
                ),
            }
        )
    return rows


def animate(result: RunResult) -> HTML:
    return CRVisualizer.animate_scenario(result.scenario, result.planning_problems, result.solution)
