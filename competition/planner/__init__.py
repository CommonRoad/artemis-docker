"""Interactive planning framework; construct planner profiles with planner.spec."""

# The public scenarios and the SMP wheel are not part of the Git repository.
# The first import of planner downloads missing ones; see planner/competition_data.py.
from planner import competition_data as _competition_data

_competition_data.ensure_available()
