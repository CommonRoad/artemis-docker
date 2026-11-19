"""Interactive planning framework; construct planner profiles with planner.spec."""

# Slim checkouts omit the public scenarios and SMP wheel.
# The first import of planner downloads missing ones; see planner/competition_data.py.
from planner import competition_data as _competition_data

_competition_data.ensure_available()
