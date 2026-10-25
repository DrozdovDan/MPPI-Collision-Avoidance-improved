"""Reproducible scenario data and evaluation adapters for existing planners."""

SCHEMA_VERSION = 1
MAP_NAMES = ("parking1", "parking2", "parking3", "warehouse", "warehouse2")
CATEGORIES = (
    ("random_low", 4, "ordinary"),
    ("random_moderate", 8, "ordinary"),
    ("random_dense", 16, "ordinary"),
    ("opposing_bottleneck", 8, "challenging"),
    ("opposing_dense", 16, "challenging"),
    ("intersection", 12, "challenging"),
    ("symmetric_swap", 12, "deadlock_prone"),
    ("competing_bottlenecks", 20, "deadlock_prone"),
    ("heterogeneous_congestion", 24, "deadlock_prone"),
    ("maximum_stress", 32, "deadlock_prone"),
)
