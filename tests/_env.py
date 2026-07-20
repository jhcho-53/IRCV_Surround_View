import os, sys
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))  # pipeline root


def skip(reason):
    """Exit 0 with a SKIP marker so data-dependent tests don't fail a clone-only run."""
    print(f"SKIP {os.path.basename(sys.argv[0])}: {reason}")
    raise SystemExit(0)


def require(path, what):
    """Skip (not fail) when externally-supplied data is missing from this checkout."""
    if not os.path.exists(path):
        skip(f"{what} not found ({path}); supply the external data or set AVM_DATA_ROOT")
