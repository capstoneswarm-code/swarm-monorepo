"""Allow python -m swarm_anomaly as well as the installed command."""

from swarm_anomaly.cli import main

if __name__ == "__main__":
    raise SystemExit(main())
