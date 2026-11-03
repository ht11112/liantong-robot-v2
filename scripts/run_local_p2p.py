"""Launch only the local point-to-point broker; never starts a motion job."""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/"app"))

from local_p2p_server import main

if __name__ == "__main__":
    main()
