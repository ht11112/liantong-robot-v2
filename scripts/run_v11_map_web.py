"""Launch the local V11 map console; no operation is sent on startup."""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'app'))

from v11_map_web import main

if __name__ == '__main__':
    main()
