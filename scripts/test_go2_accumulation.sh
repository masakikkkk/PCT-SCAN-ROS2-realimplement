#!/usr/bin/env bash
# No ROS, SDK or robot required. Compiles the actual map core and PCT A*.
set -euo pipefail
GO2_TEST_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
GO2_TEST_TMP="$(mktemp -d /tmp/go2-accumulation-test.XXXXXX)"
trap 'rm -rf "$GO2_TEST_TMP"' EXIT
cd "$GO2_TEST_ROOT"
g++ -std=c++17 -O2 -Wall -Wextra -Wpedantic -fsanitize=undefined \
  -Igo2_adapter_ws/src/go2_map_maintenance/include \
  go2_adapter_ws/src/go2_map_maintenance/test/test_voxel_map.cpp \
  -o "$GO2_TEST_TMP/map_core"
"$GO2_TEST_TMP/map_core" "$GO2_TEST_TMP/room.xyz"
PYTHONNOUSERSITE=1 \
PYTHONPATH="$GO2_TEST_ROOT/go2_adapter_ws/src/go2_pct_adapter:$GO2_TEST_ROOT/src/pct/tomography" \
  /usr/bin/python3 - "$GO2_TEST_TMP" <<'PY'
import sys
from pathlib import Path
import numpy as np
from go2_pct_adapter.online_map import build_tomogram
root = Path(sys.argv[1])
points = np.loadtxt(str(root / 'room.xyz'), dtype=np.float32)
snapshot = build_tomogram(points, {'tomogram_backend': 'cpu'})
cost = snapshot['data'][0]
ground = snapshot['data'][3]
layers, nx, ny = cost.shape
center, resolution = snapshot['center'], snapshot['resolution']
def index(xy):
    return np.round((np.asarray(xy)-center)/resolution).astype(int) + np.array([nx//2,ny//2])
start, goal = index((1.,1.)), index((5.,3.))
valid = [s for s in range(layers) if all(np.isfinite(ground[s,p[0],p[1]]) and
          cost[s,p[0],p[1]] <= 20 for p in (start,goal))]
if not valid:
    raise RuntimeError('no observed traversable floor at test endpoints')
# Native A* expects layer * y rows, x columns; tomography stores [layer,x,y].
native_cost = cost.transpose(0,2,1).reshape(layers*ny,nx)
native_ground = np.nan_to_num(ground.transpose(0,2,1),nan=-100.).reshape(layers*ny,nx)
with (root/'astar.txt').open('w') as out:
    out.write('%d %d %d %.8f %d %d %d %d %d\n' %
              (layers,ny,nx,resolution,valid[0],*start,*goal))
    np.savetxt(out,np.column_stack((native_cost.ravel(),native_ground.ravel())),fmt='%.8f')
print('Real CPU tomogram: %d accumulated points, %d layers, %dx%d cells' %
      (len(points),layers,nx,ny))
PY
g++ -std=c++17 -O2 -Wall -Wextra -fsanitize=undefined \
  -I/usr/include/eigen3 -Isrc/pct/pct_planner/lib/src \
  go2_adapter_ws/src/go2_map_maintenance/test/test_accumulated_astar.cpp \
  src/pct/pct_planner/lib/src/a_star/a_star_search.cc \
  -o "$GO2_TEST_TMP/astar"
"$GO2_TEST_TMP/astar" "$GO2_TEST_TMP/astar.txt"
