"""
plan_brandshatch.py

Genera trayectorias globales para el mapa BrandsHatch usando dos algoritmos:
  - Dijkstra (algoritmo asignado por el docente)
  - RRT (segunda tecnica de planificacion)

Para cada algoritmo se generan 2 archivos de waypoints (separacion 0.5 m y 1 m),
aplicando un proceso de suavizado (Cubic Spline, tomado de
python_motion_planning.curve_generation) sobre el camino crudo entregado por
el planificador.

Salidas (carpeta f1tenth/output/):
  dijkstra_path_0.5m.csv
  dijkstra_path_1.0m.csv
  rrt_path_0.5m.csv
  rrt_path_1.0m.csv
  dijkstra_vs_rrt.png   (comparacion visual de ambas trayectorias sobre el mapa)
"""

import os
import sys
import csv
import math
import time
from pathlib import Path

import cv2
import yaml
import numpy as np
from scipy.ndimage import distance_transform_edt
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from python_motion_planning.utils import Grid, SearchFactory
from python_motion_planning.global_planner.sample_search.rrt import RRT
from python_motion_planning.utils.environment.node import Node
from python_motion_planning.curve_generation.cubic_spline import CubicSpline


# --------------------------------------------------------------------------
# 1. Utilidades de mapa
# --------------------------------------------------------------------------
def load_map(yaml_path, downsample_factor=1):
    yaml_path = Path(yaml_path)
    with yaml_path.open('r') as f:
        map_config = yaml.safe_load(f)

    img_path = Path(map_config['image'])
    if not img_path.is_absolute():
        img_path = (yaml_path.parent / img_path).resolve()
    map_img = cv2.imread(str(img_path), cv2.IMREAD_GRAYSCALE)
    resolution = map_config['resolution']
    origin = map_config['origin']

    map_bin = np.zeros_like(map_img, dtype=np.uint8)
    map_bin[map_img < int(0.45 * 255)] = 1

    map_bin = cv2.dilate(map_bin, np.ones((3, 3), np.uint8), iterations=1)

    h, w = map_bin.shape
    new_h, new_w = h // downsample_factor, w // downsample_factor
    h_crop, w_crop = new_h * downsample_factor, new_w * downsample_factor
    map_bin = map_bin[:h_crop, :w_crop]

    map_bin = map_bin.reshape(new_h, downsample_factor, new_w, downsample_factor)
    map_bin = map_bin.max(axis=(1, 3)).astype(np.uint8)

    resolution *= downsample_factor
    return map_bin, resolution, origin


def grid_from_map(map_bin):
    h, w = map_bin.shape
    env = Grid(w, h)
    # Sin flip vertical: map_bin[y, x] usa y = indice de fila directamente,
    # igual que start_map/goal_map (pick_lap_start_goal) y map_to_world/world_to_map.
    # Un flip aqui desalineaba los obstaculos vistos por el planner respecto
    # a las coordenadas de start/goal, causando "no encontro camino".
    obstacles = {(x, y) for y in range(h) for x in range(w) if map_bin[y, x] == 1}
    env.update(obstacles)
    return env


def world_to_map(x_world, y_world, resolution, origin):
    x_map = int((x_world - origin[0]) / resolution)
    y_map = int((y_world - origin[1]) / resolution)
    return (x_map, y_map)


def map_to_world(x_map, y_map, resolution, origin):
    x_world = x_map * resolution + origin[0]
    y_world = y_map * resolution + origin[1]
    return (x_world, y_world)


def world_to_map_f(x_world, y_world, resolution, origin):
    """Version float (sin redondear) de world_to_map, necesaria para
    interpolar el campo de distancias con precision sub-celda."""
    x_map = (x_world - origin[0]) / resolution
    y_map = (y_world - origin[1]) / resolution
    return (x_map, y_map)


def compute_distance_field(map_bin):
    """Para cada celda libre, distancia (en celdas) a la pared/obstaculo
    mas cercano. El maximo de este campo, a lo largo del eje transversal
    de la pista, define la centerline del corredor."""
    free = (map_bin == 0).astype(np.uint8)
    return distance_transform_edt(free)


def sample_bilinear(field, x, y):
    h, w = field.shape
    x = min(max(x, 0.0), w - 1.0)
    y = min(max(y, 0.0), h - 1.0)
    x0, y0 = int(np.floor(x)), int(np.floor(y))
    x1, y1 = min(x0 + 1, w - 1), min(y0 + 1, h - 1)
    fx, fy = x - x0, y - y0
    v00, v10 = field[y0, x0], field[y0, x1]
    v01, v11 = field[y1, x0], field[y1, x1]
    return (v00 * (1 - fx) * (1 - fy) + v10 * fx * (1 - fy)
            + v01 * (1 - fx) * fy + v11 * fx * fy)


def pull_to_centerline(path_world, dist_field, resolution, origin,
                        iterations=15, step=0.4, delta=1.0):
    """Empuja cada punto del camino (excepto el primero y el ultimo)
    en direccion al gradiente del campo de distancias, es decir hacia
    el centro del pasillo, alejandolo de las paredes. Se repite varias
    iteraciones para converger a la linea media del corredor."""
    pts = [list(world_to_map_f(x, y, resolution, origin)) for x, y in path_world]
    h, w = dist_field.shape

    for _ in range(iterations):
        new_pts = [pts[0]]
        for i in range(1, len(pts) - 1):
            x, y = pts[i]
            gx = sample_bilinear(dist_field, x + delta, y) - sample_bilinear(dist_field, x - delta, y)
            gy = sample_bilinear(dist_field, x, y + delta) - sample_bilinear(dist_field, x, y - delta)
            norm = math.hypot(gx, gy)
            if norm > 1e-6:
                gx, gy = gx / norm, gy / norm
            else:
                gx, gy = 0.0, 0.0
            nx = min(max(x + step * gx, 0.0), w - 1.0)
            ny = min(max(y + step * gy, 0.0), h - 1.0)
            new_pts.append([nx, ny])
        new_pts.append(pts[-1])
        pts = new_pts

    return [map_to_world(x, y, resolution, origin) for x, y in pts]


def save_path_as_csv(path_world, filename):
    with open(filename, mode='w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(["x", "y"])
        for x, y in path_world:
            writer.writerow([x, y])


def isolate_track_corridor(map_bin):
    """Conserva como libre (0) solo la componente conexa que corresponde
    al pasillo de la pista. Convierte en obstaculo (1) tanto el exterior
    del circuito como el infield (interior del loop), para que ningun
    planner pueda cortar camino atravesando esas zonas."""
    free = (map_bin == 0).astype(np.uint8)
    num_labels, labels = cv2.connectedComponents(free, connectivity=4)

    h, w = map_bin.shape
    border_labels = set(labels[0, :]) | set(labels[h - 1, :]) | \
                    set(labels[:, 0]) | set(labels[:, w - 1])
    border_labels.discard(0)

    best_label, best_extent = None, -1
    for lbl in range(1, num_labels):
        if lbl in border_labels:
            continue
        ys, xs = np.where(labels == lbl)
        extent = (xs.max() - xs.min()) * (ys.max() - ys.min())
        if extent > best_extent:
            best_extent = extent
            best_label = lbl

    new_map_bin = np.ones_like(map_bin)
    new_map_bin[labels == best_label] = 0
    return new_map_bin


def add_start_finish_wall(map_bin, cut_point, half_width=8):
    """Dibuja una pared delgada perpendicular a la pista en cut_point,
    partiendo el anillo cerrado. Asi Dijkstra/RRT deben recorrer TODA
    la vuelta para conectar start y goal."""
    x0, y0 = cut_point
    h, w = map_bin.shape
    window = 5
    y_lo, y_hi = max(0, y0 - window), min(h, y0 + window + 1)
    x_lo, x_hi = max(0, x0 - window), min(w, x0 + window + 1)
    ys, xs = np.mgrid[y_lo:y_hi, x_lo:x_hi]
    mask_local = map_bin[y_lo:y_hi, x_lo:x_hi] == 0
    pts = np.column_stack([xs[mask_local], ys[mask_local]]).astype(np.float64)
    pts -= pts.mean(axis=0)
    _, _, vt = np.linalg.svd(pts, full_matrices=False)
    track_dir = vt[0]
    perp = np.array([-track_dir[1], track_dir[0]])

    new_map_bin = map_bin.copy()
    for t in np.linspace(-half_width, half_width, int(half_width * 4)):
        xx = int(round(x0 + perp[0] * t))
        yy = int(round(y0 + perp[1] * t))
        if 0 <= yy < h and 0 <= xx < w:
            new_map_bin[yy, xx] = 1
    return new_map_bin, track_dir


def pick_lap_start_goal(map_bin, offset=6, max_offset=20):
    """Ubica un punto de corte sobre el pasillo, traza la pared de meta
    ahi, y coloca start/goal a cada lado, a lo largo de la direccion de
    la pista. Verifica que ambos caigan en celda libre (no sobre el
    muro); si no, aumenta el offset hasta que lo esten."""
    free = (map_bin == 0)
    ys, xs = np.where(free)
    x_cut = xs.min() + 10
    col_ys = ys[xs == x_cut]
    if len(col_ys) == 0:
        raise RuntimeError("No se encontro pasillo libre en la columna de corte.")
    y_cut = int(np.median(col_ys))
    cut_point = (x_cut, y_cut)

    map_bin_cut, track_dir = add_start_finish_wall(map_bin, cut_point, half_width=8)
    h, w = map_bin_cut.shape

    def is_free(pt):
        x, y = pt
        return 0 <= x < w and 0 <= y < h and map_bin_cut[y, x] == 0

    off = offset
    while off <= max_offset:
        start_map = (int(round(x_cut - track_dir[0] * off)),
                     int(round(y_cut - track_dir[1] * off)))
        goal_map = (int(round(x_cut + track_dir[0] * off)),
                    int(round(y_cut + track_dir[1] * off)))
        if is_free(start_map) and is_free(goal_map):
            print(f"  (pick_lap_start_goal) offset usado: {off} celdas")
            return map_bin_cut, start_map, goal_map
        off += 1

    raise RuntimeError(
        f"No se pudo ubicar start/goal en celda libre hasta offset={max_offset}. "
        f"Revisa el ancho de la pared (half_width) vs el ancho real de la pista."
    )


# --------------------------------------------------------------------------
# 2. RRT adaptado a rejilla (Grid) con chequeo de colision O(1)
# --------------------------------------------------------------------------
class GridRRT(RRT):
    """RRT que verifica colisiones contra un entorno Grid (rejilla discreta)
    en lugar de un entorno Map (rectangulos/circulos)."""

    def __init__(self, start, goal, env: Grid, max_dist=3.0,
                 sample_num=20000, goal_sample_rate=0.1, inflation=1):
        from python_motion_planning.utils.planner.planner import Planner
        Planner.__init__(self, start, goal, env)
        self.delta = 0.5
        self.max_dist = max_dist
        self.sample_num = sample_num
        self.goal_sample_rate = goal_sample_rate
        self.inflation = inflation

        # Precomputa las celdas libres reales de la pista. Muestrear
        # uniforme sobre todo el bounding box del grid desperdicia casi
        # todas las muestras en zonas fuera del pasillo (corredor angosto
        # y sinuoso); muestrear entre celdas libres reales concentra el
        # 100% de las muestras dentro de la pista.
        all_cells = {(x, y) for x in range(self.env.x_range) for y in range(self.env.y_range)}
        self.free_cells = list(all_cells - set(self.env.obstacles))
        if not self.free_cells:
            raise RuntimeError("El grid no tiene celdas libres.")

    def __str__(self):
        return "RRT (grid-based)"

    def _cell_free(self, x, y):
        xi, yi = int(round(x)), int(round(y))
        if xi < 0 or yi < 0 or xi >= self.env.x_range or yi >= self.env.y_range:
            return False
        if self.inflation <= 0:
            return (xi, yi) not in self.env.obstacles
        for dx in range(-self.inflation, self.inflation + 1):
            for dy in range(-self.inflation, self.inflation + 1):
                if (xi + dx, yi + dy) in self.env.obstacles:
                    return False
        return True

    def isInsideObs(self, node: Node) -> bool:
        return not self._cell_free(node.x, node.y)

    def isCollision(self, node1: Node, node2: Node) -> bool:
        if self.isInsideObs(node1) or self.isInsideObs(node2):
            return True
        dist = self.dist(node1, node2)
        steps = max(1, int(math.ceil(dist)))
        for i in range(steps + 1):
            t = i / steps
            x = node1.x + t * (node2.x - node1.x)
            y = node1.y + t * (node2.y - node1.y)
            if not self._cell_free(x, y):
                return True
        return False

    def generateRandomNode(self) -> Node:
        if np.random.random() > self.goal_sample_rate:
            idx = np.random.randint(0, len(self.free_cells))
            cx, cy = self.free_cells[idx]
            # jitter dentro de la celda para no muestrear siempre el mismo punto exacto
            current = (cx + np.random.uniform(-0.5, 0.5), cy + np.random.uniform(-0.5, 0.5))
            return Node(current, None, 0, 0)
        return self.goal


# --------------------------------------------------------------------------
# 3. Suavizado + remuestreo a distancia fija entre waypoints (0.5 m / 1 m)
# --------------------------------------------------------------------------
def resample_linear(path_world, step):
    """Parte A: remuestreo por longitud de arco, SIN suavizado de curva.
    Es la trayectoria cruda del planificador, solo con waypoints
    espaciados uniformemente cada `step` metros."""
    pts = [path_world[0]]
    for p in path_world[1:]:
        if math.hypot(p[0] - pts[-1][0], p[1] - pts[-1][1]) > 1e-6:
            pts.append(p)
    if len(pts) < 2:
        return pts
    x_list = [p[0] for p in pts]
    y_list = [p[1] for p in pts]
    dx, dy = np.diff(x_list), np.diff(y_list)
    ds = [math.hypot(a, b) for a, b in zip(dx, dy)]
    s = [0.0]
    s.extend(np.cumsum(ds))
    target = np.arange(0, s[-1], step)
    x_r = np.interp(target, s, x_list)
    y_r = np.interp(target, s, y_list)
    path = list(zip(x_r, y_r))
    path.append((x_list[-1], y_list[-1]))
    return path


def path_curvature_stats(path):
    """Curvatura media y maxima (1/m) de un path [(x,y), ...]."""
    pts = np.asarray(path)
    if len(pts) < 5:
        return 0.0, 0.0
    dx = np.gradient(pts[:, 0])
    dy = np.gradient(pts[:, 1])
    ddx = np.gradient(dx)
    ddy = np.gradient(dy)
    denom = (dx**2 + dy**2) ** 1.5
    denom[denom < 1e-9] = 1e-9
    k = np.abs(dx * ddy - dy * ddx) / denom
    return float(np.mean(k)), float(np.max(k))


def moving_average_filter(path_world, window=5):
    """Pre-filtro de media movil para reducir el zigzag de RRT antes
    del spline. window debe ser impar."""
    pts = np.asarray(path_world)
    if len(pts) < window:
        return path_world
    half = window // 2
    padded = np.pad(pts, ((half, half), (0, 0)), mode="edge")
    kernel = np.ones(window) / window
    x_f = np.convolve(padded[:, 0], kernel, mode="valid")
    y_f = np.convolve(padded[:, 1], kernel, mode="valid")
    # conserva start y goal exactos
    x_f[0], y_f[0] = pts[0]
    x_f[-1], y_f[-1] = pts[-1]
    return list(zip(x_f, y_f))


def smooth_and_resample(path_world, step):
    pts = [path_world[0]]
    for p in path_world[1:]:
        if math.hypot(p[0] - pts[-1][0], p[1] - pts[-1][1]) > 1e-6:
            pts.append(p)
    if len(pts) < 3:
        return pts

    x_list = [p[0] for p in pts]
    y_list = [p[1] for p in pts]
    dx, dy = np.diff(x_list), np.diff(y_list)
    ds = [math.hypot(a, b) for a, b in zip(dx, dy)]
    s = [0.0]
    s.extend(np.cumsum(ds))

    eps = 1e-9
    t = np.arange(0, s[-1] - eps, step)

    cs = CubicSpline(step)
    path_x, _ = cs.spline(s, x_list, t)
    path_y, _ = cs.spline(s, y_list, t)
    path = list(zip(path_x, path_y))
    path.append((x_list[-1], y_list[-1]))
    return path


# --------------------------------------------------------------------------
# 4. Main
# --------------------------------------------------------------------------
if __name__ == "__main__":
    HERE = Path(__file__).resolve().parent
    yaml_path = HERE.parent / "Mapas-F1Tenth" / "BrandsHatch_map.yaml"
    downsample_factor = 8  # celda ~ 0.4 m

    out_dir = HERE / "output"
    out_dir.mkdir(exist_ok=True)

    print("Cargando mapa BrandsHatch...")
    map_bin, resolution, origin = load_map(yaml_path, downsample_factor)
    map_bin = isolate_track_corridor(map_bin)
    map_bin, start_map, goal_map = pick_lap_start_goal(map_bin, offset=6)
    env = grid_from_map(map_bin)
    print(f"Rejilla: {env.x_range} x {env.y_range} celdas, resolucion {resolution:.3f} m/celda")

    dist_field = compute_distance_field(map_bin)

    start_world = map_to_world(*start_map, resolution, origin)
    goal_world = map_to_world(*goal_map, resolution, origin)
    print(f"Start (grid): {start_map} -> world {start_world}")
    print(f"Goal  (grid): {goal_map} -> world {goal_world}")

    results = {}

    # ---------------- Dijkstra ----------------
    print("\n[1/2] Ejecutando Dijkstra...")
    t0 = time.time()
    planner = SearchFactory()("dijkstra", start=start_map, goal=goal_map, env=env)
    cost, path, _ = planner.plan()
    if not isinstance(cost, (int, float)) or not path:
        raise RuntimeError(
            "Dijkstra no encontro camino entre start y goal. "
            "Puede que la pared de meta este bloqueando todo el pasillo "
            "(reduce half_width en add_start_finish_wall) o que start/goal "
            "hayan quedado en una zona aislada."
        )
    print(f"  Dijkstra: {len(path)} nodos, costo={cost:.2f}, t={time.time()-t0:.2f}s")
    path_world = [map_to_world(x, y, resolution, origin) for x, y in reversed(path)]
    path_world = pull_to_centerline(path_world, dist_field, resolution, origin)
    results["dijkstra"] = path_world

    # ---------------- RRT (grid-based) ----------------
    print("\n[2/2] Ejecutando RRT...")
    t0 = time.time()
    rrt_planner = GridRRT(start_map, goal_map, env,
                           max_dist=3.0, sample_num=20000,
                           goal_sample_rate=0.1, inflation=0)
    cost_rrt, path_rrt, expand_rrt = rrt_planner.plan()
    if path_rrt is None:
        # Diagnostico: que tan cerca del goal llego el arbol (expand_rrt
        # es la lista real de nodos explorados, devuelta por plan()).
        if expand_rrt:
            dists = [math.hypot(n.x - goal_map[0], n.y - goal_map[1]) for n in expand_rrt]
            print(f"  DEBUG: nodos explorados={len(expand_rrt)}, "
                  f"distancia minima al goal={min(dists):.2f} celdas "
                  f"(~{min(dists)*resolution:.2f} m), max_dist={rrt_planner.max_dist} celdas")
        else:
            print("  DEBUG: expand_rrt vacio.")
        raise RuntimeError("RRT no encontro un camino: aumenta sample_num o max_dist.")
    print(f"  RRT: {len(path_rrt)} nodos, costo={cost_rrt:.2f}, t={time.time()-t0:.2f}s")
    path_rrt_world = [map_to_world(x, y, resolution, origin) for x, y in reversed(path_rrt)]
    path_rrt_world = pull_to_centerline(path_rrt_world, dist_field, resolution, origin)
    results["rrt"] = path_rrt_world

    # ---------------- Parte A: crudo remuestreado (0.5 m y 1.0 m) ----------------
    print("\n=== PARTE A: trayectorias crudas remuestreadas ===")
    raw_results = {}
    for name, raw_path in results.items():
        raw_results[name] = {}
        for step in (0.5, 1.0):
            raw_r = resample_linear(raw_path, step)
            raw_results[name][step] = raw_r
            fname = out_dir / f"{name}_path_raw_{step}m.csv"
            save_path_as_csv(raw_r, fname)
            print(f"  Guardado {fname} ({len(raw_r)} waypoints, paso={step} m)")

            # Grafica individual: SOLO crudo (Parte A), sin suavizar
            raw_xy_plot = [world_to_map(x, y, resolution, origin) for x, y in raw_r]
            plt.figure(figsize=(8, 8))
            plt.imshow(map_bin, cmap="gray_r", origin="lower")
            plt.plot(*zip(*raw_xy_plot), "-r", linewidth=2,
                     label=f"{name.capitalize()} crudo ({step} m)")
            plt.plot(start_map[0], start_map[1], "og", markersize=8, label="Start")
            plt.plot(goal_map[0], goal_map[1], "om", markersize=8, label="Goal")
            plt.legend()
            plt.title(f"{name.capitalize()} - Parte A - crudo, paso {step} m ({len(raw_r)} pts)")
            fname_png = out_dir / f"{name}_raw_{step}m.png"
            plt.savefig(fname_png, dpi=150)
            plt.close()
            print(f"  Grafica guardada en {fname_png}")

    # ---------------- Parte B: suavizado (Cubic Spline) sobre lo anterior ----------------
    print("\n=== PARTE B: trayectorias suavizadas ===")
    for name, raw_path in results.items():
        # RRT es mas ruidoso (zigzag) que Dijkstra: se pre-filtra con
        # media movil antes del spline para evitar que el spline
        # interpolante amplifique picos de curvatura.
        pre_path = moving_average_filter(raw_path, window=7) if name == "rrt" else raw_path
        for step in (0.5, 1.0):
            smoothed = smooth_and_resample(pre_path, step)
            fname = out_dir / f"{name}_path_{step}m.csv"
            save_path_as_csv(smoothed, fname)

            k_mean_raw, k_max_raw = path_curvature_stats(raw_results[name][step])
            k_mean_sm, k_max_sm = path_curvature_stats(smoothed)
            print(f"  Guardado {fname} ({len(smoothed)} waypoints, paso={step} m)")
            print(f"    Curvatura media  raw={k_mean_raw:.4f} -> smooth={k_mean_sm:.4f} 1/m")
            print(f"    Curvatura maxima raw={k_max_raw:.4f} -> smooth={k_max_sm:.4f} 1/m")

    # ---------------- Grafica comparativa ----------------
    plt.figure(figsize=(8, 8))
    plt.imshow(map_bin, cmap="gray_r", origin="lower")
    dijkstra_xy = [world_to_map(x, y, resolution, origin) for x, y in results["dijkstra"]]
    rrt_xy = [world_to_map(x, y, resolution, origin) for x, y in results["rrt"]]
    plt.plot(*zip(*dijkstra_xy), "-b", linewidth=2, label="Dijkstra")
    plt.plot(*zip(*rrt_xy), "-g", linewidth=2, label="RRT")
    plt.plot(start_map[0], start_map[1], "og", markersize=8, label="Start")
    plt.plot(goal_map[0], goal_map[1], "or", markersize=8, label="Goal")
    plt.legend()
    plt.title("BrandsHatch - Dijkstra vs RRT")
    plt.savefig(out_dir / "dijkstra_vs_rrt.png", dpi=150)
    print(f"\nGrafica guardada en {out_dir / 'dijkstra_vs_rrt.png'}")
    plt.close()

    # ---------------- Parte B: graficas comparativas raw vs suavizado ----------------
    for name in results.keys():
        for step in (0.5, 1.0):
            raw_r = raw_results[name][step]
            smoothed_path = out_dir / f"{name}_path_{step}m.csv"
            import csv as _csv
            with open(smoothed_path) as fcsv:
                reader = _csv.reader(fcsv)
                next(reader)
                smooth_r = [(float(row[0]), float(row[1])) for row in reader]

            raw_xy = [world_to_map(x, y, resolution, origin) for x, y in raw_r]
            smooth_xy = [world_to_map(x, y, resolution, origin) for x, y in smooth_r]

            plt.figure(figsize=(8, 8))
            plt.imshow(map_bin, cmap="gray_r", origin="lower")
            plt.plot(*zip(*raw_xy), "-r", linewidth=1, alpha=0.6, label="Crudo (sin suavizar)")
            plt.plot(*zip(*smooth_xy), "-b", linewidth=2, label="Suavizado (Cubic Spline)")
            plt.plot(start_map[0], start_map[1], "og", markersize=8, label="Start")
            plt.plot(goal_map[0], goal_map[1], "om", markersize=8, label="Goal")
            plt.legend()
            plt.title(f"{name.capitalize()} - Parte B - paso {step} m")
            fname = out_dir / f"{name}_smooth_vs_raw_{step}m.png"
            plt.savefig(fname, dpi=150)
            plt.close()
            print(f"Grafica guardada en {fname}")

    # ---------------- Graficas individuales por metodo ----------------
    individual_plots = [
        ("dijkstra", dijkstra_xy, "b", "Dijkstra - BrandsHatch"),
        ("rrt", rrt_xy, "g", "RRT - BrandsHatch"),
    ]
    for name, xy, color, title in individual_plots:
        plt.figure(figsize=(8, 8))
        plt.imshow(map_bin, cmap="gray_r", origin="lower")
        plt.plot(*zip(*xy), f"-{color}", linewidth=2, label=name.capitalize())
        plt.plot(start_map[0], start_map[1], "og", markersize=8, label="Start")
        plt.plot(goal_map[0], goal_map[1], "or", markersize=8, label="Goal")
        plt.legend()
        plt.title(title)
        fname = out_dir / f"{name}_path.png"
        plt.savefig(fname, dpi=150)
        plt.close()
        print(f"Grafica guardada en {fname}")

    print("\nListo.")
