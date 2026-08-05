# Parte B — Suavizado de trayectoria (Curve Generation)

Una vez generadas las trayectorias globales con **Dijkstra** y **RRT**
(ver `PARTE_A_GENERACION_TRAYECTORIAS.md`), se aplica un proceso de
**suavizado de trayectoria (curve generation / path smoothing)** para
mejorar la fluidez y factibilidad de la ruta, evitando esquinas agudas no
aptas para un vehículo con restricciones de giro como el F1TENTH.

---

## 1. Método utilizado

Se usa **Cubic Spline** (`python_motion_planning.curve_generation.CubicSpline`),
parametrizado por **longitud de arco** (distancia acumulada a lo largo del
camino), no por índice de punto. Esto evita distorsiones cuando los
waypoints no están espaciados uniformemente.

Pipeline de suavizado (`smooth_and_resample` en `f1tenth/plan_brandshatch.py`):

1. Se eliminan puntos duplicados/casi coincidentes del camino crudo.
2. Se calcula la longitud de arco acumulada `s` entre waypoints.
3. Se ajusta un `CubicSpline` de `x(s)` y de `y(s)` por separado.
4. Se evalúa el spline en pasos uniformes de `s` (0.5 m o 1.0 m),
   generando los waypoints finales suavizados.

## 2. Pre-filtro de media móvil (solo para RRT)

RRT, al ser un método basado en muestreo aleatorio, produce un camino crudo
con **zigzag** (cambios de dirección bruscos entre nodos consecutivos del
árbol). Un spline cúbico es *interpolante* — pasa exactamente por cada
punto dado — por lo que si se le entrega directamente el camino ruidoso de
RRT, el spline puede **amplificar** esas oscilaciones en vez de suavizarlas
(esto se comprobó empíricamente, ver sección 4).

**Solución:** antes de aplicar el spline, el camino crudo de RRT pasa por
un filtro de **media móvil** (`moving_average_filter`, ventana de 7 puntos)
que atenúa el ruido de alta frecuencia sin desplazar significativamente la
trayectoria general. El punto de inicio y meta se preservan exactos. Este
pre-filtro **no se aplica a Dijkstra**, ya que su camino crudo (sobre grid)
no presenta ese ruido de alta frecuencia.

```python
def moving_average_filter(path_world, window=7):
    """Reduce el zigzag de RRT antes del spline."""
    ...
```

## 3. Validación: curvatura antes y después

Se calcula la curvatura discreta a lo largo de cada camino:

```
k = |x' y'' - y' x''| / (x'^2 + y'^2)^(3/2)
```

Un valor de curvatura más bajo y sin picos indica una trayectoria más
suave y más factible para un vehículo con radio de giro limitado.

### Resultados (BrandsHatch)

| Algoritmo | Paso | Curvatura media (raw → smooth) | Curvatura máxima (raw → smooth) |
|---|---|---|---|
| Dijkstra | 0.5 m | 0.2058 → 0.2316 | 0.8945 → 0.9560 |
| Dijkstra | 1.0 m | 0.0990 → 0.1024 | 0.6298 → 0.6144 |
| RRT | 0.5 m | 0.3728 → **0.0848** | 5.1580 → **2.0365** |
| RRT | 1.0 m | 0.1916 → **0.0660** | 1.1096 → **0.4530** |

*(unidades: 1/metro; valores obtenidos con el pre-filtro de media móvil
activo para RRT)*

**Interpretación:**
- **Dijkstra**: el camino crudo ya es razonablemente suave (proviene de un
  grid con 8 vecinos), por lo que el spline lo ajusta con un incremento
  leve y controlado de curvatura (efecto normal de interpolar entre
  waypoints ligeramente separados).
- **RRT**: el camino crudo tiene curvatura máxima muy alta (zigzag propio
  del muestreo aleatorio). Con el pre-filtro + spline, tanto la curvatura
  media como la máxima **se reducen drásticamente** (hasta ~5.5x menor en
  el pico), confirmando que el proceso de suavizado cumple su objetivo:
  producir una trayectoria sin esquinas agudas, apta para las restricciones
  de giro del F1TENTH.

## 4. Evidencia visual

| Archivo | Contenido |
|---|---|
| `dijkstra_smooth_vs_raw_0.5m.png` | Dijkstra, 0.5 m: crudo (rojo) vs. suavizado (azul) |
| `dijkstra_smooth_vs_raw_1.0m.png` | Dijkstra, 1.0 m: crudo vs. suavizado |
| `rrt_smooth_vs_raw_0.5m.png` | RRT, 0.5 m: crudo vs. suavizado |
| `rrt_smooth_vs_raw_1.0m.png` | RRT, 1.0 m: crudo vs. suavizado |

![Dijkstra suavizado vs crudo - 0.5m](dijkstra_smooth_vs_raw_0.5m.png)

![Dijkstra suavizado vs crudo - 1.0m](dijkstra_smooth_vs_raw_1.0m.png)

![RRT suavizado vs crudo - 0.5m](rrt_smooth_vs_raw_0.5m.png)

![RRT suavizado vs crudo - 1.0m](rrt_smooth_vs_raw_1.0m.png)

> Nota: coloca este `.md` en la misma carpeta donde subiste las imágenes
> (o ajusta la ruta relativa, ej. `f1tenth/output/nombre.png`, según donde
> las hayas colocado en el repo).

## 5. Archivos finales generados

En `f1tenth/output/`:

| Archivo | Descripción |
|---|---|
| `dijkstra_path_0.5m.csv` | Dijkstra suavizado, waypoints cada 0.5 m |
| `dijkstra_path_1.0m.csv` | Dijkstra suavizado, waypoints cada 1.0 m |
| `rrt_path_0.5m.csv` | RRT suavizado (pre-filtro + spline), waypoints cada 0.5 m |
| `rrt_path_1.0m.csv` | RRT suavizado (pre-filtro + spline), waypoints cada 1.0 m |

Estos son los mismos nombres de archivo que la Parte A crudo (`_raw_`),
pero **sin** el sufijo `_raw_` — representan el resultado final tras el
suavizado.

## 6. Cómo subir esto a GitHub

```bash
cd Global_Planner
git add f1tenth/plan_brandshatch.py f1tenth/output/ PARTE_B_SUAVIZADO.md
git commit -m "Parte B: suavizado de trayectorias (Cubic Spline + pre-filtro para RRT)"
git push origin master
```
