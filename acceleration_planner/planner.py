
import math
import logging
import numpy as np
from enum import Enum, auto
from dataclasses import dataclass, field

from . import config as cfg

logger = logging.getLogger(__name__)


Point = tuple[float, float]


@dataclass
class TrajectoryData:
    
    midpoints:   list[Point]
    slope:       float          
    intercept:   float
    finish_line: list[Point]    
    finish_y:    float          
    start_y:     float        
    advance_axis: str           


@dataclass
class CarState:
    
    class Phase(Enum):
        WAITING      = auto()
        ACCELERATING = auto()
        BRAKING      = auto()
        FINISHED     = auto()

    phase:    Phase  = Phase.WAITING 
    position: Point  = (0.0, 0.0) 
    speed:    float  = 0.0              
    distance_to_finish: float = 0.0


def infer_advance_axis(yellow: list[Point], blue: list[Point]) -> str:
   
   
    all_cones = yellow + blue
    if len(all_cones) < 2:
        logger.warning("Cones insuficientes para inferir eixo — a assumir 'y'.")
        return "y"

    xs = [p[0] for p in all_cones]
    ys = [p[1] for p in all_cones]
    axis = "y" if np.var(ys) >= np.var(xs) else "x"
    logger.info(f"Eixo de avanço inferido: '{axis}' (var_x={np.var(xs):.2f}, var_y={np.var(ys):.2f})")
    return axis


def get_advance_coord(point: Point, axis: str) -> float:
  
    return point[1] if axis == "y" else point[0]


def get_lateral_coord(point: Point, axis: str) -> float:

    return point[0] if axis == "y" else point[1]



def match_cones(
    yellow: list[Point],
    blue:   list[Point],
    axis:   str,
) -> list[tuple[Point, Point]]:
   
    if not yellow or not blue:
        logger.warning("Sem cones suficientes para fazer matching.")
        return []

    sorted_yellow = sorted(yellow, key=lambda p: get_advance_coord(p, axis))
    sorted_blue   = sorted(blue,   key=lambda p: get_advance_coord(p, axis))

    used_blue = set()
    pairs: list[tuple[Point, Point]] = []

    for y_cone in sorted_yellow:
        best_idx  = None
        best_dist = float("inf")

        for i, b_cone in enumerate(sorted_blue):
            if i in used_blue:
                continue
            dist = math.hypot(y_cone[0] - b_cone[0], y_cone[1] - b_cone[1])
            if dist < best_dist and dist <= cfg.MAX_PAIR_DISTANCE:
                best_dist = dist
                best_idx  = i

        if best_idx is not None:
            used_blue.add(best_idx)
            pairs.append((y_cone, sorted_blue[best_idx]))
        else:
            logger.debug(f"Cone amarelo {y_cone} sem par azul dentro de {cfg.MAX_PAIR_DISTANCE}m.")

    logger.info(f"Pares encontrados: {len(pairs)} de {len(sorted_yellow)} cones amarelos.")
    return pairs

def get_midpoints(pairs: list[tuple[Point, Point]]) -> list[Point]:
    
    return [
        ((y[0] + b[0]) / 2, (y[1] + b[1]) / 2)
        for y, b in pairs
    ]


def get_trajectory(midpoints: list[Point]) -> tuple[float, float]:
    
    
    if len(midpoints) < 2:
        logger.warning("Midpoints insuficientes para regressão linear.")
        return 0.0, 0.0

    xs = np.array([p[0] for p in midpoints])
    ys = np.array([p[1] for p in midpoints])

    var_x = np.var(xs)
    var_y = np.var(ys)

    if var_x < 1e-6 and var_y < 1e-6:
        logger.warning("Midpoints todos iguais.")
        return 0.0, float(xs[0])

    if var_y >= var_x:

        slope, intercept = np.polyfit(ys, xs, 1)
    else:
        
        slope, intercept = np.polyfit(xs, ys, 1)

    return float(slope), float(intercept)


def build_trajectory(
    yellow:  list[Point],
    blue:    list[Point],
    red:     list[Point],
) -> TrajectoryData:
    
    axis      = infer_advance_axis(yellow, blue)
    pairs     = match_cones(yellow, blue, axis)
    midpoints = get_midpoints(pairs)
    slope, intercept = get_trajectory(midpoints)

    all_advance = [get_advance_coord(p, axis) for p in yellow + blue]
    start_y  = min(all_advance) if all_advance else 0.0
    finish_y = (
        np.mean([get_advance_coord(p, axis) for p in red])
        if red else max(all_advance) if all_advance else 75.0
    )

    return TrajectoryData(
        midpoints=midpoints,
        slope=slope,
        intercept=intercept,
        finish_line=red,
        finish_y=finish_y,
        start_y=start_y,
        advance_axis=axis,
    )


def decide_phase(state: CarState, traj: TrajectoryData) -> CarState.Phase:
    
    p = state.phase

    if p == CarState.Phase.WAITING:
        return CarState.Phase.ACCELERATING

    if p == CarState.Phase.ACCELERATING:
        if state.distance_to_finish <= cfg.BRAKE_ZONE_DISTANCE:
            logger.info(f"BRAKING (faltam {state.distance_to_finish:.1f}m)")
            return CarState.Phase.BRAKING

    if p == CarState.Phase.BRAKING:
        if state.speed < 0.1:
            logger.info("FINISHED")
            return CarState.Phase.FINISHED

    return p

def make_initial_car_state(traj: TrajectoryData) -> CarState:
    """Cria o estado inicial do carro antes da largada."""
    axis = traj.advance_axis
    start = traj.start_y + cfg.CAR_START_OFFSET

    if axis == "y":
        pos = (0.0, start)
    else:
        pos = (start, 0.0)

    distance = abs(traj.finish_y - start)

    return CarState(
        phase=CarState.Phase.WAITING,
        position=pos,
        speed=0.0,
        distance_to_finish=distance,
    )


def step_simulation(state: CarState, traj: TrajectoryData, dt: float) -> CarState:
    
    new_phase = decide_phase(state, traj)
    speed     = state.speed
    pos       = state.position
    axis      = traj.advance_axis

    if new_phase == CarState.Phase.ACCELERATING:
        speed = min(speed + cfg.CAR_ACCEL * dt, cfg.CAR_MAX_SPEED)

    elif new_phase == CarState.Phase.BRAKING:
        speed = max(speed - cfg.CAR_DECEL * dt, 0.0)

    if axis == "y":
        new_pos = (pos[0], pos[1] + speed * dt)
    else:
        new_pos = (pos[0] + speed * dt, pos[1])

    advance = get_advance_coord(new_pos, axis)
    dist    = max(traj.finish_y - advance, 0.0)

    return CarState(
        phase=new_phase,
        position=new_pos,
        speed=speed,
        distance_to_finish=dist,
    )


def generate_simulation_frames(traj: TrajectoryData) -> list[CarState]:
    
    
    frames = []
    state  = make_initial_car_state(traj)
    state  = CarState(phase=decide_phase(state, traj), **{
        k: getattr(state, k) for k in ("position", "speed", "distance_to_finish")
    })

    max_steps = int(60 / cfg.SIMULATION_DT)  

    for _ in range(max_steps):
        frames.append(state)
        if state.phase == CarState.Phase.FINISHED:
            break
        state = step_simulation(state, traj, cfg.SIMULATION_DT)

    logger.info(f"Simulação gerada: {len(frames)} frames ({len(frames)*cfg.SIMULATION_DT:.1f}s)")
    return frames