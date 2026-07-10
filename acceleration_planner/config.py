# --- Pista ---
BRAKE_ZONE_DISTANCE = 15.0   # metros antes da meta para a FSM mudar para BRAKING
MAX_PAIR_DISTANCE   = 8.0    # distância máxima (m) para considerar um par amarelo-azul válido

# --- Simulação do carro ---
CAR_START_OFFSET    = -3.0   # metros antes do primeiro cone (posição inicial no eixo de avanço)
CAR_MAX_SPEED       = 20.0   # m/s (velocidade máxima na fase de aceleração)
CAR_ACCEL           = 4.0    # m/s² (aceleração)
CAR_DECEL           = 8.0    # m/s² (travagem)
SIMULATION_DT       = 0.05   # segundos por frame da animação

# --- Visualização ---
SHOW_MIDPOINTS      = True
SHOW_TRAJECTORY     = True
CONE_MARKER_SIZE    = 9
MIDPOINT_MARKER_SIZE = 6
CAR_MARKER_SIZE     = 14
ANIMATION_INTERVAL  = 50     # ms entre frames (20 fps)
ASPECT_MODE         = "equal"

# --- Cores (matplotlib) ---
COLOR_YELLOW_CONE   = "#f5c400"
COLOR_BLUE_CONE     = "#1565c0"
COLOR_RED_CONE      = "#c62828"
COLOR_MIDPOINT      = "#e91e63"
COLOR_TRAJECTORY    = "#2e7d32"
COLOR_CAR           = "#ff6f00"
COLOR_CAR_BRAKING   = "#c62828"
COLOR_CAR_FINISHED  = "#555555"
