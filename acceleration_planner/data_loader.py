import csv
import logging
from collections import namedtuple

logger = logging.getLogger(__name__)

ConeData = namedtuple("ConeData", ["yellow", "blue", "red"])


def _parse_point(row: dict) -> tuple[float, float] | None:

    try:
        return (float(row["x"]), float(row["y"]))
    except (KeyError, ValueError) as e:
        logger.warning(f"Linha inválida ignorada: {row} — {e}")
        return None


class CSVLoader:
    
    VALID_COLORS = {"yellow", "blue", "red"}

    def __init__(self, filepath: str):
        self.filepath = filepath

    def get_cones(self) -> ConeData:
        yellow, blue, red = [], [], []

        try:
            with open(self.filepath, newline="") as f:
                for row in csv.DictReader(f):
                    point = _parse_point(row)
                    if point is None:
                        continue

                    color = row.get("color", "").strip().lower()
                    if color not in self.VALID_COLORS:
                        logger.warning(f"Cor desconhecida ignorada: '{color}' em {row}")
                        continue

                    if color == "yellow":
                        yellow.append(point)
                    elif color == "blue":
                        blue.append(point)
                    elif color == "red":
                        red.append(point)

        except FileNotFoundError:
            logger.error(f"Ficheiro não encontrado: {self.filepath}")

        logger.info(f"Cones carregados — amarelos: {len(yellow)}, azuis: {len(blue)}, vermelhos: {len(red)}")
        return ConeData(yellow=yellow, blue=blue, red=red)


class StreamLoader:
    
    def __init__(self, host: str = "localhost", port: int = 5000):
        self.host = host
        self.port = port
        logger.warning("StreamLoader ainda não implementado — a devolver cones vazios.")

    def get_cones(self) -> ConeData:
       
        return ConeData(yellow=[], blue=[], red=[])
