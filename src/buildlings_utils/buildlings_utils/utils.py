from enum import IntEnum

class BlockState(IntEnum):
    ABYSS = 0
    GROUND = 1
    AGENT_BRIDGE = 2
    OCCUPIED = 3
    GOAL = 4