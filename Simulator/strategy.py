"""
strategy.py  –  Game-state tracker (mirrors C++ Strategy singleton)
"""

from __future__ import annotations

from enum import IntEnum
from . import config as C


class Block(IntEnum):
    NONE = -1
    WOOD = 0
    STONE = 1
    IRON = 2
    DIAMOND = 3


class ToolLevel(IntEnum):
    WOOD_TOOL = 0
    STONE_TOOL = 1
    IRON_TOOL = 2
    DIAMOND_TOOL = 3


class CycleType(IntEnum):
    PICKAXE_HANDLE = 0
    PICKAXE_TIP = 1
    SWORD = 2
    SHIELD = 3


class IndexDirection(IntEnum):
    REJECT = -1
    LEFT = 0
    HOLD = 1
    RIGHT = 2
    CHEST = 3


class Strategy:
    """Singleton-like game strategy tracker from C++ Strategy class."""

    def __init__(self):
        self.pickaxe_level = ToolLevel.WOOD_TOOL
        self.sword_level = ToolLevel.WOOD_TOOL
        self.next_item = CycleType.PICKAXE_HANDLE
        self.possession: list[Block] = [Block.NONE, Block.NONE, Block.NONE]  # LEFT, HOLD, RIGHT

    def get_max_mineable_block(self) -> Block:
        return Block(C.MAX_BLOCK_MINEABLE[self.pickaxe_level])

    def hits_to_kill_silverfish(self) -> int:
        return C.SILVERFISH_HITS[self.sword_level]

    def hits_to_mine(self, block: Block) -> int:
        return C.MINING_CHART[block][self.pickaxe_level]

    def where_to_index_block(self, new_block: Block) -> IndexDirection:
        if self.next_item == CycleType.PICKAXE_HANDLE:
            if new_block == Block.WOOD:
                if self.possession[IndexDirection.LEFT] != Block.NONE:
                    return IndexDirection.HOLD
                return IndexDirection.LEFT
            return IndexDirection.REJECT

        if self.next_item == CycleType.PICKAXE_TIP:
            if new_block != self.get_max_mineable_block():
                return IndexDirection.CHEST
            if self.possession[IndexDirection.LEFT] == Block.NONE:
                return IndexDirection.LEFT
            if self.possession[IndexDirection.RIGHT] == Block.NONE:
                return IndexDirection.RIGHT
            return IndexDirection.HOLD

        if self.next_item == CycleType.SWORD:
            if new_block == Block.WOOD:
                if self.possession[IndexDirection.LEFT] == Block.NONE:
                    return IndexDirection.LEFT
                return IndexDirection.CHEST
            if self.possession[IndexDirection.RIGHT] == Block.NONE:
                return IndexDirection.RIGHT
            if new_block != self.possession[IndexDirection.RIGHT]:
                return IndexDirection.REJECT
            return IndexDirection.HOLD

        return IndexDirection.REJECT

    def ready_to_score(self) -> bool:
        if self.next_item == CycleType.PICKAXE_HANDLE:
            return (self.possession[IndexDirection.LEFT] == Block.WOOD
                    and self.possession[IndexDirection.HOLD] == Block.WOOD)
        if self.next_item == CycleType.PICKAXE_TIP:
            tip = self.get_max_mineable_block()
            return all(p == tip for p in self.possession)
        if self.next_item == CycleType.SWORD:
            return (self.possession[IndexDirection.LEFT] == Block.WOOD
                    and self.possession[IndexDirection.RIGHT] != Block.NONE
                    and self.possession[IndexDirection.HOLD] == self.possession[IndexDirection.RIGHT])
        return False

    def set_possession(self, idx: IndexDirection, block: Block):
        if idx == IndexDirection.REJECT:
            return
        if 0 <= idx <= 2:
            self.possession[idx] = block

    def advance_cycle(self):
        self.clear_all_possession()
        if self.next_item == CycleType.PICKAXE_HANDLE:
            self.next_item = CycleType.PICKAXE_TIP
        elif self.next_item == CycleType.PICKAXE_TIP and self.pickaxe_level != ToolLevel.DIAMOND_TOOL:
            self.pickaxe_level = ToolLevel(min(self.pickaxe_level + 1, ToolLevel.DIAMOND_TOOL))
            self.next_item = CycleType.PICKAXE_HANDLE
        else:
            self.next_item = CycleType.SWORD

    def clear_all_possession(self):
        self.possession = [Block.NONE, Block.NONE, Block.NONE]
