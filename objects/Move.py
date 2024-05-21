import numpy as np


class Move:
    def __init__(self, end_time: int, start: tuple[int, int], end: tuple[int, int]):
        self.end_time = end_time
        self.start = start
        self.end = end

    @classmethod
    def from_dtype(cls, move_dt):
        return Move(
            move_dt["end_time"],
            tuple[int, int](move_dt["start"]),
            tuple[int, int](move_dt["end"])
        )

    def __repr__(self) -> str:
        return f"Move({self.end_time}, {self.start}, {self.end})"

    def to_array(self, array_item) -> None:
        array_item["end_time"] = self.end_time
        array_item["start"] = self.start
        array_item["end"] = self.end
