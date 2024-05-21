import numpy as np

from objects.Move import Move


class Robot:
    heading = 0.0
    _path = np.empty(100, dtype=np.dtype([
        ("end_time", np.uint32),
        ("start", np.int16, 2),
        ("end", np.int16, 2)
    ]))

    def __init__(self, name: str, address: int, radius: float) -> None:
        self.name = name
        self.address = address
        self.radius = radius

    @property
    def path(self) -> list[Move]:
        return [Move.from_dtype(move_dt) for move_dt in self._path]

    @path.setter
    def path(self, path: list[Move]) -> None:
        self._path = np.array(path, dtype=np.dtype([
            ("end_time", np.uint32),
            ("start", np.int16, 2),
            ("end", np.int16, 2)
        ]))

    def __getitem__(self, index) -> Move:
        return Move.from_dtype(self._path[index])

    def __setitem__(self, index, value: Move):
        value.to_array(self._path[index])

    def set_end_time(self, index: int, end_time: int) -> None:
        self._path[index]["end_time"] = end_time

    def set_start(self, index: int, start: tuple[int, int]) -> None:
        self._path[index]["start"] = start

    def set_end(self, index: int, end: tuple[int, int]) -> None:
        self._path[index]["end"] = end
