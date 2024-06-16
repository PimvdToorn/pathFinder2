import time


class Timer:
    def __init__(self):
        self.start = time.time_ns()
        self.stop_time = 0

    def reset(self):
        self.start = time.time_ns()
        self.stop_time = 0

    def stop(self) -> int:
        self.stop_time = time.time_ns()
        return self.stop_time - self.start

    def elapsed(self) -> int:
        if self.stop_time:
            return self.stop_time - self.start
        return time.time_ns() - self.start

    def seconds(self) -> float:
        return self.elapsed() / 1_000_000_000
