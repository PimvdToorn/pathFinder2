import time


class Timer:
    def __init__(self):
        self.start = time.time_ns()
        self.stop_time = 0

    @property
    def stopped(self):
        return self.stop_time != 0

    def reset(self):
        self.start = time.time_ns()
        self.stop_time = 0

    def stop(self) -> int:
        self.stop_time = time.time_ns()
        return self.stop_time - self.start

    # Nanoseconds
    def ns(self) -> int:
        if self.stopped:
            return self.stop_time - self.start
        return time.time_ns() - self.start

    def seconds(self) -> float:
        return self.ns() / 1_000_000_000
