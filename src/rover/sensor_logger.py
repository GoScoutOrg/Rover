from datetime import datetime, timedelta
from rover.gps_client import GpsClient
from rover.imu import Imu
from io import TextIOWrapper
from queue import Queue
from threading import Thread
from time import sleep, mktime
from typing import Callable, Optional

from rover.commands import get_encoders, get_turning

DEFAULT_LOG_FILE = "sensor-data.csv"
DEFAULT_TIMESTEP = timedelta(seconds=2)
DEFAULT_TIMESTEP_MARGIN = timedelta(milliseconds=10)


def min_time(time: timedelta, margin: timedelta) -> Callable:
    def decorator(fn: Callable):
        def wrapper(*args, **kwargs):
            t = datetime.now()
            fn(*args, **kwargs)
            delta = time - (datetime.now() - t)
            if delta > margin:
                sleep(delta.seconds)

        return wrapper

    return decorator


class SensorLogger(Thread):
    def __init__(
        self, path: Optional[str] = DEFAULT_LOG_FILE, clear: Optional[bool] = True
    ):
        self.gpsc = GpsClient()
        self.gpsc.start()
        self.imu = Imu()
        self.path = path
        self.clear = clear
        self._queue = Queue()
        Thread.__init__(self)

    @min_time(DEFAULT_TIMESTEP, DEFAULT_TIMESTEP_MARGIN)
    def _log_timestep(self, time: datetime, file: TextIOWrapper):
        file.write(
            ",".join(
                [
                    str(e)
                    for e in [
                        int(mktime(datetime.now().timetuple())),
                        ",".join([str(v) for _, v in get_encoders()]),
                        self.gpsc["latitude"],
                        self.gpsc["longitude"],
                        self.gpsc["altitude"],
                        ",".join([str(e) for e in self.imu.accelerometer()]),
                        ",".join([str(e) for e in self.imu.magnetometer()]),
                        ",".join([str(e) for e in self.imu.gyroscope()]),
                        str(get_turning()),
                    ]
                ]
            )
            + "\n"
        )

    def run(self):
        if self.clear:
            with open(self.path, "w") as file:
                file.write(
                    ",".join(
                        [
                            "time",
                            ",".join([str(n) for n, _ in get_encoders()]),
                            "lat",
                            "long",
                            "alt",
                            "accl_x",
                            "accl_y",
                            "accl_z",
                            "mag_x",
                            "mag_y",
                            "mag_z",
                            "gyro_x",
                            "gyro_y",
                            "gyro_z",
                            "turning",
                        ]
                    )
                    + "\n"
                )
        with open(self.path, "a") as file:
            while self._queue.empty():
                self._log_timestep(datetime.now(), file)

    def stop(self):
        self._queue.put(True)
        self.gpsc.stop()
