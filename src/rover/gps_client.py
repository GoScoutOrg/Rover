from gps import gps, WATCH_ENABLE
from queue import Queue
from threading import Thread


class GpsClient(Thread):
    """
    Creates a thread that continuously updates GPS readings. Access fields as if this
    object were a dictionary.

    Example:

    ```
    gps = GpsClient()
    gps.start()

    lat = gps["latitude"]
    lon = gps["latitude"]
    alt = gps["altitude"]

    # And so on...
    ```

    See https://gitlab.com/gpsd/gpsd.
    """

    _gps: gps
    _queue: Queue[bool]

    def __init__(self):
        self._gps = gps(mode=WATCH_ENABLE)
        self._queue = Queue()
        Thread.__init__(self)

    def run(self):
        while self._queue.empty():
            self._gps.next()

    def stop(self):
        self._queue.put(True)

    def __getitem__(self, key):
        return getattr(self._gps.fix, key)
