from rplidar import RPLidar
from Vector2D import Vector2D
from PolarCoordinate import PolarCoordinate
import logging
import logging.handlers
import os

class Lidar:
    def __init__(self, port, minimumDistance=100, minimumQuality=5):
        self.handler = logging.handlers.WatchedFileHandler(
        os.environ.get("LOGFILE", "log/lidar.log"))
        self.formatter = logging.Formatter(logging.BASIC_FORMAT)
        self.handler.setFormatter(self.formatter)
        self.lidarLogger = logging.getLogger("LIDAR")
        self.lidarLogger.setLevel(os.environ.get("LOGLEVEL", "DEBUG"))
        self.lidarLogger.addHandler(self.handler)

        self.lidar = RPLidar(port, baudrate=115200, logger=self.lidarLogger)
        self.minimumDistance = minimumDistance
        self.minimumQuality  = minimumQuality

    def Start(self):
        self.lidar.connect()

    def Shutdown(self):
        self.lidar.stop()
        self.lidar.stop_motor()
        self.lidar.disconnect()

    def GetInformation(self):
        return self.lidar.get_info()

    def GetHealth(self):
        return self.lidar.get_health()

    def GetLidar(self):
        return self.lidar
'''
if __name__== "__main__":
    lidar = Lidar('COM36', minimumDistance=50, minimumQuality=5)
    lidar.Start()

    try:
        while True:
            print("Fetching newest scan...")
            lidar.GetScan()
    except Exception as ex:
        print(ex)
        lidar.Shutdown()
'''
