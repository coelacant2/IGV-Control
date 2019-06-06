from MotionHandler import Motion
from time import sleep, time
from Vector2D import Vector2D
from threading import Thread
from ThreadHandler import StoppableThread
from queue import Queue
from serial import Serial
from Lidar import Lidar
import dynamic_window_approach
import numpy as np
import matplotlib.pyplot as plt
import re
import math

motion = Motion('COM34', Vector2D(0.0, 0.0), 0.25, 0.517525, 0.1, 2.0)#meters: 1.678mph -> 4.474mph
mpuComPort = 'COM20'
gpsComPort = 'COM21'
orientation = 0.0
position    = [0, 0]
points      = []
mapRange    = 1.0

plt.ion()
fig, ax = plt.subplots()
sc = ax.scatter([], [], marker='x')
plt.xlim(-3,3)
plt.ylim(-3,3)
plt.show()

def calculate(q, skipCompensation):
    global orientation

    if not q.empty():
        orientation = q.get()

    while motion.calculateStep(orientation, skipCompensation):
        displayMap()
        if not q.empty():
            orientation = q.get()

        sleep(0.001)

def readMPU(self, name, q):
    global orientation

    while True:
        mpu = Serial(mpuComPort, 115200)

        lastReadTime = time()

        while not self.stopped():
            try:
                if mpu.in_waiting:
                    text = mpu.readline()[5:-2].decode('utf-8')
                    lastReadTime = time()

                    #print(text.split(',')[0])

                    #q.put(float(text.split(',')[0]))
                    orientation = float(text.split(',')[0])
                else:
                    if time() - lastReadTime > 2.0:
                        print("Reseting MPU Arduino...")
                        break;
            except Exception as ex:
                print("Read failed: " + str(ex))

            sleep(0.005)

        mpu.close()

        if self.stopped():
            break

    print("MPU Thread Execution Terminated.")
    return None

def readGPS(self, name, q):
    while True:
        gps = Serial(gpsComPort, 115200)

        lastReadTime = time()

        while not self.stopped():
            try:
                if gps.in_waiting:
                    text = gps.readline().decode('utf-8')
                    lastReadTime = time()

                    #### DELIMIT GPS FEED
                    #q.put(float(text.split(',')[0]))
                else:
                    if time() - lastReadTime > 2.0:
                        print("Reseting GPS Module...")
                        break;
            except Exception as ex:
                print("Read failed: " + str(ex))

            sleep(0.005)

        gps.close()

    print("GPS Thread Execution Terminated.")
    return None

def updateLidarMap(self, name, q):
    print("Starting Map Thread")
    global orientation
    global points

    while not self.stopped():
        if not q.empty():
            value = q.get()

            vec = Vector2D.fromPolarCoordinate(value[3] / 1000.0, value[2] + orientation - 90)#+ Vector2D(position[0], position[1])

            constrain = 5.0

            xV = int(vec.X * constrain)
            yV = int(-vec.Y * constrain)

            exists = False
            for p in points:
                if int(p[0] * constrain) == xV and int(p[1] * constrain) == yV:
                    exists = True
                    break

            if not exists:
                points.append([xV / constrain, yV / constrain, 0])

            clearOldPoints()
        else:
            sleep(0.005)

    print("Lidar Map Thread Execution Terminated.")
    return None

def clearOldPoints(age=200):
    global points

    tPoints = []
    for p in points:
        if p[2] < age:
            tPoints.append([p[0], p[1], p[2] + 1])

    points = tPoints

def displayMap():
    global points

    if len(points) > 1:
        x, y, a = list(zip(*points))

        sc.set_offsets(np.c_[x,y])
        ax.set_aspect('equal')
        ax.relim()
        ax.autoscale_view()
        fig.canvas.flush_events()

def readLidar(self, name, q):
    while not self.stopped():
        lidar = Lidar('COM36', minimumDistance=100, minimumQuality=1)
        lidar.Start()

        lidObj = lidar.GetLidar()

        try:
            for new, quality, angle, distance in lidObj.iter_measurments(max_buf_meas=500):
                if self.stopped():
                    print("Shutting down lidar...")
                    lidar.Shutdown()
                    break
                if quality > 5 and distance > 200 and distance < 4000:
                    q.put((new, quality, angle, distance))
        except Exception as ex:
            print(ex)
            lidar.Shutdown()


    print("Lidar Read Thread Execution Terminated.")
    return None

def path1(q):
    #spin
    motion.rotate(True,50,45,0, 0, 15)
    calculate(q)
    #move 45 degree
    motion.move(True,  25, 100, 15, 15)
    calculate(q)
    #S arc
    motion.rotate(True,25,90,-50, 15, 15)
    calculate(q)
    motion.rotate(True,50,45,100, 15, 25)
    calculate(q)

    motion.move(True,  25, 100, 25, 25)
    calculate(q)
    motion.rotate(True,50,135,-10, 25, 25)
    calculate(q)

    #double u turn
    motion.move(True,  25, 10, 25, 25)
    calculate(q)
    motion.rotate(True,50,180,5, 25, 25)
    calculate(q)
    motion.move(True,  25, 10, 25, 25)
    calculate(q)
    motion.rotate(True,50,180,-5, 25, 25)
    calculate(q)

    #double u turn
    motion.move(True,  25, 10, 25, 25)
    calculate(q)
    motion.rotate(True,50,180,5, 25, 25)
    calculate(q)
    motion.move(True,  25, 10, 25, 25)
    calculate(q)
    motion.rotate(True,50,180,-5, 25, 25)
    calculate(q)

    motion.move(True,  25, 10, 25, 25)
    calculate(q)
    motion.rotate(True, 50, 45, -25, 25, 50)
    calculate(q)

    motion.move(True, 100, 315, 50, 50)
    calculate(q)
    motion.rotate(True, 50, 135, -10, 50, 0)
    calculate(q)

def squarePath(q):
    motion.rotate(True, 1, 360, 0, 0.1, 0.1)
    calculate(q, True)

    squareSize = 1.0
    #turn 45 move out 0.8 meter
    motion.rotate(True, 0.5, 90, 0, 0.1, 0.25)
    calculate(q, False)
    motion.move(True, 0.5, 0.4 * squareSize, 0.25, 0.25)
    calculate(q, False)

    motion.rotate(True, 0.5, 90, 0.1 * squareSize, 0.25, 0.25)#short edge
    calculate(q, False)
    motion.move(True, 1, 0.3 * squareSize, 0.25, 0.25)
    calculate(q, False)

    motion.rotate(True, 0.5, 90, 0.1 * squareSize, 0.25, 0.25)#first long edge
    calculate(q, False)
    motion.move(True, 1, 0.8 * squareSize, 0.25, 0.25)
    calculate(q, False)

    motion.rotate(True, 0.5, 90, 0.1 * squareSize, 0.25, 0.25)#second long edge
    calculate(q, False)
    motion.move(True, 1, 0.8 * squareSize, 0.25, 0.25)
    calculate(q, False)
    motion.rotate(True, 0.5, 90, 0.1 * squareSize, 0.25, 0.25)#third long edge
    calculate(q, False)
    motion.move(True, 1, 0.8 * squareSize, 0.25, 0.25)
    calculate(q, False)

    motion.rotate(True, 0.5, 90, 0.1 * squareSize, 0.25, 0.25)#last short edge
    calculate(q, False)
    motion.move(True, 1, 0.3 * squareSize, 0.25, 0.25)
    calculate(q, False)

    motion.rotate(True, 0.5, 90, 0.1 * squareSize, 0.25, 0.25)#back to start, spin 180
    calculate(q, False)
    motion.move(True, 1, 0.4 * squareSize, 0.25, 0.25)
    calculate(q, False)

    motion.rotate(True, 0.5, 180, 0, 0.25, 0.1)
    calculate(q, False)

def line(q):
    motion.rotate(True, 0.5, 720, 0, 0.1, 0.1)
    calculate(q, True)

    motion.rotate(True, 1, 20, 0, 0.1, 0.1)
    calculate(q, False)

    motion.move(True, 1, 10, 0.1, 0.1)
    calculate(q, False)

    motion.move(True, 1, -10, 0.1, 0.1)
    calculate(q, False)

def spin(q):
    motion.rotate(True, 0.5, 720, 0, 0.1, 0.1)
    calculate(q, True)

def reducePoints(points, range):
    reducedPoints = []

    for p in points:
        if p[0] < position[0] + range and p[0] > position[0] - range and p[1] < position[1] + range and p[1] > position[1] - range:
            reducedPoints.append(p)

    return reducedPoints

def handleMotion():
    global points
    global position
    global orientation
    #path1(q)
    #line(q)
    #squarePath(q)
    goal = [3, 2]
    position = [0.0, 0.0]

    # initial state [x(m), y(m), yaw(rad), v(m/s), omega(rad/s)]
    x = np.array([position[0], position[1], (orientation + 90.0) * math.pi / 180.0, 0.0, 0.0])
    previousfsYaw = [0.0, 0.0]
    while True:
        dist_to_goal = math.sqrt((x[0] - goal[0])**2 + (x[1] - goal[1])**2)
        if dist_to_goal <= 0.25:
            break

        if len(points) > 1:
            '''
            for i in range(12):
                reducedPoints = reducePoints(points, mapRange + i / 0.25)

                if len(reducedPoints) > 6:
                    break
            '''

            fsYaw = dynamic_window_approach.getSpeedAndYaw(previousfsYaw, x, goal, points[::4])

            # REMOVE, modify X position with MPU yaw, GPS position, and motor speed
            x = dynamic_window_approach.motion(x, fsYaw, 0.1) # simulate robot
            position = [x[0], x[1]]
            #x = np.array([position[0], position[1], (orientation + 90.0) * math.pi / 180.0, 0.0, 0.0]) # actual robot
            #if fsYaw[0] < 0.25:
            #    fsYaw[0] = 0.25

            if fsYaw[0] + fsYaw[1] > fsYaw[0] - fsYaw[1]:
                right = motion.constrain(fsYaw[0] + fsYaw[1], 0.0, 1.0)
                offset = fsYaw[0] + fsYaw[1] - right

                left = motion.constrain(fsYaw[0] - fsYaw[1] - offset, 0.0, 1.0)
            else:
                left = motion.constrain(fsYaw[0] - fsYaw[1], 0.0, 1.0)
                offset = fsYaw[0] + fsYaw[1] - left

                right = motion.constrain(fsYaw[0] + fsYaw[1] - offset, 0.0, 1.0)

            motion.setSpeedsRaw(left * 30.0, right * 30.0)
            previousfsYaw = fsYaw

    #return None

if __name__ == "__main__":
    mpuQueue = Queue()
    gpsQueue = Queue()
    lidarQueue = Queue()

    mpuThread       = StoppableThread(target=readMPU,        args=("MPUThread",       mpuQueue))
    gpsThread       = StoppableThread(target=readGPS,        args=("GPSThread",       gpsQueue))
    lidarMapThread  = StoppableThread(target=updateLidarMap, args=("MapLIDARThread",  lidarQueue))
    lidarReadThread = StoppableThread(target=readLidar,      args=("ReadLIDARThread", lidarQueue))

    mpuThread.start()
    gpsThread.start()
    lidarMapThread.start()
    lidarReadThread.start()

    try:
        spin(mpuQueue)
        points = []
        sleep(2)
        handleMotion()
        #while True:
        #    displayMap()
        #motion.savePlot('Test1')
    except KeyboardInterrupt:
        print("Shutting down")
    except Exception as ex:
        print(ex)

    motion.shutdown()

    plt.close('all')
    mpuThread.stop()
    gpsThread.stop()
    lidarMapThread.stop()
    lidarReadThread.stop()

    mpuThread.join()
    gpsThread.join()
    lidarMapThread.join()
    lidarReadThread.join()
