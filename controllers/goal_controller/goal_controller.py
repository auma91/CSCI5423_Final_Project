"""goal_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Camera, CameraRecognitionObject, Keyboard
import sys, math
import numpy as np

# create the Robot instance.
robot = Robot()

MAX_SPEED = 6.28

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

compass = robot.getDevice("compass")
compass.enable(timestep)

gps = robot.getDevice("gps")
gps.enable(timestep)

camera = Camera("camera")
camera.enable(timestep)
camera.recognitionEnable(timestep)

keyboard = robot.getKeyboard()
keyboard.enable(timestep)

leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')

leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftSpeed, rightSpeed = 0, 0
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)

gpsCoordinates = []

Trial = "9"

logging = False
if len(sys.argv)> 1 and sys.argv[2] == "True":
    logging = True

state_machine = {"locateBall": False, "offsetBall": False}

deltaTheta = 0

counter = 0

currentTime1, currentTime2 = 0,0

# n = compass.getValues()
# rad = -math.atan2(n[0], n[1])
# pose_theta = rad
#
# deltaTheta = 0

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    n = compass.getValues()
    rad = -math.atan2(n[0], n[1])
    pose_theta = rad
    if not state_machine["locateBall"]:
        leftSpeed = -0.1 * MAX_SPEED
        rightSpeed = 0.1 * MAX_SPEED
        cameraRecogObjects = camera.getRecognitionObjects()
        for obj_check in cameraRecogObjects:
            color_check = obj_check.get_colors()
            objectPostion = obj_check.get_position()
            pos_im = obj_check.get_position_on_image()
            if color_check == [1, 0.54, 0.08]:
                if abs(pos_im[0] - camera.getWidth()/2) <= 2:
                    leftSpeed = rightSpeed = 0
                    state_machine["locateBall"] = True
                    print("Angle: {}".format(pose_theta))
    elif not state_machine["offsetBall"]:
        if abs(pose_theta + 2.669) <= 0.02:
            leftSpeed = rightSpeed = 0.1 * MAX_SPEED
        if pose_theta > -2.669:
            leftSpeed = -0.1 * MAX_SPEED
            rightSpeed = 0.1 * MAX_SPEED
        elif pose_theta <= -2.699:
            leftSpeed = 0.1 * MAX_SPEED
            rightSpeed = -0.1 * MAX_SPEED


    if np.random.rand() < 0.25:
        gpsCoordinates.append(gps.getValues()[0:2])
    if keyboard.getKey() == ord('S'):
        np.save("./output/goal_" + sys.argv[1] + "_" + Trial + ".npy", gpsCoordinates)
        print("Map file saved: {}.npy".format("goal_" + sys.argv[1] + "_" + Trial))


    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)

# Enter here exit cleanup code.
