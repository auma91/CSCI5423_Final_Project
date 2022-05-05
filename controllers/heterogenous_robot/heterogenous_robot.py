"""heterogenous_robot controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Camera, CameraRecognitionObject, Compass, InertialUnit, GPS, Keyboard
import math, sys
import numpy as np
# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

MAX_SPEED = 6.28

leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')

compass = robot.getDevice("compass")
compass.enable(timestep)

gps = robot.getDevice("gps")
gps.enable(timestep)

camera = Camera("camera")
camera.enable(timestep)
camera.recognitionEnable(timestep)

keyboard = robot.getKeyboard()
keyboard.enable(timestep)

leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftSpeed, rightSpeed = 0, 0

logging = False
if len(sys.argv)> 1 and sys.argv[2] == "True":
    logging = True

state_machine = {"initial360" : [False, None, None, None, None], "alignmentphase" : False, "aligned" : False, "isNearObject" : False, "searchingGoal" : [False, 0], "goalFindable" : [False, None],
                 "foundGoal" : [False, None, None, None]}

deltaTheta = 0

counter = 0

currentTime1, currentTime2 = 0,0

n = compass.getValues()
rad = -math.atan2(n[0], n[1])
pose_theta = rad

deltaTheta = 0

gpsCoordinates = []

Trial = "9"

def waitingTime(angularDistance):
    return (-(8 * angularDistance)/(0.5 * math.pi)) + 8

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    n = compass.getValues()
    rad = -math.atan2(n[0], n[1])
    pose_theta = rad
    if not state_machine["initial360"][0]:
        if deltaTheta == 0:
            previousTheta = pose_theta
            deltaTheta += 0.00000001
        else:
            deltaTheta += abs(abs(previousTheta) - abs(pose_theta))
            previousTheta = pose_theta

        if deltaTheta <= 2 * math.pi:
            leftSpeed = -0.2 * MAX_SPEED
            rightSpeed = 0.2 * MAX_SPEED
            cameraRecogObjects = camera.getRecognitionObjects()
            # list of all objects detected
            count = 0
            greenVec = []
            ballInView = (False, None, None, None)
            for obj_check in cameraRecogObjects:
                color_check = obj_check.get_colors()
                objectPostion = obj_check.get_position()
                pos_im = obj_check.get_position_on_image()
                if color_check == [1, 0.54, 0.08]:
                    ballInView = (True, pose_theta, objectPostion, pos_im)
            for obj in cameraRecogObjects:
                # pos = i.get_position()
                # x,y,z
                colors = obj.get_colors()
                # [R,G,B]
                # orange ball: [1.0, 0.54, 0.08]
                objectPostion = obj.get_position()
                pos_im = obj.get_position_on_image()

                if colors == [1, 0.54, 0.08]:
                    if state_machine["initial360"][1] is None:
                        if logging:
                            print("found Orange ball")
                        if abs(pos_im[0] - camera.getWidth()/2) <= 1:
                            state_machine["initial360"][1] = (pose_theta, objectPostion, pos_im)
                            if logging:
                                print("Orange ball in alignement: {}".format(state_machine["initial360"][1]))
                        # print("[{} == {}?={}, {} == {}?={}, and {} == {}?={}]".format(colors[0], 1, colors[0] == 1, colors[1],
                        #                                                               0.54, colors[1] == 0.54, colors[2], 0.08,
                        #                                                               colors[2] == 0.08))
                if colors == [0.192, 1, 0.133]:
                    #green:
                    count += 1
                    greenVec.append((pose_theta, objectPostion, pos_im))
                    if count == 2:
                        left = greenVec[0] if greenVec[0][2][0] < greenVec[1][2][0] else greenVec[1]
                        right = greenVec[0] if greenVec[0][2][0] > greenVec[1][2][0] else greenVec[1]
                        if logging:
                            print("Both goals in view: {}".format((left, right)))
                        if right[2][0] > camera.getWidth()/2:
                            if logging:
                                print("Partially aligned")
                            left_diff = abs(left[2][0] - camera.getWidth()/2)
                            right_diff = abs(right[2][0] - camera.getWidth() / 2)
                            if logging:
                                print("left diff: {}, right diff: {}, center diff: {}".format(left_diff, right_diff, abs(left_diff - right_diff)))
                            if abs(left_diff - right_diff) <= 2:
                                if logging:
                                    print("Aligned 2 goals")
                                state_machine["initial360"][2] = left
                                state_machine["initial360"][3] = right
                    elif ballInView[0] and count==1:
                        left = greenVec[0] if greenVec[0][2][0] < ballInView[3][0] else (ballInView[1], ballInView[2], ballInView[3])
                        right = greenVec[0] if greenVec[0][2][0] > ballInView[3][0] else (ballInView[1], ballInView[2], ballInView[3])
                        if logging:
                            print("1 goals in view, with occluded goal (by object): {}".format((left, right)))
                        if right[2][0] > camera.getWidth()/2:
                            if logging:
                                print("Partially aligned")
                            left_diff = abs(left[2][0] - camera.getWidth()/2)
                            right_diff = abs(right[2][0] - camera.getWidth() / 2)
                            if logging:
                                print("left diff: {}, right diff: {}, center diff: {}".format(left_diff, right_diff, abs(left_diff - right_diff)))
                            if abs(left_diff - right_diff) <= 2:
                                if logging:
                                    print("Aligned 2 goals, with using object as goal estiamtion (occluded)")
                                state_machine["initial360"][2] = left
                                state_machine["initial360"][3] = right
                    else:
                        if logging:
                            print("# of goals found: {}, {}".format(len(greenVec), greenVec))
        else:
            leftSpeed = 0
            rightSpeed = 0
            state_machine["initial360"][0] = True
    elif not state_machine["alignmentphase"]:
        if state_machine["initial360"][1][0] < 0:
            if logging:
                print("Rotate counterclockwise")
                print("current pose: {}, object pose: {}".format(pose_theta, state_machine["initial360"][1][0]))
            if abs(pose_theta - state_machine["initial360"][1][0]) < 0.02:
                leftSpeed, rightSpeed = 0,0
                state_machine["alignmentphase"] = True
            elif abs(pose_theta - state_machine["initial360"][1][0]) < math.pi/6:
                leftSpeed = -0.1 * MAX_SPEED
                rightSpeed = 0.1 * MAX_SPEED
            elif pose_theta < state_machine["initial360"][1][0] or pose_theta > 0.1:
                leftSpeed = 0.1 * MAX_SPEED
                rightSpeed = -0.1 * MAX_SPEED
            elif pose_theta > state_machine["initial360"][1][0]:
                leftSpeed = -0.2 * MAX_SPEED
                rightSpeed = 0.2 * MAX_SPEED

        else:
            if logging:
                print("Rotate Clockwise")
                print("current pose: {}, object pose: {}".format(pose_theta, state_machine["initial360"][1][0]))
            if abs(pose_theta - state_machine["initial360"][1][0]) < 0.015:
                leftSpeed, rightSpeed = 0, 0
                state_machine["alignmentphase"] = True
            elif pose_theta > state_machine["initial360"][1][0]:
                leftSpeed = -0.1 * MAX_SPEED
                rightSpeed = 0.1 * MAX_SPEED
            elif pose_theta < state_machine["initial360"][1][0]:
                leftSpeed = 0.2 * MAX_SPEED
                rightSpeed = -0.2 * MAX_SPEED
    elif not state_machine["aligned"]:
        cameraRecogObjects = camera.getRecognitionObjects()
        for obj in cameraRecogObjects:
            colors = obj.get_colors()
            if colors == [1, 0.54, 0.08]:
                pos_im = obj.get_position_on_image()
                if math.isclose(pos_im[0], camera.getWidth()/2):
                    state_machine["aligned"] = True
                    if logging:
                        print("aligned epuck")
                    leftSpeed = 0
                    rightSpeed = 0
                elif (camera.getWidth() / 2) < pos_im[0] < ((camera.getWidth() / 2) + (camera.getWidth() / 16)):
                    if logging:
                        print("Epuck to the right of center")
                    leftSpeed = 0.05 * MAX_SPEED
                    rightSpeed = -0.05 * MAX_SPEED
                elif ((camera.getWidth() / 2) - (camera.getWidth() / 16)) < pos_im[0] < (camera.getWidth() / 2):
                    if logging:
                        print("Epuck to the left of center")
                    leftSpeed = -0.05 * MAX_SPEED
                    rightSpeed = 0.05 * MAX_SPEED
                elif pos_im[0] > ((camera.getWidth() / 2) + (camera.getWidth() / 16)):
                    leftSpeed = 0.25 * MAX_SPEED
                    rightSpeed = -0.25 * MAX_SPEED
                elif pos_im[0] < ((camera.getWidth() / 2) - (camera.getWidth() / 16)):
                    leftSpeed = -0.25 * MAX_SPEED
                    rightSpeed = 0.25 * MAX_SPEED
    elif not state_machine["isNearObject"]:
        leftSpeed = 0.35 * MAX_SPEED
        rightSpeed = 0.35 * MAX_SPEED
        cameraRecogObjects = camera.getRecognitionObjects()
        for obj in cameraRecogObjects:
            colors = obj.get_colors()
            if colors == [1, 0.54, 0.08]:
                objectPostion = obj.get_position()
                if objectPostion[0] <= 0.08:
                    state_machine["isNearObject"] = True
                    leftSpeed = 0
                    rightSpeed = 0
                    state_machine["searchingGoal"][0] = True
                    angularDistance = 0
                    if np.sign(pose_theta) == np.sign(state_machine["initial360"][2][0]):
                        angularDistance = abs(pose_theta - state_machine["initial360"][2][0])
                        # state_machine["searchingGoal"] =
                    else:
                        if pose_theta < 0:
                            angularDistance = min(-pose_theta + state_machine["initial360"][2][0],
                                                  (math.pi + pose_theta) + (math.pi - state_machine["initial360"][2][0]))
                        else:
                            angularDistance = min((math.pi - pose_theta) + (math.pi + state_machine["initial360"][2][0]),
                                                  -state_machine["initial360"][2][0] + pose_theta)
                    state_machine["searchingGoal"][1] = waitingTime(angularDistance)
                    deltaTheta = 0
                    if logging:
                        print("epuck is at ball")
                # print("Object position: {}".format(objectPostion))
    elif state_machine["searchingGoal"][0]:
        # Green RGB
        # 0.192 1 0.133

        if logging:
            print("current angle: {}, goal angle: {}".format(pose_theta, state_machine['initial360'][2][0]))
        if abs(pose_theta - state_machine["initial360"][2][0]) > 0.02:
            currentTime1 = None
            if np.sign(pose_theta) == np.sign(state_machine["initial360"][2][0]):
                # within same hemisphere
                if pose_theta > state_machine["initial360"][2][0]:
                    leftSpeed = -0.1 * MAX_SPEED
                    rightSpeed = 0.1 * MAX_SPEED
                else:
                    leftSpeed = 0.1 * MAX_SPEED
                    rightSpeed = -0.1 * MAX_SPEED

            else:
                if pose_theta < 0:
                    if -pose_theta + state_machine["initial360"][2][0] < (math.pi + pose_theta) + (math.pi - state_machine["initial360"][2][0]):
                        leftSpeed = 0.1 * MAX_SPEED
                        rightSpeed = -0.1 * MAX_SPEED
                    else:
                        leftSpeed = -0.1 * MAX_SPEED
                        rightSpeed = 0.1 * MAX_SPEED
                else:
                    if (math.pi - pose_theta) + (math.pi + state_machine["initial360"][2][0]) < -state_machine["initial360"][2][0] + pose_theta:
                        leftSpeed = 0.1 * MAX_SPEED
                        rightSpeed = -0.1 * MAX_SPEED
                    else:
                        leftSpeed = -0.1 * MAX_SPEED
                        rightSpeed = 0.1 * MAX_SPEED
        else:
            leftSpeed = rightSpeed = 0
            if currentTime1 == None:
                currentTime1 = robot.getTime()
            else:
                if logging:
                    print("Time elapsed: {}, need to wait: {}".format(robot.getTime() - currentTime1, state_machine["searchingGoal"][1]))
                if robot.getTime() - currentTime1 > state_machine["searchingGoal"][1]:
                    leftSpeed = rightSpeed = 0.1 * MAX_SPEED

        # else:
        #     if deltaTheta == 0:
        #         previousTheta = pose_theta
        #         deltaTheta += 0.0001
        #     else:
        #         deltaTheta += abs(abs(previousTheta) - abs(pose_theta))
        #         previousTheta = pose_theta
        #     if logging:
        #         print("Current angle: {}, current amount of rotation: {}".format(pose_theta, deltaTheta))
        #     if deltaTheta >= 2 * math.pi:
        #         leftSpeed = 0
        #         rightSpeed = 0
        #         state_machine["searchingGoal"] = False
        #         currentTime1 = robot.getTime()
        #         if logging:
        #             print("Goal findable: {}".format(state_machine["goalFindable"][0]))
        #
        #     else:
        #         leftSpeed = -0.05 * MAX_SPEED
        #         rightSpeed = 0.05 * MAX_SPEED
        #         cameraRecogObjects = camera.getRecognitionObjects()
        #         # list of all objects detected
        #         for obj in cameraRecogObjects:
        #             # pos = i.get_position()
        #             # x,y,z
        #             colors = obj.get_colors()
        #             pos_im = obj.get_position_on_image()
        #
        #             if colors == [0.192, 1, 0.133]:
        #                 if 0 <= deltaTheta <= math.pi / 2:
        #                     state_machine["foundGoal"][1] = "left"
        #                     state_machine["foundGoal"][2] = pose_theta
        #                     if logging:
        #                         print("found left goal")
        #
        #                 elif math.pi/2 <= deltaTheta <= 3 * math.pi/2:
        #                     state_machine["foundGoal"][1] = "behind"
        #                     state_machine["foundGoal"][2] = pose_theta
        #                     if logging:
        #                         print("found behind goal")
        #                 elif 3 * math.pi/2 <= deltaTheta:
        #                     state_machine["foundGoal"][1] = "right"
        #                     state_machine["foundGoal"][2] = pose_theta
        #                     if logging:
        #                         print("found right goal")
        #                 state_machine["foundGoal"][0] = True
        #                 state_machine["searchingGoal"] = False
        #                 currentTime1 = robot.getTime()
        #             if logging:
        #                 print("finding goal: ", colors, pos_im)
                    #
    elif not state_machine["searchingGoal"][0]:
        if state_machine["foundGoal"][0]:
            leftSpeed = 0
            rightSpeed = 0
            currentTime2 = robot.getTime()
            if state_machine["foundGoal"][1] == "left":
                if currentTime2 - currentTime1>=15:
                    leftSpeed = 0.1 * MAX_SPEED
                    rightSpeed = 0.1 * MAX_SPEED
                if logging:
                    print("time diff: {}".format(currentTime2 - currentTime1))
                # if currentTime2 - currentTime2:

            elif state_machine["foundGoal"][1] == "behind":
                if currentTime2 - currentTime1>=0:
                    leftSpeed = 0.1 * MAX_SPEED
                    rightSpeed = 0.1 * MAX_SPEED
                if logging:
                    print("time diff: {}".format(currentTime2 - currentTime1))
            elif state_machine["foundGoal"][1] == "right":
                if currentTime2 - currentTime1>=7:
                    leftSpeed = 0.1 * MAX_SPEED
                    rightSpeed = 0.1 * MAX_SPEED
                if logging:
                    print("time diff: {}".format(currentTime2 - currentTime1))
        else:
            currentTime2 = robot.getTime()
            if currentTime2 - currentTime1 > 1:
                leftSpeed = 0.01 * MAX_SPEED
                rightSpeed = 0.01 * MAX_SPEED
            else:
                leftSpeed = 0.1 * MAX_SPEED
                rightSpeed = 0.1 * MAX_SPEED
    if np.random.rand() < 0.25:
        gpsCoordinates.append(gps.getValues()[0:2])
    if keyboard.getKey() == ord('S'):
        np.save("./output/" + sys.argv[1] + "_" + Trial + ".npy", gpsCoordinates)
        print("Map file saved: {}.npy".format(sys.argv[1] + "_" + Trial))

    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)

    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
