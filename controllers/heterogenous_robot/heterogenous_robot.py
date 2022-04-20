"""heterogenous_robot controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Camera, CameraRecognitionObject, Compass, InertialUnit
import math, sys
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

camera = Camera("camera")
camera.enable(timestep)
camera.recognitionEnable(timestep)

leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftSpeed, rightSpeed = 0, 0

logging = False
if len(sys.argv)> 1 and sys.argv[1] == "True":
    logging = True

state_machine = {"foundObject" : False, "aligned" : False, "isNearObject" : False, "searchingGoal" : False, "goalFindable" : [False, None], "foundGoal" : [False, None, None, None]}

deltaTheta = 0

counter = 0

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    n = compass.getValues()
    rad = -math.atan2(n[0], n[1])
    pose_theta = rad
    if not state_machine["foundObject"]:
        leftSpeed = -0.5 * MAX_SPEED
        rightSpeed = 0.5 * MAX_SPEED
        cameraRecogObjects = camera.getRecognitionObjects()
        # list of all objects detected
        for obj in cameraRecogObjects:
            # pos = i.get_position()
            # x,y,z
            colors = obj.get_colors()
            print(colors)
            # [R,G,B]
            # orange ball: [1.0, 0.54, 0.08]
            if colors == [1, 0.54, 0.08]:
                state_machine["foundObject"] = True
                if logging:
                    print("found Orange ball")
                # print("[{} == {}?={}, {} == {}?={}, and {} == {}?={}]".format(colors[0], 1, colors[0] == 1, colors[1],
                #                                                               0.54, colors[1] == 0.54, colors[2], 0.08,
                #                                                               colors[2] == 0.08))
            else:
                pass
                # print("[{} == {}?={}, {} == {}?={}, and {} == {}?={}]".format(colors[0], 1, colors[0] == 1, colors[1],
                #                                                               0.54, colors[1] == 0.54, colors[2], 0.08,
                #                                                               colors[2] == 0.08))
    elif not state_machine["aligned"]:
        cameraRecogObjects = camera.getRecognitionObjects()
        for obj in cameraRecogObjects:
            colors = obj.get_colors()
            if colors == [1, 0.54, 0.08]:
                pos_im = obj.get_position_on_image()
                if math.isclose(pos_im[0],camera.getWidth()/2):
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
                if objectPostion[0] <= 0.085:
                    state_machine["isNearObject"] = True
                    leftSpeed = 0
                    rightSpeed = 0
                    state_machine["searchingGoal"] = True
                    deltaTheta = 0
                    if logging:
                        print("epuck is at ball")
                # print("Object position: {}".format(objectPostion))
    elif state_machine["searchingGoal"]:
        # Green RGB
        # 0.192 1 0.133
        if not state_machine["foundGoal"][0]:
            if deltaTheta == 0:
                previousTheta = pose_theta
                deltaTheta += 0.0001
            else:
                deltaTheta += abs(abs(previousTheta) - abs(pose_theta))
                previousTheta = pose_theta
            if logging:
                print("Current angle: {}, current amount of rotation: {}".format(pose_theta, deltaTheta))
            if deltaTheta >= 2 * math.pi:
                leftSpeed = 0
                rightSpeed = 0
                if logging:
                    print("Goal findable: {}".format(state_machine["goalFindable"][0]))
            else:

                leftSpeed = -0.05 * MAX_SPEED
                rightSpeed = 0.05 * MAX_SPEED
                cameraRecogObjects = camera.getRecognitionObjects()
                # list of all objects detected
                goalCount = 0
                for obj in cameraRecogObjects:
                    # pos = i.get_position()
                    # x,y,z
                    colors = obj.get_colors()
                    pos_im = obj.get_position_on_image()

                    if colors == [0.192, 1, 0.133]:
                        if 0 <= deltaTheta <= math.pi / 2:
                            if logging:
                                print("found left goal")
                        elif math.pi/2 <= deltaTheta <= 3 * math.pi/2:
                            if logging:
                                print("found behind goal")
                        elif 3 * math.pi/2 <= deltaTheta:
                            if logging:
                                print("found right goal")
                        state_machine["goalFindable"][0] = True
                        goalCount += 1
                if logging:
                    print("finding goal: ", colors, pos_im)

        else:
            leftSpeed = 0
            rightSpeed = 0

    else:
        pass

    leftMotor.setVelocity(leftSpeed)
    rightMotor.setVelocity(rightSpeed)



    # angles = Inertialun.getRollPitchYaw()
    #
    # print("Angle: {}".format(angles))
    #
    #
    # counter += 1
    # if counter % 10:
    #     print(timestep)
    #     x = camera.getImage()
    #     y = camera.getRecognitionObjects()
    #     #list of all objects detected
    #     for i in y:
    #         pos = i.get_position()
    #         # x,y,z
    #         colors = i.get_colors()
    #         # [R,G,B]
    #         # orange ball: [1.0, 0.54, 0.08]
    #         pos_im = i.get_position_on_image()
            # [x,y] on image coordinates of the detected object
            # print("position relative to epuck{}, color: {}, position on image: {}, current angle: {}".format(pos, colors, pos_im, current_pose))
    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
