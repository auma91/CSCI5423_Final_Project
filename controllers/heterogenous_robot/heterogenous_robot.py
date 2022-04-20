"""heterogenous_robot controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Camera, CameraRecognitionObject, Compass, InertialUnit
import math
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

Inertialun = robot.getDevice("inertial unit")
Inertialun.enable(timestep)

camera = Camera("camera")
camera.enable(timestep)
camera.recognitionEnable(timestep)

leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))
leftSpeed, rightSpeed = 0, 0

state_machine = {"foundObject" : False, "aligned" : False, "isNearObject" : False, "searchingGoal" : False, "goalFindable" : [False, 0], "foundGoal" : [False, None, None, None]}

counter = 0

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
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
                    print("aligned epuck")
                    leftSpeed = 0
                    rightSpeed = 0
                elif (camera.getWidth() / 2) < pos_im[0] < ((camera.getWidth() / 2) + (camera.getWidth() / 16)):
                    print("Epuck to the right of center")
                    leftSpeed = 0.05 * MAX_SPEED
                    rightSpeed = -0.05 * MAX_SPEED
                elif ((camera.getWidth() / 2) - (camera.getWidth() / 16)) < pos_im[0] < (camera.getWidth() / 2):
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
                    state_machine["searchingGoal"] = [True, InertialUnit.getRollPitchYaw()]
                    print("epuck is at ball")
                print("Object position: {}".format(objectPostion))
    elif state_machine["searchingGoal"][0]:
        # Green RGB
        # 0.192157
        # 1
        # 0.133333

        print("Current angle: {}, 360 angle: {}".format(InertialUnit.getRollPitchYaw()[0], state_machine["searchingGoal"][1]))
        if math.isclose(InertialUnit.getRollPitchYaw()[0], state_machine["searchingGoal"][1]):
            pass
        else:
            leftSpeed = -0.05 * MAX_SPEED
            rightSpeed = 0.05 * MAX_SPEED
            cameraRecogObjects = camera.getRecognitionObjects()
            # list of all objects detected
            for obj in cameraRecogObjects:
                # pos = i.get_position()
                # x,y,z
                colors = obj.get_colors()
                pos_im = obj.get_position_on_image()
                print("finding goal: ", colors, pos_im)
                # [R,G,B]
                # orange ball: [1.0, 0.54, 0.08]

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
