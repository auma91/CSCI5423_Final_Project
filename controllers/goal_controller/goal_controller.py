"""goal_controller controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot

# create the Robot instance.
robot = Robot()

MAX_SPEED = 6.28

leftMotor = robot.getDevice('left wheel motor')
rightMotor = robot.getDevice('right wheel motor')

leftMotor.setPosition(float('inf'))
rightMotor.setPosition(float('inf'))


# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)

# Main loop:
# - perform simulation steps until Webots is stopping the controller

    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()

    # Process sensor data here.
leftMotor.setVelocity(0)
rightMotor.setVelocity(0)

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
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

state_machine = {"intial360" : [False, None, None, None, None], "alignmentphase" : False, "aligned" : False, "isNearObject" : False, "searchingGoal" : False, "goalFindable" : [False, None],
                 "foundGoal" : [False, None, None, None]}

deltaTheta = 0

counter = 0

currentTime1, currentTime2 = 0,0

n = compass.getValues()
rad = -math.atan2(n[0], n[1])
pose_theta = rad

deltaTheta = 0

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    n = compass.getValues()
    rad = -math.atan2(n[0], n[1])
    pose_theta = rad
    if not state_machine["intial360"][0]:
        if deltaTheta == 0:
            previousTheta = pose_theta
            deltaTheta += 0.000001
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
                    #found orange ball
                    # state_machine["intial360"][1] = pose_theta

                    if state_machine["intial360"][1] is None:
                        if logging:
                            print("found Orange ball")
                        if abs(pos_im[0] - camera.getWidth()/2) <= 1:
                            state_machine["intial360"][1] = (pose_theta, objectPostion, pos_im)
                            if logging:
                                print("Orange ball in alignement: {}".format(state_machine["intial360"][1]))
                        # print("[{} == {}?={}, {} == {}?={}, and {} == {}?={}]".format(colors[0], 1, colors[0] == 1, colors[1],
                        #                                                               0.54, colors[1] == 0.54, colors[2], 0.08,
                        #                                                               colors[2] == 0.08))
               
    pass

# Enter here exit cleanup code.
