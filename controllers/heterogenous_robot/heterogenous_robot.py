"""heterogenous_robot controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot, Camera, CameraRecognitionObject

# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getDevice('motorname')
#  ds = robot.getDevice('dsname')
#  ds.enable(timestep)
camera = Camera("camera")
camera.enable(timestep)
camera.recognitionEnable(timestep)
counter = 0

# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    counter += 1
    if counter % 10:
        print(timestep)
        x = camera.getImage()
        y = camera.getRecognitionObjects()
        #list of all objects detected
        for i in y:
            pos = i.get_position()
            # x,y,z
            colors = i.get_colors()
            # [R,G,B]
            # orange ball: [1.0, 0.54, 0.08]
            pos_im = i.get_position_on_image()
            # [x,y] on image coordinates of the detected object
            print(pos, colors, pos_im)
    # Process sensor data here.

    # Enter here functions to send actuator commands, like:
    #  motor.setPosition(10.0)
    pass

# Enter here exit cleanup code.
