"""controller_py controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import InertialUnit
from controller import Keyboard
from controller import RangeFinder
# create the Robot instance.
robot = Robot()

# get the time step of the current world.
timestep = int(robot.getBasicTimeStep())

# You should insert a getDevice-like function in order to get the
# instance of a device of the robot. Something like:
#  motor = robot.getMotor('motorname')
#  ds = robot.getDistanceSensor('dsname')
#  ds.enable(timestep)
Inertial = robot.getInertialUnit("imu")
Inertial.enable(timestep)

kb = Keyboard()
kb.enable(timestep)


RangeFinder = robot.getRangeFinder("d435")
RangeFinder.enable(timestep)
RangeFinder.getWidth()
RangeFinder.getHeight()




wheels = []
wheels_names = [ "motor_rr","motor_rl",  "motor_fr","motor_fl"]
for i in range(4):
    wheels.append(robot.getMotor(wheels_names[i]))
    wheels[i].setPosition(float('inf'))
    wheels[i].setVelocity(0)

print ("init successd ...\n");

  
leftSpeed = 0.0;
rightSpeed = 0.0;
results = []
# Main loop:
# - perform simulation steps until Webots is stopping the controller
while robot.step(timestep) != -1:
    # Read the sensors:
    # Enter here functions to read sensor data, like:
    #  val = ds.getValue()
    i = 0
    key = kb.getKey();
    if key==315:
      leftSpeed = -3.0;
      rightSpeed = -3.0;
    elif(key== 317):
      leftSpeed = 3.0;
      rightSpeed = 3.0;
    elif(key== 314):
      leftSpeed = -3.0;
      rightSpeed = 3.0;
    elif(key== 316):
      leftSpeed = 3.0;
      rightSpeed = -3.0;
    else:
      leftSpeed = 0.0;
      rightSpeed = 0.0;
      
    wheels[0].setVelocity(rightSpeed)
    wheels[1].setVelocity(leftSpeed)
    wheels[2].setVelocity(rightSpeed)
    wheels[3].setVelocity(leftSpeed)
    # Process sensor data here.

    imageData = RangeFinder.getRangeImage()
    imageArray = RangeFinder.getRangeImageArray()
    RangeFinder.rangeImageGetDepth(imageData,RangeFinder.getWidth(),5,10)


   
    with open('results.txt', 'a') as f:
        f.write(str(round(Inertial.getRollPitchYaw()[0],2))+'\t')
        f.write(str(round(Inertial.getRollPitchYaw()[1],2))+'\t')
        f.write(str(round(Inertial.getRollPitchYaw()[2],2))+'\n')
    
# Enter here exit cleanup code.
print ("ending ...\n");