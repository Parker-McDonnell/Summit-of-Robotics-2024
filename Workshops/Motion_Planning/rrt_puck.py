"""rrt_puck controller."""

# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
import struct
import random

# create the Robot instance.

def monte_carlo_propagate():
    success = True
    leftMotor = robot.getDevice('left wheel motor')
    rightMotor = robot.getDevice('right wheel motor')
    leftMotor.setPosition(float('inf'))
    rightMotor.setPosition(float('inf'))


    # Set the desired duration of random motion (in seconds)
    duration = 2.0
    timestep = int(robot.getBasicTimeStep())
    prox_sensors = []
    for i in range(8):
        prox_sensors.append(robot.getDevice("ps" + str(i)))
        prox_sensors[i].enable(timestep)

    start_time = robot.getTime()
    left_speed = random.uniform(-5.28, 5.28)
    right_speed = random.uniform(left_speed-1, left_speed+1)
    leftMotor.setVelocity(left_speed)
    rightMotor.setVelocity(right_speed)
    timestep = 32
    while robot.step(timestep) != -1:
        current_time = robot.getTime()
        elapsed_time = current_time - start_time

        if elapsed_time < duration:
            # Generate random velocities
            for i in range(8):
                value = prox_sensors[i].getValue()
                if value > 100:  # Adjust the threshold as needed
                    success = False
                    return success, [0,0]
            pass
        else:
            leftMotor.setVelocity(0)
            rightMotor.setVelocity(0)
            # Stop the robot's motion
            #leftMotor.setVelocity(0.0)
            #rightMotor.setVelocity(0.0)
            return success, [left_speed, right_speed]



robot = Robot()
timestep = int(robot.getBasicTimeStep())
emitter = robot.getDevice("emitter")
receiver = robot.getDevice("receiver")
emitter.setChannel(10)
receiver.setChannel(10)
receiver.enable(timestep)

while robot.step(timestep) != -1:
    if receiver.getQueueLength() > 0:
        message = receiver.getBytes()
        receiver.nextPacket()
        #message_y = receiver.getBytes()
        #receiver.nextPacket()
        message = struct.unpack('f', message)[0]
        #message_y = struct.unpack('f', message_y)[-1]
        #print("Received float:", message_x, message_y)
        if message == 1:
            success_count = 0
            while(success_count <5):
                success, action = monte_carlo_propagate()
                print(success, type(success), action, type(action[0]))
                #success = struct.pack('f', float(success))
                #left_speed = struct.pack('f', action[0])
                #right_speed = struct.pack('f', action[1])
                data_bytes = struct.pack('fff', float(success), action[0], action[1])
                #emitter.send(success)
                #emitter.send(left_speed)
                #emitter.send(right_speed)
                emitter.send(data_bytes)
        else:
            print("ENTERED FINAL STAGE")
            while robot.step(timestep) != -1:
                
                leftMotor = robot.getDevice('left wheel motor')
                rightMotor = robot.getDevice('right wheel motor')
                leftMotor.setPosition(float('inf'))
                rightMotor.setPosition(float('inf'))
                
                left_speed = receiver.getBytes()
                receiver.nextPacket()
                right_speed = receiver.getBytes()
                receiver.nextPacket()
                
                left_speed = struct.unpack('f', left_speed)[0]
                right_speed = struct.unpack('f', right_speed)[0]
                
                if left_speed==1000 or right_speed == 1000:
                    break
                
                action = [left_speed, right_speed]
                timestep = 32
                
                leftMotor.setVelocity(left_speed)
                rightMotor.setVelocity(right_speed)
                duration = 2
                start_time = robot.getTime()
                while robot.step(timestep) != -1:
                    
                    current_time = robot.getTime()
                    elapsed_time = current_time - start_time
            
                    if elapsed_time < duration:
                        pass
                    else:
                        leftMotor.setVelocity(0)
                        rightMotor.setVelocity(0)
    pass

# Enter here exit cleanup code.
