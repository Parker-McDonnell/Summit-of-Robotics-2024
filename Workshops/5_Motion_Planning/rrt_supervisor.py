# You may need to import some classes of the controller module. Ex:
#  from controller import Robot, Motor, DistanceSensor
from controller import Robot
from controller import Supervisor, Emitter
import sys
import struct
import numpy as np
import time

class Graph_Node:
  def __init__(self, position, parent, action=None):
    self.position = position
    self.parent = parent
    self.action = action

def randomly_sample_state():
    random = np.random.uniform(0,1)
    if(random < 0.5):
        random_coordinate_x = 0.4
        random_coordinate_y = 0.4
    else:
        random_coordinate_x = np.random.uniform(0,1)
        random_coordinate_y = np.random.uniform(0,1)
    return random_coordinate_x, random_coordinate_y 

#def monte_carlo_propagate():

def main():
    # Initialize the Webots Supervisor
    supervisor = Supervisor()
    graph = []
    start = Graph_Node([0,0,0],None)
    graph.append(start)
    goal = [0.4,0.4,0]
    # Get a reference to the iRobot Create node
    puck = supervisor.getFromDef("e-puck")
    emitter = supervisor.getDevice("emitter")
    receiver = supervisor.getDevice("receiver")
    emitter.setChannel(10)
    receiver.setChannel(10)
    timestep = int(supervisor.getBasicTimeStep())
    receiver.enable(timestep)
    #translation_field = irobot.getField('translation')
    
    # Run the simulation step by step
    while supervisor.step(timestep) != -1:
        success_count = 0
        random_coordinate_x,random_coordinate_y = randomly_sample_state()
        nearest_node = None
        nearest_node_distance = 100
        for node in graph:
            if(np.linalg.norm(np.array(node.position)-np.array([random_coordinate_x,random_coordinate_y,0])) < nearest_node_distance):
               nearest_node_distance = np.linalg.norm(np.array(node.position)-np.array([random_coordinate_x,random_coordinate_y,0]))
               nearest_node = node
        puck.getField("translation").setSFVec3f(nearest_node.position)
        message = struct.pack('f', 1)
        #message_y = struct.pack('f', random_coordinate_y)
        emitter.send(message)
        #emitter.send(message_y)
        monte_carlo_prop_final_states = []
        current_state = puck.getField("translation").getSFVec3f()
        while(success_count < 5):    
            while(receiver.getQueueLength() == 0):
                supervisor.step(timestep)
            data = receiver.getBytes()
            success, left_speed, right_speed = struct.unpack('fff', data)
            receiver.nextPacket()
            #success = receiver.getBytes()
            #success = struct.unpack('f', success)[0]
            #receiver.nextPacket()
            #left_speed = receiver.getBytes()
            #left_speed = struct.unpack('f', left_speed)[0]
            #receiver.nextPacket()            
            #right_speed = receiver.getBytes()
            #right_speed = struct.unpack('f', right_speed)[0]
            action = [left_speed, right_speed]
            success = bool(success)
            print("THE VALUES ARE", success, action)
            if success:
                success_count+=1
                position = puck.getField("translation").getSFVec3f()
                monte_carlo_prop_final_states.append([position,action])
            else:
                print("Collision Occured!")
            puck.getField("translation").setSFVec3f(nearest_node.position)
            supervisor.step(timestep)
        nearest_state = None
        nearest_state_distance = 100
        nearest_state_action = None
        for state in monte_carlo_prop_final_states:
            if np.linalg.norm(np.array([random_coordinate_x,random_coordinate_y,0])-np.array(state[0])) < nearest_state_distance:
                nearest_state_distance = np.linalg.norm(np.array([random_coordinate_x,random_coordinate_y,0])-np.array(state[0]))
                nearest_state = state[0]
                nearest_state_action = state[1]
        print("ADDED TO TREE", nearest_state)
        node = Graph_Node(nearest_state,current_state,nearest_state_action)
        graph.append(node)
        if np.linalg.norm(np.array(nearest_state)-np.array([0.4,0.4,0])) < 0.1:
            print("GOAL REACHED")
            time.sleep(100)
            final_actions = []
            current_node = graph[-1]
            while(current_node is not None):
                final_actions.append(current_node.action)
                current_node = current_node.parent
            final_actions.reverse()
            puck.getField("translation").setSFVec3f([0,0,0])
            message = struct.pack('f', 1)
            emitter.send(message)
            supervisor.step(timestep)
            for i in final_actions:
                left_speed = struct.pack('f', i[0])
                right_speed = struct.pack('f', i[1])
                emitter.send(left_speed)
                emitter.send(right_speed)
                supervisor.step(timestep)
            message = struct.pack('f', 1000)
            emitter.send(message)
            emitter.send(message)
            break    
            
if __name__ == "__main__":
    main()
