


import math
import time
import matplotlib.pyplot as plt
from utils import * #functions and classes python file

#it's a simulation for 4 differents initial positons of the robot
#it takes around 6 minutes   

robot = Robot("Jimmy", 0.7,0.7, [0,0,0])

#gain values (rho, alpha , beta)
gain = [3,8,-3.5]
controller = Controller(gain[0], gain[1] , gain[2])
iteration_time_sec = 0.1
robot.set_dtime(iteration_time_sec)

#target position (x,y, orientation)
target_pos = [10, 10,180]

# Initialize robot time:
t_k = time.time()
final_time = time.time()
k=1


color = ["red","gray","blue","green"]
pose = [0,0,0,20,0,0,0,20,0,20,20,0]


for i in range(len(color)):
    #draw the robot initial position
    tx = 0.5 + 0.5 * math.cos(pose[i*3+2]*math.pi/180)
    ty = 0.5 + 0.5 * math.sin(pose[i*3+2]*math.pi/180)
    plt.plot([pose[i*3]-tx, pose[i*3]-tx, pose[i*3]+tx, pose[i*3]+tx,pose[i*3]-tx], [pose[i*3+1]-ty, pose[i*3+1]+ty, pose[i*3+1]+ty, pose[i*3+1]-ty,pose[i*3+1]-ty])
    plt.plot([pose[i*3], pose[i*3]+tx-0.5], [pose[i*3+1], pose[i*3+1]+ty-0.5],c=color[i])
    plt.plot([pose[i*3], pose[i*3]+ty-0.5], [pose[i*3+1], pose[i*3+1]+tx-0.5],c=color[i])

    #set the initial position
    robot.set_position(pose[i*3],pose[i*3+1],pose[i*3+2])

    while True:
        if distance_to_target(robot.get_position()[:2], target_pos[:2]) < 0.31:
            print("time taken to reach target is", final_time - t_k)
            break
        else:
            f, r = controller.simulation(robot.get_position(), target_pos)
            # Compute forward and rotation speed with controller
            robot.set_forward_speed(f)
            robot.set_rotational_speed(r) # set speed to robot        
            # t_k = current_time        
            time.sleep(iteration_time_sec)        
            currrent_time = time.time()

            # update robot pos with t_k and current_time
            robot.update_position()
            print(f"step :{k} \n forwardspeed = ", robot.get_forward_speed(),"r_ speed : ", robot.get_rotational_speed(), " current position :" , robot.get_position())
            print("Target pos",target_pos)
            print('Distance', distance_to_target(robot.get_position()[:2], target_pos[:2]))
            
            #plot the trajectory
            plt.scatter(robot.get_position()[0], robot.get_position()[1],c=color[i],s=0.5)# shows the position of the robot (X,Y) at each iteration
            final_time = time.time()
            k = k+1
    
#draw the robot target position
tx = 0.5 + 0.5 * abs(math.cos(target_pos[2]*math.pi/180))
ty = 0.5 + 0.5 * abs(math.sin(target_pos[2]*math.pi/180))
plt.plot([target_pos[0]-tx, target_pos[0]-tx, target_pos[0]+tx, target_pos[0]+tx,target_pos[0]-tx], [target_pos[1]-ty, target_pos[1]+ty, target_pos[1]+ty, target_pos[1]-ty,target_pos[1]-ty],c="green")
plt.plot([target_pos[0], target_pos[0]+tx-0.5], [target_pos[1], target_pos[1]+ty-0.5],c="red")

print("{} arrived to the target!".format(robot.get_name()))
title = "gain (rho , alpha , beta ) : " 
plt.suptitle(title + str(gain)+ "\ntarget position "+str(target_pos))
plt.show()
