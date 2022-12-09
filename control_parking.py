#!/usr/bin/env python3



import rospy

from geometry_msgs.msg import Twist

from pyueye import ueye
import numpy as np
import cv2
from utils import * #functions and classes python file

# init camera
hcam = ueye.HIDS(0)
ret = ueye.is_InitCamera(hcam, None)
print(f"inittheta_currentCamera returns {ret}")
# set color mode
ret = ueye.is_SetColorMode(hcam, ueye.IS_CM_BGR8_PACKED)
print(f"SetColorMode IS_CM_BGR8_PACKED returns {ret}")
# set region of interest
width = 1280
height = 1080
rect_aoi = ueye.IS_RECT()
rect_aoi.s32X = ueye.int(0)
rect_aoi.s32Y = ueye.int(0)
rect_aoi.s32Width = ueye.int(width)
rect_aoi.s32Height = ueye.int(height)
ueye.is_AOI(hcam, ueye.IS_AOI_IMAGE_SET_AOI, rect_aoi, ueye.sizeof(rect_aoi))
print(f"AOI IS_AOI_IMAGE_SET_AOI returns {ret}")
# allocate memory
mem_ptr = ueye.c_mem_p()
mem_id = ueye.int()
bitspixel = 24 # for colormode = IS_CM_BGR8_PACKED
ret = ueye.is_AllocImageMem(hcam, width, height, bitspixel,
                            mem_ptr, mem_id)
print(f"AllocImageMem returns {ret}")

# set active memory region
ret = ueye.is_SetImageMem(hcam, mem_ptr, mem_id)
print(f"SetImageMem returns {ret}")
# continuous capture to memory5
ret = ueye.is_CaptureVideo(hcam, ueye.IS_DONT_WAIT)
print(f"CaptureVideo returns {ret}")

# get data from camera and display
lineinc = width * int((bitspixel + 7) / 8)



#init publisher

TwistMsg = Twist

rospy.init_node('gotogoal')
pub=rospy.Publisher('/cmd_vel', TwistMsg, queue_size = 1)
twist_msg = TwistMsg()
rate=rospy.Rate(50)
twist = twist_msg




#start a counter
k=1

while True:
    #get the image from the camera
    img = ueye.get_data(mem_ptr, width, height, bitspixel, lineinc, copy=True)
    img = np.reshape(img, (height, width, 3))

    #get robot and target position from the photos
    initial_pose = get_pos(img)
    img_target = cv2.imread("/home/esirem/Medhi_rob/robot_ws/src/msft/src/Goal_target5.png")
    target_pos = get_pos(img_target) 

    #transformation matrix to work in the robot frame
    Mr = get_M(initial_pose)
    Mt = get_M(target_pos)
    M = get_Mr_t(Mr,Mt)
    target_pos = get_Target_inRobot_frame(M)
    initial_pose = [0,0,0]

    #init the robot object with the initial position from the get_pos function
    robot = Robot("Jimmy", 0.1, 1, initial_pose) 

    #init the controller with the values of the 3 parameters (rho, alpha and beta)
    controller = Controller(0.01, 0.9,-0.7)
    iteration_time_sec = 0.001
    robot.set_dtime(iteration_time_sec)


    # Initialize robot time:
    t_k = time.time()
    final_time = time.time()
    
    #check the distance to the target
    if (distance_to_target(robot.get_position()[:2], target_pos[:2]) < 40) :  
        #if the distance to the target less than 40 pixels 
        # 40 pixels equal to 130/(12*40) =  3.69 cm of acceptable error
        # correct the error between the target and the robot orientation
        if(abs(robot.get_position()[2]*180/math.pi - target_pos[2]) < 10):
            print("time taken to reach target is", final_time - t_k)
            print("{} arrived to the target!".format(robot.get_name()))
        else:
            # Correct the angle
            f,r = controller.correct_parking(robot.get_position(), target_pos)
            
            robot.set_forward_speed(f)
            robot.set_rotational_speed(r) # set speed to robot to control the max and the minimum speed    

            # t_k = current_time        
            time.sleep(iteration_time_sec)        
            currrent_time = time.time()

            print(f"step :{k} \n forwardspeed = ", robot.get_forward_speed(),"r_ speed : ", robot.get_rotational_speed(), " current position :" , robot.get_position())
            print('Distance', distance_to_target(robot.get_position()[:2], target_pos[:2]))
            final_time = time.time()
            #move robot 
            twist.linear.x = robot.get_forward_speed()
            print("vitesse x", robot.get_forward_speed())
            twist.linear.y = robot.get_forward_speed()
            print("vitesse y", robot.get_forward_speed())
            twist.linear.z = 0
            twist.angular.x = 0
            twist.angular.y = 0
            twist.angular.z = r
            pub.publish(twist)
            rate.sleep()
            k=k+1
            
        
    else:
        # Compute forward and rotation speed with controller
        f,r = controller.parking(robot.get_position(), target_pos)
        
        robot.set_forward_speed(f)
        robot.set_rotational_speed(r) # set speed to robot        
    
        time.sleep(iteration_time_sec)        
        currrent_time = time.time()
        
        print(f"step :{k} \n forwardspeed = ", robot.get_forward_speed(),"r_ speed : ", robot.get_rotational_speed(), " current position :" , robot.get_position())

        print('Distance', distance_to_target(robot.get_position()[:2], target_pos[:2]))
        final_time = time.time()


        #move robot (publish)
        twist.linear.x = robot.get_forward_speed()
        print("vitesse x", robot.get_forward_speed())
        twist.linear.y = robot.get_forward_speed()
        print("vitesse y", robot.get_forward_speed())
        twist.linear.z = 0
        twist.angular.x = 0
        twist.angular.y = 0
        twist.angular.z = robot.get_rotational_speed()
        pub.publish(twist)
        rate.sleep()
        k=k+1

    cv2.imshow('uEye Python Example (q to exit)', img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
    if cv2.waitKey(1) & 0xFF == ord('s'):
        cv2.imwrite("Goal target",img)
        break
    
cv2.destroyAllWindows()

# cleanup
ret = ueye.is_StopLiveVideo(hcam, ueye.IS_FORCE_VIDEO_STOP)
print(f"StopLiveVideo returns {ret}")
ret = ueye.is_ExitCamera(hcam)
print(f"ExitCamera returns {ret}")




