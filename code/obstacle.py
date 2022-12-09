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
#Front_right
pub=rospy.Publisher('/cmd_vel', TwistMsg, queue_size = 1)
twist_msg = TwistMsg()
rate=rospy.Rate(50)
twist = twist_msg


#get the image and detect the initial position
strt_img = ueye.get_data(mem_ptr, width, height, bitspixel, lineinc, copy=True)
strt_img = np.reshape(strt_img, (height, width, 3))
start_position = get_pos(strt_img)
print(start_position)

#get the target image and detect the target position
img_target = cv2.imread("/home/esirem/Medhi_rob/robot_ws/src/msft/src/Goal_target5.png")
target_posinImage = get_pos(img_target) 

#calculate the path
path , path_img = get_path(strt_img,start_position[:2],target_posinImage[:2])
print(path)
cv2.imshow('path image',path_img)
cv2.waitKey(0)


target_pos = target_posinImage 

for node in path:
    #move the robot to the nodes in the path
    #start a counter
    k=1
    while True:
        img = ueye.get_data(mem_ptr, width, height, bitspixel, lineinc, copy=True)
        img = np.reshape(img, (height, width, 3))
        robot = Robot("Jimmy", 0.1, 1,get_pos(img))
        controller = Controller(0.001, 2,-0.7)
        iteration_time_sec = 0.001
        robot.set_dtime(iteration_time_sec)
        
        # Initialize robot time:
        t_k = time.time()  
        final_time = time.time()


        if (distance_to_target(robot.get_position()[:2], node[:2]) < 40) :  
            print("time taken to reach to", node ,"is", final_time - t_k)
            print(target_pos)
            print(f"{format(robot.get_name())} arrived!")
            break
        else:
            f,r = controller.robot_path(robot.get_position(), node)
            robot.set_forward_speed(f)
            robot.set_rotational_speed(r) # set speed to robot        
      
            time.sleep(iteration_time_sec)        
            currrent_time = time.time()

            print(f"step :{k} \n forwardspeed = ", robot.get_forward_speed(),"r_ speed : ", robot.get_rotational_speed(), " current position :" , robot.get_position())
            print('Target node',node)
            print('Target position',target_pos)
            print('Distance', distance_to_target(robot.get_position()[:2], node[:2]))
            final_time = time.time()   
            #move robot    
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




