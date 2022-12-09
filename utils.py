

import time
import math
import matplotlib.pyplot as plt
import numpy as np
import cv2
import cv2.aruco as aruco


ARUCO_DICT = aruco.Dictionary_get(aruco.DICT_4X4_1000)
ARUCO_PARAMETERS = aruco.DetectorParameters_create()
robot_size = 130 #Robot size on pixels according to the camera used and it's focal length

#calculate the transformation matrix of a frame 
# from the position of the origine and the angle of orientation according to the axis Z (perpendicular to the image plan)
def get_M(pose):
    theta = pose[2]*math.pi/180
    M = np.array([[math.cos(theta), -math.sin(theta), pose[0]], 
    [math.sin(theta), math.cos(theta), pose[1]], 
    [0,0,1]])
    return M

#from the data that we get from the pose estimaton function 
#this function can calculate the transformation matrix
def get_M_from_pose(pose):
    rvec= pose['rvec']
    rvec = list(rvec[0])
    R_martix= cv2.Rodrigues(np.array(rvec))[0]
    tvec = pose['tvec']
    TransformationMatrix = np.vstack( [np.hstack([R_martix,tvec.reshape(-1,1)])  , [0,0,0,1]])
    return TransformationMatrix

#The transformation matrix from the target frame to the robot Tt_r 
#we multiply the inverse of the robot matrix by the target matrix
def get_Mr_t(Mr,Mt): 
    return np.linalg.inv(Mr).dot(Mt)


def get_M_Radian(pose):
    theta = pose[2]
    M = np.array([[math.cos(theta), -math.sin(theta), pose[0]], 
    [math.sin(theta), math.cos(theta), pose[1]], 
    [0,0,1]])

    return M

#get target and the direction of the target from the transformation matrix
def get_Target_inRobot_frame(M):
    if M.shape == (3,3):
        x = M[0][2]
        y = M[1][2]
        #arctang of 
        theta = math.atan2(M[1,0], M[0,0]) *180/math.pi

    else: 
        x = M[0][3]
        y = M[1][3]
        theta = rotationMatrixToEulerAngles(M[:3,:3])[2]*180/math.pi #return in degrees

    return [x,y,theta]


def rotationMatrixToEulerAngles(R) :

    sy = math.sqrt(R[0,0] * R[0,0] + R[1,0] * R[1,0])
    singular = sy < 1e-6
    if not singular :
        x = math.atan2(R[2,1] , R[2,2])
        y = math.atan2(-R[2,0], sy)
        z = math.atan2(R[1,0], R[0,0])
    else :
        x = math.atan2(-R[1,2], R[1,1])
        y = math.atan2(-R[2,0], sy)
        z = 0
    return np.array([x, y, z])

def eulerAnglesToRotationMatrix(theta) :

    R_x = np.array([[1, 0, 0 ],
                [0, math.cos(theta[0]), -math.sin(theta[0]) ],
                [0, math.sin(theta[0]), math.cos(theta[0]) ]
                ])
    
    R_y = np.array([[math.cos(theta[1]), 0, math.sin(theta[1]) ],
                    [0,                     1, 0 ],
                    [-math.sin(theta[1]),   0, math.cos(theta[1]) ]
                    ])
 
    R_z = np.array([[math.cos(theta[2]),    -math.sin(theta[2]),    0],
                    [math.sin(theta[2]),    math.cos(theta[2]),     0],
                    [0,                     0,                      1]
                    ])
 
    R = np.dot(R_z, np.dot( R_y, R_x ))
 
    return R


def get_inR(M,pose):
    b = np.linalg.inv(M)
    return b.dot(pose)

#distance from two points
def distance_to_target(current_position, target_position):
    distance = math.sqrt((target_position[0]-current_position[0])**2 + (target_position[1]-current_position[1])**2 )
    return distance

#get position and the orientation of the marker from the image in pixels
def get_pos(img):
    corners, ids, rejectedImgPoints = aruco.detectMarkers(img, ARUCO_DICT, parameters=ARUCO_PARAMETERS)
    if ids is not None:
        img = aruco.drawDetectedMarkers(img, corners, borderColor=(255, 0, 0))
        centerx = (corners[0][0][0][0] + corners[0][0][2][0])/2
        centery = (corners[0][0][0][1] + corners[0][0][2][1])/2
        theta = math.atan2(  corners[0][0][3][1] - corners[0][0][0][1],corners[0][0][3][0] - corners[0][0][0][0] )*180/math.pi
        pnt_imterx = (int(corners[0][0][1][0]), int(corners[0][0][1][1]))
        pnt_imtery = (int(corners[0][0][3][0]), int(corners[0][0][3][1]))
        cv2.line(img, (int(corners[0][0][0][0]), int(corners[0][0][0][1])), pnt_imterx, (0, 255, 200), 3)
        cv2.line(img, (int(corners[0][0][0][0]), int(corners[0][0][0][1])), pnt_imtery, (0, 0, 255), 3)
    else:
        print('NONE')
        centerx = 0
        centery = 0
        theta = 0
    return [int(centerx),int(centery),theta]


#This function split the image to boxes then calculate the cost of each one according to the distance and the obstacles. 
#Then return a path of points of pixels in the image using an algorithm that we developped.
def get_path(img,robot_pose,target): 

    #Image processing and red mask to detect the obstacles

    lower_red = np.array([0,0,200]) #lower value of red pixels
    upper_red = np.array([80,80,255]) #Upper value of red pixels
    red_mask = cv2.inRange(img,lower_red,upper_red) 
    red_color = cv2.bitwise_and(img,img,mask=red_mask) #apply the red mask to the image

    #Take the height and the width of the image
    img_height = img.shape[0]
    img_width =img.shape[1]

    box_list =[]
    obstacle_box = []
    #split the image into boxes of the size of the robot and draw rectangles
    for y in range(int(img_height/robot_size))  :
         for x in range(int(img_width/robot_size)) :
            r,g,b = img[int(robot_size/2 + y*robot_size) ,int(robot_size/2 + x*robot_size)]
            cv2.rectangle(img , (0 + x*robot_size , 0 + y*robot_size), (robot_size + x*robot_size , robot_size + y*robot_size), (0 , 240 , 0) , 2)
            nbre_red_pixel = 0
            #check for obstacles
            for yl in range(0+ y*robot_size, robot_size + y*robot_size):
                for xl in range(0 + x*robot_size,robot_size + x*robot_size):
                    r,g,b = red_color[int(yl) ,int(xl)] #check the value of the pixels in the red color of the image
                    if (r > 20)or (g > 20)or (b>200):
                        nbre_red_pixel +=1
  
            if nbre_red_pixel > 100: #if there are more than 100 red pixels in a box we consider it inavailable
                cv2.circle(img, (int(robot_size/2 + x*robot_size) ,int(robot_size/2 + y*robot_size)), int(robot_size/2), (0 , 0 , 255), 5)
                print("pose", (robot_size/2 + x*robot_size,robot_size/2 + y*robot_size) )
                box = Box( str(x) + str(y),(0 + x*robot_size , 0 + y*robot_size),(robot_size + x*robot_size , robot_size + y*robot_size),True)
                obstacle_box.append(box)
            else:
                box = Box( str(x) + str(y),(0 + x*robot_size , 0 + y*robot_size),(robot_size + x*robot_size , robot_size + y*robot_size),False)
            box_list.append(box)

    #init and associate the values
    terget_pose = target
    initial_pos = robot_pose
    target_box = start_box = Box(00,(0,0),(2,2),False)

    #calculate the cost
    for box in box_list:
        if not box.get_state():
            #distance to target of each box with a gain of 5
            d = distance_to_target(box.get_centre(),terget_pose)*5
            d_obstacle = 0
            #add bigger cost to obstacles neighbor boxes 
            for obstacle in obstacle_box:
                if distance_to_target(box.get_centre(),obstacle.get_centre())  < distance_to_target((0,0),(robot_size,robot_size)):
                    d_obstacle += distance_to_target((0,0),(robot_size,robot_size))*5
            #set the cost calculated to the box
            box.set_cost(d+d_obstacle)
            cv2.putText(img, str(round(box.get_cost(),2)), box.get_centre(), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 0))
        #if the target position is in the box save the box as target box
        if box.Pixel_in_box(terget_pose):
            target_box = box
            cv2.circle(img, terget_pose , int(robot_size/2 - 20), (255 , 0 , 255), 5)
            cv2.putText(img, "target", terget_pose, cv2.FONT_HERSHEY_PLAIN, 1, (0, 255, 255))

        #if the initial position is in the box save the box as start_box
        elif box.Pixel_in_box(initial_pos):
            start_box = box
            cv2.circle(img, initial_pos , int(robot_size/2 - 20), (255 , 0 , 0), 5)
            cv2.putText(img, "initial", initial_pos, cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255))

    #calculate the path
    path =[start_box]
    next_box = path[-1]
    indice_box = 0
    while True:
        cost = 10000000000000000000
        for box1 in box_list:
            #ignore if the box has obstacle
            if box1.get_state():
                continue
            #ignore if the box is not a neighbor box from the last node
            elif distance_to_target(box1.get_centre(),path[-1].get_centre()) > distance_to_target((0,0),(robot_size,robot_size)):
                continue
            else :
                
                #ignore if the box already visited
                if box1.get_cost() == 100000000:
                    print("ignore")
                    continue
                #get the box with the smallest cost
                elif box1.get_cost() < cost and box1 not in path:
                    cost = box1.get_cost()
                    next_box = box1
                    indice_box =  box_list.index(box1)
                    print("nex box ", next_box.get_number(), box1.get_cost())
        #when arrived to the end
        if next_box == target_box:
            print("box target",next_box.get_number())
            path.append(next_box)
            break
        #if the next box is already on the path and the path is not just the start box
        elif next_box in path and len(path) > 1:
            box_list[indice_box].set_cost(100000000) #set this box cost to 100000000
            cv2.putText(img, str(round(box_list[indice_box].get_cost(),2)), box_list[indice_box].get_centre(), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 255))
            path.remove(next_box) #remove this box from the path, because it's a closed path
            
            if path[-1] != start_box: #increase the last node also and remove it from the path
                box_list[box_list.index(path[-1])].set_cost(100000000)
                cv2.putText(img, str(round(box_list[box_list.index(path[-1])].get_cost(),2)), box_list[box_list.index(path[-1])].get_centre(), cv2.FONT_HERSHEY_COMPLEX, 0.5, (0, 0, 255))
                path.remove(path[-1])

            continue
        elif next_box not in path:
            path.append(next_box)
            print("box in the path",next_box.get_number())
        #if we arrived to the target
        if distance_to_target(next_box.get_centre(),target_box.get_centre()) < distance_to_target((0,0),(0,robot_size)):
            break

    #compute the final path with the poitns positions
    final_path =[]
    for box in path:
        if box == start_box:#because we take the initial position as a start point
            continue
        elif box == target_box:
            final_path.append(target)
        else:
            cv2.circle(img, box.get_centre() , int(robot_size/2 - 20), (255 , 0 , 0), 5)
            final_path.append(box.get_centre())
    

    return final_path, img

#this class is used to calculate the path
class Box:
    def __init__(self,number, pose_min,pose_max,contain_obstacle):
        self.number = number
        self.pose_min = pose_min
        self.pose_max = pose_max
        self.contain_obstacle = contain_obstacle
        self.centre = (int((pose_min[0]+pose_max[0])/2),int((pose_min[1]+pose_max[1])/2))
        self.cost = 10000000000

    def get_state(self):
        return self.contain_obstacle
    def set_cost(self,cost):
        self.cost = cost
    def get_cost(self):
        return self.cost 
    def get_number(self):
        return self.number
    def get_borders(self):
        return (self.pose_min,self.pose_max)
    def get_centre(self):
        return self.centre
    #if the oixel in parameters is in the box or not
    def Pixel_in_box(self,pixel):
        if (pixel[0] in range(self.pose_min[0],self.pose_max[0])and (pixel[1] in range(self.pose_min[1],self.pose_max[1]))):
            return True 
        else:
            return False

class Robot:
    def __init__(self,name,max_speed, max_rot_speed ,init_pos):
        self.__x = init_pos[0]
        self.__y = init_pos[1]
        self.__theta = init_pos[2]* math.pi/180
        self.__name = name
        self.__max_speed = max_speed
        self.__max_rot_speed = max_rot_speed
        self.__forward_speed = 0.1
        self.__rotational_speed = 0.0
        self.__Dtime = 0.0

    def set_name(self, name):
        self.__name = name
    def get_name(self):
        return self.__name
    def get_position(self):
        return [self.__x, self.__y, self.__theta]
    def set_position(self,x,y,theta):
        self.__x = x
        self.__y = y
        self.__theta = theta* math.pi/180
    def set_dtime(self, dtime):
        self.__Dtime = dtime
    def set_forward_speed(self, forward_speed):
        if (forward_speed > self.__max_speed) :
            self.__forward_speed = self.__max_speed
        else:
            self.__forward_speed = forward_speed
    def set_forward_speed_list(self, forward_speed):
        if (forward_speed[0] > self.__max_speed) :
            x = self.__max_speed
        else:
            x = forward_speed
        if (forward_speed[1] > self.__max_speed) :
            y = self.__max_speed
        else:
            y = forward_speed
        return [x,y]
    def get_forward_speed(self): 
        return self.__forward_speed
    def set_rotational_speed(self, rotational_speed):
        if(rotational_speed > self.__max_rot_speed):
            self.__rotational_speed = self.__max_rot_speed
        elif(rotational_speed < -self.__max_rot_speed):
            self.__rotational_speed = -self.__max_rot_speed
        else:
            self.__rotational_speed = rotational_speed
    def get_rotational_speed(self):
        return self.__rotational_speed

    #will update the position of the robot with the help of forward and rotational speed
    #used in simulation
    def update_position(self): 
        self.__theta = self.__theta + self.get_rotational_speed() * self.__Dtime
        velocity_x = self.get_forward_speed() * math.cos(self.__theta)
        velocity_y = self.get_forward_speed() * math.sin(self.__theta)
        self.__x = self.__x + velocity_x * self.__Dtime
        self.__y = self.__y + velocity_y * self.__Dtime



        



class Controller:
    def __init__(self, forward_speed_gain, alpha_gain, beta_gain):
        self.forward_speed_gain = forward_speed_gain
        self.alpha_gain = alpha_gain
        self.beta_gain = beta_gain

    def robot_path(self, current_position, target_position):
        #calculate distance between robot current position and target position
        difference = [(target_position[0] - current_position[0]), (target_position[1] - current_position[1])]
    
        theta = current_position[2] #the angle of the robot according to the base frame (here image frame)
        
        #the angle between the robot orientation and the direction of the target 
        alpha = math.atan2(  target_position[1] - current_position[1],target_position[0] - current_position[0]) - theta

        #the rotational speed is the multipication of the error by the gain 
        # the negative alpha is because we work in the image frame
        self.rotationalspeed = self.alpha_gain * float(-alpha) 
        print("theta:", theta*(180/math.pi),"\n")
        print("alpha:", alpha*(180/math.pi),"\n")

        #correct the angle and then go , to make a smooth trajectory because the points are very close
        if( abs(alpha)*(180/math.pi) > 30):
            self.forwardspeed = 0
        else:
            a = [difference[0]*self.forward_speed_gain, difference[1]*self.forward_speed_gain]
            self.forwardspeed = math.sqrt((a[0])**2 + (a[1])**2) #resultant of x and y linear components speed
        return (self.forwardspeed , self.rotationalspeed)



    def parking(self, current_position, target_position): #returns the forward and rotational speed
        #calculate distance between robot current position and target position
        difference = [(target_position[0] - current_position[0]), (target_position[1] - current_position[1])]

        #the angle between the orientation of the target and the robot
        theta = current_position[2] - target_position[2]*math.pi/180

        #the angle between the robot orientation and the direction of the target
        alpha = math.atan2(  target_position[1] - current_position[1],target_position[0] - current_position[0]) - current_position[2] 
        beta = -alpha -theta 

        #the rotational speed is the multipication of the error by the gain 
        # the negative alpha is because we work in the image frame
        self.rotationalspeed = self.alpha_gain * float(-alpha) + self.beta_gain*float(-beta) 
        print("Beta :", beta*(180/math.pi) )
        print("theta:", theta*(180/math.pi),"\n")
        print("alpha:", alpha*(180/math.pi),"\n")
        a = [difference[0]*self.forward_speed_gain, difference[1]*self.forward_speed_gain]
        self.forwardspeed = math.sqrt((a[0])**2 + (a[1])**2) #resultant of x and y linear components speed
        return (self.forwardspeed , self.rotationalspeed)

    #correct the angle between the target and the robot
    def correct_parking(self, current_position, target_position): 
        #the angle between the orientation of the target and the robot
        theta = current_position[2] - target_position[2]*math.pi/180  
        self.rotationalspeed = self.alpha_gain * float(theta)
        print("theta:", theta*(180/math.pi),"\n")

        self.forwardspeed = 0 #the robot has already arrived to the target
        return (self.forwardspeed , self.rotationalspeed)

    def simulation(self, current_position, target_position): #returns the forward and rotational speed
        rho =  math.sqrt(((target_position[0] - current_position[0]))**2 + (target_position[1] - current_position[1])**2)

        theta = current_position[2] - target_position[2]*math.pi/180
        alpha =  - current_position[2] + math.atan2(  target_position[1] - current_position[1],target_position[0] - current_position[0])
        beta = -theta - alpha

        #the rotational speed is the multipication of the error by the gain 
        #alpha and beta are positiove here
        self.rotationalspeed = self.alpha_gain * float(alpha) + self.beta_gain * float(beta)
        print("Beta :", beta*(180/math.pi) )
        print("alpha :", alpha*(180/math.pi) )
        print("theta :", theta*(180/math.pi) )

        self.forwardspeed = self.forward_speed_gain* rho #resultant of x and y linear components speed
        return (self.forwardspeed , self.rotationalspeed)









