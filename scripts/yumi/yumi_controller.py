#!/usr/bin/env python3
import rospy
from std_msgs.msg import UInt16MultiArray, String, Int8
from abb import* 
from time import sleep


class RobotNode:
    def __init__(self):
        rospy.init_node("yumi_controller")
        self.yumi_status = 0
        self.mid_box_x = 0
        self.mid_box_y = 0
        self.worktool = ""
        self.orientation = ""

        # Connect to Yumi
        self.robot = YuMi("192.168.4.3")
        self.robot.Connect()
        self.robot.InitGrippers()

        # Move Yumi home
        """
        print("HomePosition START")
        self.robot.MoveHome()
        self.robot.RightHand.Calibrate()
        self.robot.LeftHand.Calibrate()
        print(self.robot.RightArm.Read())
        print(self.robot.LeftArm.Read())
        print("HomePosition END")
        """
        
        self.sub_coords = rospy.Subscriber("/object_reg", UInt16MultiArray, self.coords_callback)
        self.sub_target = rospy.Subscriber("/get_object_class", String, self.orientation_callback)
        self.sub_termite = rospy.Subscriber("/voice_reg", String, self.termite_callback)
        self.pub_move = rospy.Publisher("/yumi_status", Int8, queue_size=10)
        
        rospy.Timer(rospy.Duration(1.0), self.timmer_callback)

        #linearMovement
        self.pR = LinearTarget()
        self.pL = LinearTarget()

        # parking position
        self.pR.trans = Position(135, -245, 100)
        self.pL.trans = Position(140, 290, 100)
    
        self.pR.rot = EulerAngles(pi, 0, -pi).toQuaternion()
        self.pL.rot = EulerAngles(pi, 0, 0).toQuaternion()

        self.pR.robconf = Conf(1, -2, 2, 4)
        self.pL.robconf = Conf(-1, 2, -2, 4)

        self.pR.extax[0] = -82.6
        self.pL.extax[0] = 82.6
    
        self.pR.speed = self.pL.speed = Speed(500)
        self.pR.zone = self.pL.zone = Zone(-1)

        self.robot.RightArm.MoveTo(self.pR)
        self.robot.LeftArm.MoveTo(self.pL)
        print(self.robot.RightArm.Read())
        print(self.robot.LeftArm.Read())

        self.robot.RightHand.MoveTo(25.0)
        self.robot.LeftHand.MoveTo(25.0)
        

        rospy.loginfo("YuMi is in parking position")

    # Definition of x,y coords and orientation of object
    def coords_callback(self, msg: UInt16MultiArray):
        self.mid_box_x = msg.data[0]
        self.mid_box_y = msg.data[1]

    def orientation_callback(self, msg: String):
        list_of_object =  msg.data.split("/")
        self.worktool = list_of_object[0]
        self.orientation = list_of_object[1]
        rospy.loginfo(self.worktool)

    def termite_callback(self, msg: String):
        received_bytes = msg.data.encode('utf-8')
        decoded_text = received_bytes.decode('utf-8')
        if decoded_text == "ukonči prevádzku":
            rospy.signal_shutdown("Ukončenie programu!")

    def timmer_callback(self, msg):
        # Calculate coords from res
        x,y = calculate_grasp_position(self.mid_box_x, self.mid_box_y)
        if self.worktool != "":
            self.yumi_status = 1
            self.pub_move.publish(self.yumi_status)
            """
            LEFT ARM
            """
            if y in range(0,290):
                # sending robot to detected position
                self.pL.trans.x = x
                self.pL.trans.y = y
                self.pL.trans.z = 30
                count = 0
                while self.robot.LeftArm.MoveTo(self.pL) != True:
                    self.robot.LeftArm.MoveTo(self.pL)
                    count +=1
                    if count >=5 : break

                # grasping postion
                grip = drop = 0
                try:
                    grip,drop = get_orientation(self.orientation, arm="left")
                except:
                    print ("orientation or chosen worktool is none")
                print(grip,drop)
                self.pL.rot = EulerAngles(pi, 0, grip ).toQuaternion()

                count = 0
                while self.robot.LeftArm.MoveTo(self.pL) != True:
                    self.robot.LeftArm.MoveTo(self.pL)
                    count +=1
                    if count >=5 : break 

                self.pL.trans.z = 1

                count = 0
                while self.robot.LeftArm.MoveTo(self.pL) != True:
                    self.robot.LeftArm.MoveTo(self.pL)
                    count +=1
                    if count >=5 : break

                # grip worktool
                self.robot.LeftHand.GripSmart(10.0, 6.0, 20.0) # force 10, distance between fingers 6mm tolerance 20mm
                sleep(0.5)
                #handling position
                self.pL.trans.z = 60
                self.pL.rot = EulerAngles(pi, 0, drop ).toQuaternion()

                count = 0
                while self.robot.LeftArm.MoveTo(self.pL) != True:
                    self.robot.LeftArm.MoveTo(self.pL)
                    count +=1
                    if count >=5 : break 

                self.pL.trans.x = 600
                self.pL.trans.y = 0
                self.pL.trans.z = 200

                count = 0
                while self.robot.LeftArm.MoveTo(self.pL) != True:
                    self.robot.LeftArm.MoveTo(self.pL)
                    count +=1
                    if count >=5 : break

                self.robot.LeftHand.MoveTo(25.0)
            
            """
            RIGHT ARM
            """
            if y in range(-245,0):
                # sending robot to detected position
                self.pR.trans.x = x
                self.pR.trans.y = y
                self.pR.trans.z = 30
                count = 0
                while self.robot.RightArm.MoveTo(self.pR) != True:
                    self.robot.RightArm.MoveTo(self.pR)
                    count +=1
                    if count >=5 : break

                # grasping postion
                grip = drop = 0
                try:
                    grip,drop = get_orientation(self.orientation, arm="right")
                except:
                        print ("orientation or chosen worktool is none")
                print(grip,drop)
                self.pR.rot = EulerAngles(pi, 0, grip ).toQuaternion()

                count = 0
                while self.robot.RightArm.MoveTo(self.pR) != True:
                    self.robot.RightArm.MoveTo(self.pR)
                    count +=1
                    if count >=5 : break 

                self.pR.trans.z = 1
                
                count = 0
                while self.robot.RightArm.MoveTo(self.pR) != True:
                    self.robot.RightArm.MoveTo(self.pR)
                    count +=1
                    if count >=5 : break

                # grip worktool
                self.robot.RightHand.GripSmart(10.0, 6.0, 20.0) # force 10, distance between fingers 6mm tolerance 20mm
                sleep(0.5)
                #handling position
                self.pR.trans.z = 60
                self.pR.rot = EulerAngles(pi, 0, drop ).toQuaternion()

                count = 0
                while self.robot.RightArm.MoveTo(self.pR) != True:
                    self.robot.RightArm.MoveTo(self.pR)
                    count +=1
                    if count >=5 : break 
                # Pick_drop possition
                self.pR.trans.x = 600
                self.pR.trans.y = 0
                self.pR.trans.z = 200

                count = 0
                while self.robot.RightArm.MoveTo(self.pR) != True:
                    self.robot.RightArm.MoveTo(self.pR)
                    count +=1
                    if count >=5 : break

                self.robot.RightHand.MoveTo(25.0)

        # Set up default values
        self.yumi_status = 0
        self.mid_box_x = 0
        self.mid_box_y = 0
        self.worktool = ""
        self.orientation = ""

        # parking position
        self.pR.trans = Position(135, -245, 100)
        self.pL.trans = Position(140, 290, 100)
        self.pR.rot = EulerAngles(pi, 0, -pi).toQuaternion()
        self.pL.rot = EulerAngles(pi, 0, 0).toQuaternion()
        self.robot.LeftArm.MoveTo(self.pL)
        self.robot.RightArm.MoveTo(self.pR)

        self.pub_move.publish(self.yumi_status)


# Get robot_coords
def calculate_grasp_position(u,v):
    grasp_y = rescale(u,0,640,290,-245)
    grasp_x = rescale(v,0,480,510,120)
    return int(grasp_x),int(grasp_y)

# Recalculation of res_coods to robot_coords
def rescale(x, old_min, old_max, new_min, new_max):
  if old_min == old_max:
    raise ValueError("Old range cannot be empty.")
  if new_min == new_max:
    raise ValueError("New range cannot be empty.")
  # Normalize x to the old range
  normalized_x = (x - old_min) / (old_max - old_min)
  # Rescale to the new range
  rescaled_x = normalized_x * (new_max - new_min) + new_min
  return rescaled_x


def get_orientation(orientation, arm):
        if arm == "left":
            if orientation == "up":
                grip_rot_pick = -pi/2
                grip_rot_drop = -pi/2
                return grip_rot_pick, grip_rot_drop  

            if orientation == "right":
                grip_rot_pick = 0
                grip_rot_drop = -pi/2
                return grip_rot_pick, grip_rot_drop  
                
        elif arm == "right":
            if orientation == "up":
                grip_rot_pick = -3*pi/2
                grip_rot_drop = -3*pi/2
                return grip_rot_pick, grip_rot_drop  
                
            if orientation == "right":
                grip_rot_pick = -pi
                grip_rot_drop = -3*pi/2
                return grip_rot_pick, grip_rot_drop

        else:
            rospy.logwarn("Unknown orientation") 


def main():  
    # Create subscriber
    rospy.init_node("yumi_controller")
    RobotNode()
    rospy.spin()

if __name__ == "__main__":
    main()