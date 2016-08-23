#!/usr/bin/env python

"""
    patrol_tree.py - Version 1.0 2013-03-18
    
    Navigate a series of waypoints while monitoring battery levels.
    Uses the pi_trees package to implement a behavior tree task manager.
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2013 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
"""

import rospy
import time
from std_msgs.msg import Int32, Bool
from geometry_msgs.msg import Twist, Pose, PoseStamped
from std_msgs.msg import String
from kobuki_msgs.msg import Sound
from apriltags_nodelet.msg import AprilTagDetections, AprilTagDetection
from pi_trees_ros.pi_trees_ros import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionFeedback
from time import sleep

class BlackBoard():
    def __init__(self):	
        self.battery_level = None
        self.charging = True
        self.rate = 10
        # timestamp of the last time the wake word was heard
        self.command_time = 0
        self.wake_word = "robot"
        self.target_id = -1
        self.target_goal = None
        self.pub_beep_  = rospy.Publisher("/mobile_base/commands/sound", Sound, queue_size=2)
        self.docked = True

class Patrol():
    def __init__(self):
        rospy.init_node("behavior_tree")

        # Set the shutdown function (stop the robot)
        rospy.on_shutdown(self.shutdown)
        
        # Initialize the black board
        self.blackboard = BlackBoard()
        
        # How frequently do we "tic" the tree?
        self.blackboard.rate = rospy.get_param('~rate', 10)
        
        # Convert tic rate to a ROS rate
        tic = rospy.Rate(self.blackboard.rate)
        
        # Where should the DOT file be stored.  Default location is $HOME/.ros/tree.dot
        dotfilepath = rospy.get_param('~dotfilepath', None)

        # Create the root node
        BEHAVE = Sequence("BEHAVE", reset_after=True)
        

        ACTIONS = Selector("ACTIONS")

        LISTEN = MonitorTask("LISTEN", "/recognizer/output", String, self.listen)
        BEHAVE.add_child(LISTEN)
        BEHAVE.add_child(ACTIONS)

        FETCH = Sequence("FETCH", reset_after=True)
        HOME = Sequence("HOME", reset_after=False)
        TRASH = Sequence("TRASH", reset_after=False)
        WATER = Sequence("WATER", reset_after=False)
        
        # Add the two subtrees to the root node in order of priority
        ACTIONS.add_child(HOME)
        ACTIONS.add_child(TRASH)
        ACTIONS.add_child(WATER)
        ACTIONS.add_child(FETCH)

        with HOME:
            CHECK_HOME_DOCK = BlackboardTask("CHECK_HOME_DOCK", self.blackboard, "docked", False)
            CHECK_HOME = BlackboardTask("CHECK_HOME", self.blackboard, "target_id", 0)
            FIND_HOME = MonitorOnceTask("FIND_HOME", "/apriltags/detections_target", AprilTagDetections, self.apriltag_detections)
            HOME_GRIP = PublishTask("HOME_GRIP", "/grip", Int32, 60)
            HOME_ARM = PublishTask("HOME_ARM", "/arm", Int32, 400)
            NAV_HOME_TARGET =  DynamicActionTask("NAV_HOME_TARGET", "planner/move_base", MoveBaseAction, self.blackboard, rate=self.blackboard.rate, result_timeout=120, reset_after=False)
            
            dock_pose = PoseStamped()
            dock_pose.header.frame_id = '0'
            dock_pose.header.stamp = rospy.Time.now()
            p = Pose()
            p.position.x = -0.02
            p.position.y = 0.27
            dock_pose.pose = p

            DOCK_TARGET = PublishTask("DOCK_TARGET", "apriltags/goal", PoseStamped, dock_pose, 6, done_cb=self.done_dock)
            
            HOME.add_child(CHECK_HOME_DOCK)
            HOME.add_child(CHECK_HOME)
            HOME.add_child(FIND_HOME)
            HOME.add_child(HOME_GRIP)
            HOME.add_child(HOME_ARM)
            HOME.add_child(NAV_HOME_TARGET)
            HOME.add_child(DOCK_TARGET)

        # backup 0.6 meter if still docked
        UNDOCK = Selector("UNDOCK")
        CHECK_DOCK = BlackboardTask("CHECK_DOCK", self.blackboard, "docked", False)
        UNDOCK_ACTION = Sequence("UNDOCK_ACTION")

        UNDOCK.add_child(CHECK_DOCK)
        UNDOCK.add_child(UNDOCK_ACTION)

        CLOSE_GRIP = PublishTask("CLOSE_GRIP", "/grip", Int32, 60, 2)
        LOWER_ARM = PublishTask("LOWER_ARM", "/arm", Int32, 80)
        backup = Twist()
        backup.linear.x = -0.13; backup.linear.y = 0; backup.linear.z = 0;
        backup.angular.x = 0; backup.angular.y = 0; backup.angular.z = 0;
        UNDOCK_BACKUP = TwistTask("UNDOCK_BACKUP", "/mobile_base/commands/velocity", backup, 4, rate=self.blackboard.rate, done_cb=self.done_undock)

        UNDOCK_ACTION.add_child(CLOSE_GRIP)
        UNDOCK_ACTION.add_child(UNDOCK_BACKUP)
        UNDOCK_ACTION.add_child(LOWER_ARM)

        home_pose = Pose()
        home_pose.position.x = -0.8
        home_pose.position.y = 0.0
        home_pose.position.z = 0.0
        home_pose.orientation.z = 0.678491842963
        home_pose.orientation.w = 0.734607935591

        home_goal = MoveBaseGoal()
        home_goal.target_pose.header.frame_id = 'map'
        home_goal.target_pose.header.stamp = rospy.Time.now()
        home_goal.target_pose.pose = home_pose

        with TRASH:
            CHECK_TRASH = BlackboardTask("CHECK_TRASH", self.blackboard, "target_id", 2)
            FIND_TRASH = MonitorOnceTask("FIND_TRASH", "/apriltags/detections_target", AprilTagDetections, self.apriltag_detections)
            PRESENT = SimpleActionTask("PRESENT", "planner/move_base", MoveBaseAction, home_goal, reset_after=False)
            OPEN_GRIP_TRASH = PublishTask("OPEN_GRIP_TRASH", "/grip", Int32, 30)
            RAISE_ARM_TRASH = PublishTask("RAISE_ARM_TRASH", "/arm", Int32, 900, 4)
            DETECT_BOOL_RESET_TRASH = PublishTask("DETECT_BOOL_RESET_TRASH", "/active", Bool, False, 0.5)
            DETECT_BOOL_TRASH = PublishTask("DETECT_BOOL_TRASH", "/active", Bool, True)
            DETECT_TRASH = MonitorOnceTask("DETECT_TRASH", "/detected", Bool, self.detect_hand)
            CLOSE_GRIP_TRASH = PublishTask("CLOSE_GRIP_TRASH", "/grip", Int32, 75, 1)
            LOWER_ARM_TRASH = PublishTask("LOWER_ARM_TRASH", "/arm", Int32, 80)
            NAV_TRASH_TASK = DynamicActionTask("NAV_TRASH_TASK", "planner/move_base", MoveBaseAction, self.blackboard, rate=self.blackboard.rate, result_timeout=120, reset_after=False)
            RAISE_ARM_TRASH2 = PublishTask("RAISE_ARM_TRASH2", "/arm", Int32, 700, 3.5)

            trash_pose = PoseStamped()
            trash_pose.header.frame_id = '2'
            trash_pose.header.stamp = rospy.Time.now()
            p = Pose()
            p.position.x = 0.12
            p.position.y = 0.22
            trash_pose.pose = p

            TRASH_TARGET = PublishTask("TRASH_TARGET", "apriltags/goal", PoseStamped, trash_pose, 6)
            OPEN_GRIP_TRASH2 = PublishTask("OPEN_GRIP_TRASH2", "/grip", Int32, 20)
            TRASH_BACK = TwistTask("TWIST", "/mobile_base/commands/velocity", backup, 3, rate=self.blackboard.rate)
            LOWER_ARM_TRASH2 = PublishTask("LOWER_ARM_TRASH2", "/arm", Int32, 80)
            TRASH_HOME_TASK = SimpleActionTask("TRASH_HOME_TASK", "planner/move_base", MoveBaseAction, home_goal, done_cb=self.reset_fetch, reset_after=False)
            
            TRASH.add_child(CHECK_TRASH)
            TRASH.add_child(FIND_TRASH)
            TRASH.add_child(UNDOCK)
            TRASH.add_child(PRESENT)
            TRASH.add_child(OPEN_GRIP_TRASH)
            TRASH.add_child(RAISE_ARM_TRASH)
            TRASH.add_child(DETECT_BOOL_RESET_TRASH)
            TRASH.add_child(DETECT_BOOL_TRASH)
            TRASH.add_child(DETECT_TRASH)
            TRASH.add_child(CLOSE_GRIP_TRASH)
            TRASH.add_child(LOWER_ARM_TRASH)
            TRASH.add_child(NAV_TRASH_TASK)
            TRASH.add_child(RAISE_ARM_TRASH2)
            TRASH.add_child(TRASH_TARGET)
            TRASH.add_child(OPEN_GRIP_TRASH2)
            TRASH.add_child(TRASH_BACK)
            TRASH.add_child(LOWER_ARM_TRASH2)
            TRASH.add_child(TRASH_HOME_TASK)

        with WATER:
            CHECK_WATER = Selector("CHECK_WATER")
            CHECK_WATER3 = BlackboardTask("CHECK_WATER3", self.blackboard, "target_id", 3)
            CHECK_WATER4 = BlackboardTask("CHECK_WATER4", self.blackboard, "target_id", 4)
            CHECK_WATER.add_child(CHECK_WATER3)
            CHECK_WATER.add_child(CHECK_WATER4)

            FIND_WATER = MonitorOnceTask("FIND_WATER", "/apriltags/detections_target", AprilTagDetections, self.apriltag_detections)
            NAV_WATER_TASK = DynamicActionTask("NAV_WATER_TASK", "planner/move_base", MoveBaseAction, self.blackboard, rate=self.blackboard.rate, result_timeout=120, reset_after=False)

            WATER_ARM1 = PublishTask("WATER_ARM1", "/arm", Int32, 700)
            WATER_GRIP1 = PublishTask("WATER_GRIP1", "/grip", Int32, 35, 3)
            water_pose = PoseStamped()
            water_pose.header.frame_id = '3'
            water_pose.header.stamp = rospy.Time.now()
            p = Pose()
            p.position.x = 0.10
            p.position.y = 0.32
            water_pose.pose = p

            WATER_TARGET = PublishTask("WATER_TARGET", "apriltags/goal", PoseStamped, water_pose, 8)
            WATER_GRIP2 = PublishTask("WATER_GRIP2", "/grip", Int32, 55,1)
            WATER_ARM2 = PublishTask("WATER_ARM2", "/arm", Int32, 850, 1.5)
            WATER_BACKUP = TwistTask("WATER_BACKUP", "/mobile_base/commands/velocity", backup, 4, rate=self.blackboard.rate)
            WATER_ARM3 = PublishTask("WATER_ARM3", "/arm", Int32, 100, 1, done_cb=self.target_water)

            FIND_WATER2 = MonitorOnceTask("FIND_WATER2", "/apriltags/detections_target", AprilTagDetections, self.apriltag_detections)
            NAV_WATER_TASK2 = DynamicActionTask("NAV_WATER_TASK2", "planner/move_base", MoveBaseAction, self.blackboard, rate=self.blackboard.rate, result_timeout=120, reset_after=False)

            water_pose2 = PoseStamped()
            water_pose2.header.frame_id = '4'
            water_pose2.header.stamp = rospy.Time.now()
            p = Pose()
            p.position.x = -0.05
            p.position.y = 0.42
            water_pose2.pose = p

            WATER_ARM32 = PublishTask("WATER_ARM32", "/arm", Int32, 380, 2)

            twist = Twist()
            twist.linear.x = 0.0; twist.linear.y = 0; twist.linear.z = 0;
            twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = -0.4;
            WATER_TWIST = TwistTask("WATER_TWIST", "/mobile_base/commands/velocity", twist, 1, rate=self.blackboard.rate)

            WATER_TARGET2 = PublishTask("WATER_TARGET2", "apriltags/goal", PoseStamped, water_pose2, 8)
            WATER_ARM35 = PublishTask("WATER_ARM35", "/arm", Int32, 340, 2)

            WATER_GRIP3 = PublishTask("WATER_GRIP3", "/grip", Int32, 30,1)

            WATER_BACKUP15 = TwistTask("WATER_BACKUP15", "/mobile_base/commands/velocity", backup, 0.75, rate=self.blackboard.rate)
            WATER_GRIP35 = PublishTask("WATER_GRIP35", "/grip", Int32, 55,1)
            WATER_ARM4 = PublishTask("WATER_ARM4", "/arm", Int32, 710, 10)
            WATER_ARM5 = PublishTask("WATER_ARM5", "/arm", Int32, 350, 2)
            WATER_GRIP38 = PublishTask("WATER_GRIP38", "/grip", Int32, 40,1)

            WATER_TARGET25 = PublishTask("WATER_TARGET25", "apriltags/goal", PoseStamped, water_pose2, 4)

            WATER_GRIP4 = PublishTask("WATER_GRIP4", "/grip", Int32, 65,1)
            WATER_BACKUP2 = TwistTask("WATER_BACKUP2", "/mobile_base/commands/velocity", backup, 3, rate=self.blackboard.rate)
            WATER_ARM5B = PublishTask("WATER_ARM5B", "/arm", Int32, 100, 2)
            WATER_HOME_TASK = SimpleActionTask("WATER_HOME_TASK", "planner/move_base", MoveBaseAction, home_goal, reset_after=False)
            WATER_ARM6 = PublishTask("WATER_ARM6", "/arm", Int32, 50, 2)
            WATER_GRIP5 = PublishTask("WATER_GRIP5", "/grip", Int32, 30, 1)
            WATER_GRIP6 = PublishTask("WATER_GRIP6", "/grip", Int32, 10, 1)
            WATER_BACKUP3 = TwistTask("WATER_BACKUP3", "/mobile_base/commands/velocity", backup, 2, rate=self.blackboard.rate, done_cb=self.reset_fetch)

            WATER.add_child(CHECK_WATER)
            WATER.add_child(FIND_WATER)
            WATER.add_child(UNDOCK)
            WATER.add_child(NAV_WATER_TASK)
            WATER.add_child(WATER_ARM1)
            WATER.add_child(WATER_GRIP1)
            WATER.add_child(WATER_TARGET)
            WATER.add_child(WATER_GRIP2)
            WATER.add_child(WATER_ARM2)
            WATER.add_child(WATER_BACKUP)
            WATER.add_child(WATER_ARM3)

            WATER.add_child(FIND_WATER2)
            WATER.add_child(NAV_WATER_TASK2)
            WATER.add_child(WATER_ARM32)
            WATER.add_child(WATER_TWIST)
            WATER.add_child(WATER_TARGET2)
            WATER.add_child(WATER_ARM35)
            WATER.add_child(WATER_GRIP3)
            WATER.add_child(WATER_BACKUP15)

            WATER.add_child(WATER_GRIP35)
            WATER.add_child(WATER_ARM4)
            WATER.add_child(WATER_ARM5)
            WATER.add_child(WATER_GRIP38)
            WATER.add_child(WATER_TARGET25)
            WATER.add_child(WATER_GRIP4)
            WATER.add_child(WATER_BACKUP2)
            WATER.add_child(WATER_ARM5B)
            WATER.add_child(WATER_HOME_TASK)
            WATER.add_child(WATER_ARM6)
            WATER.add_child(WATER_GRIP5)
            WATER.add_child(WATER_GRIP6)
            WATER.add_child(WATER_BACKUP3)

        with FETCH:

            FIND_WAYPOINT = MonitorOnceTask("FIND_WAYPOINT", "/apriltags/detections_target", AprilTagDetections, self.apriltag_detections)
        
            NAV_FETCH_TASK = DynamicActionTask("NAV_FETCH_TASK", "planner/move_base", MoveBaseAction, self.blackboard, rate=self.blackboard.rate, result_timeout=120, reset_after=False)
            
            RAISE_ARM = PublishTask("RAISE_ARM", "/arm", Int32, 780)
            OPEN_GRIP = PublishTask("OPEN_GRIP", "/grip", Int32, 10, 3)

            # ensure required apriltag is within view
            APRILTAG_CHECK = MonitorOnceTask("APRILTAG_CHECK", "apriltags/detections", AprilTagDetections, self.apriltag_check)

            # move robot
            target_pose = PoseStamped()
            target_pose.header.frame_id = '1'
            target_pose.header.stamp = rospy.Time.now()
            p = Pose()
            p.position.x = 0.12
            p.position.y = 0.32
            p.position.z = 0.0
            p.orientation.w = 1.0
            target_pose.pose = p

            APRIL_TARGET = PublishTask("APRIL_TARGET", "apriltags/goal", PoseStamped, target_pose, 7.5)

            APRIL_GRIP = PublishTask("APRIL_GRIP", "grip", Int32, 70, 1)

            RAISE_ARM2 = PublishTask("RAISE_ARM2", "/arm", Int32, 900, 1)
            APRIL_BACK = TwistTask("TWIST", "/mobile_base/commands/velocity", backup, 3, rate=self.blackboard.rate)
            LOWER_ARM2 = PublishTask("LOWER_ARM2", "/arm", Int32, 80)

            NAV_HOME_TASK = SimpleActionTask("NAV_HOME_TASK", "planner/move_base", MoveBaseAction, home_goal, reset_after=False)

            RAISE_ARM3 = PublishTask("RAISE_ARM3", "/arm", Int32, 900, 5)
            DETECT_BOOL = PublishTask("DETECT_BOOL", "/active", Bool, True)
            DETECT_HAND = MonitorOnceTask("DETECT_HAND", "/detected", Bool, self.detect_hand)
            OPEN_GRIP2 = PublishTask("OPEN_GRIP2", "/grip", Int32, 20, 5) # delay for longer
            DETECT_BOOL_RESET = PublishTask("DETECT_BOOL_RESET", "/active", Bool, False, 0.5)
            DETECT_BOOL2 = PublishTask("DETECT_BOOL2", "/active", Bool, True, 0.5)
            DETECT_HAND2 = MonitorOnceTask("DETECT_HAND2", "/detected", Bool, self.detect_hand)
            CLOSE_GRIP2 = PublishTask("CLOSE_GRIP2", "/grip", Int32, 70, 2)
            LOWER_ARM3 = PublishTask("LOWER_ARM3", "/arm", Int32, 80)
            NAV_FETCH_TASK2 = DynamicActionTask("NAV_FETCH_TASK2", "planner/move_base", MoveBaseAction, self.blackboard, rate=self.blackboard.rate, result_timeout=120, reset_after=False)
            RAISE_ARM4 = PublishTask("RAISE_ARM4", "/arm", Int32, 900, 3)
            APRIL_TARGET2 = PublishTask("APRIL_TARGET", "apriltags/goal", PoseStamped, target_pose, 7.5)
            LOWER_ARM4 = PublishTask("LOWER_ARM4", "/arm", Int32, 800, 1)
            OPEN_GRIP3 = PublishTask("OPEN_GRIP3", "/grip", Int32, 30, 1)
            APRIL_BACK2 = TwistTask("TWIST", "/mobile_base/commands/velocity", backup, 3, rate=self.blackboard.rate)
            LOWER_ARM5 = PublishTask("LOWER_ARM5", "/arm", Int32, 80)
            NAV_HOME_TASK2 = SimpleActionTask("NAV_HOME_TASK2", "planner/move_base", MoveBaseAction, home_goal, done_cb=self.reset_fetch, reset_after=False)
            
            FETCH.add_child(FIND_WAYPOINT)
            FETCH.add_child(UNDOCK)
            FETCH.add_child(NAV_FETCH_TASK)
            FETCH.add_child(RAISE_ARM)
            FETCH.add_child(OPEN_GRIP)
            FETCH.add_child(APRIL_TARGET)
            FETCH.add_child(APRIL_GRIP)
            FETCH.add_child(RAISE_ARM2)
            FETCH.add_child(APRIL_BACK)
            FETCH.add_child(LOWER_ARM2)
            FETCH.add_child(NAV_HOME_TASK)
            FETCH.add_child(RAISE_ARM3)
            FETCH.add_child(DETECT_BOOL)
            FETCH.add_child(DETECT_HAND)
            FETCH.add_child(OPEN_GRIP2)
            FETCH.add_child(DETECT_BOOL2)
            FETCH.add_child(DETECT_HAND2)
            FETCH.add_child(CLOSE_GRIP2)
            FETCH.add_child(LOWER_ARM3)
            FETCH.add_child(NAV_FETCH_TASK2)
            FETCH.add_child(RAISE_ARM4)
            FETCH.add_child(APRIL_TARGET2)
            FETCH.add_child(LOWER_ARM4)
            FETCH.add_child(OPEN_GRIP3)
            FETCH.add_child(APRIL_BACK2)
            FETCH.add_child(LOWER_ARM5)
            FETCH.add_child(NAV_HOME_TASK2)

        # Display the tree before beginning execution
        print "Patrol Behavior Tree"
        print_tree(BEHAVE)
        
        # Run the tree
        while not rospy.is_shutdown():
            BEHAVE.status = BEHAVE.run()
            tic.sleep()
            print_dot_tree(BEHAVE, dotfilepath)

    def done_dock(self):
        self.blackboard.docked = True

        self.blackboard.command_time = 0
        self.blackboard.target_id = -1
        self.blackboard.target_goal = None

    def done_undock(self):
        self.blackboard.docked = False
    def reset_fetch(self, result_state=None, result=None):
        rospy.loginfo("FETCH task reset")
        self.blackboard.command_time = 0
        self.blackboard.target_id = -1
        self.blackboard.target_goal = None
    def listen(self, msg):
        if self.blackboard.target_id > -1:
            return TaskStatus.SUCCESS
        rospy.loginfo("Listening: " + msg.data)
        now = int(round(time.time()))
        if msg.data.find(self.blackboard.wake_word) > -1:
            rospy.loginfo("Commanded!")
            self.blackboard.command_time = now
            self.blackboard.pub_beep_.publish(Sound(Sound.ON))
            return TaskStatus.RUNNING
        elif now - self.blackboard.command_time < 5:
            if msg.data.find("resistor") > -1:
                self.blackboard.target_id = 1
                self.blackboard.pub_beep_.publish(Sound(Sound.OFF))
                return TaskStatus.SUCCESS
            elif msg.data.find("home") > -1:
                self.blackboard.target_id = 0
                self.blackboard.pub_beep_.publish(Sound(Sound.OFF))
                return TaskStatus.SUCCESS
            elif msg.data.find("trash") > -1:
                self.blackboard.target_id = 2
                self.blackboard.pub_beep_.publish(Sound(Sound.OFF))
                return TaskStatus.SUCCESS
            elif msg.data.find("water") > -1:
                self.blackboard.target_id = 3
                self.blackboard.pub_beep_.publish(Sound(Sound.OFF))
                return TaskStatus.SUCCESS
            else:
                return TaskStatus.FAILURE
        else:
            return TaskStatus.FAILURE
    def target_water(self):
        self.blackboard.target_id = 4
    def apriltag_detections(self, msg):
        if self.blackboard.target_id < 0:
            return TaskStatus.FAILURE

        for d in msg.detections:
            if d.id == self.blackboard.target_id:
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = "map"
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose = d.pose
                self.blackboard.target_goal = goal
                return TaskStatus.SUCCESS

        return TaskStatus.RUNNING
    def apriltag_check(self, msg):
        for d in msg.detections:
            if d.id == self.blackboard.target_id and d.tag_size > 0.01 and d.tag_size < 0.5:
                return True
        return False
    def monitor_battery(self, msg):
        # Store the battery level as published on the fake battery level topic
        self.blackboard.battery_level = msg.data
        return True
    def detect_hand(self, msg):
        rospy.loginfo("OMG "+str(msg.data))
        if msg.data:
            return TaskStatus.SUCCESS
        return TaskStatus.RUNNING
    
    def shutdown(self):
        rospy.loginfo("Stopping the robot...")
        rospy.sleep(1)
        
class TwistTask(Task):
    """
        issue a given twist command for x number of seconds
    """
    def __init__(self, name, topic, twist, interval, rate=5, done_cb=None):
        super(TwistTask, self).__init__(name)
        
        self.topic = topic
        self.twist = twist
        self.interval = interval
        self.tick = 1.0 / rate
        self.timer = 0
        self.running = False
        self.publisher = rospy.Publisher(self.topic, Twist, latch=True, queue_size=1)
        self.done_cb = done_cb

    def run(self):
        if self.timer == 0:
            rospy.loginfo("Issuing twist")
            rospy.loginfo(self.tick)
            self.publisher.publish(self.twist)

        if self.timer < self.interval:
            self.publisher.publish(self.twist)
            self.running = True
            self.timer += self.tick
            rospy.sleep(self.tick)
            return TaskStatus.RUNNING
        else:
            # stop robot
            if self.running:
                self.publisher.publish(Twist())
                self.running = False
            if self.done_cb is not None:
                self.done_cb()
            return TaskStatus.SUCCESS
    def reset(self):
        self.timer = 0
        self.running = False

class PublishTask(Task):
    """
        Turn a ROS publisher into a Task.
    """
    def __init__(self, name, topic, msg_type, value, delay=0, done_cb=None):
        super(PublishTask, self).__init__(name)
        
        self.topic = topic
        self.msg_type = msg_type
        self.value = value
        self.done = False
        self.done_cb = done_cb
        self.delay = delay
        self.publisher = rospy.Publisher(self.topic, self.msg_type, queue_size=1, latch=True)

    def run(self):
        if self.done:
            return TaskStatus.SUCCESS
        rospy.loginfo("Publishing to topic "+self.topic+" with value "+str(self.value))
        self.publisher.publish(self.value)
        if self.delay > 0:
            sleep(self.delay)
        self.done = True
        if self.done_cb is not None:
            self.done_cb()
        return TaskStatus.SUCCESS
    def reset(self):
        rospy.loginfo("Resetting "+str(self.name))
        self.done = False

class DelayTask(Task):
    """
        This is a *blocking* wait task.  The interval argument is in seconds.
    """
    def __init__(self, name, interval):
        super(DelayTask, self).__init__(name)
        self.done = False
        self.interval = interval
    def run(self):
        if self.done:
            return TaskStatus.SUCCESS

        sleep(self.interval)
        self.done = True

        return TaskStatus.SUCCESS

class BlackboardTask(Task):
    def __init__(self, name, blackboard, variable, value):
        super(BlackboardTask, self).__init__(name)
        self.blackboard = blackboard
        self.variable = variable
        self.value = value
    def run(self):
        if getattr(self.blackboard, self.variable) == self.value:
            return TaskStatus.SUCCESS
        return TaskStatus.FAILURE


# same as the SimpleActionTask except for two implementation changes:
# goal is fetched from the blackboard, so it can be updated dynamically
# goal is re-issued at set intervals, so the global plan can adjust if the tracked goal moves around
class DynamicActionTask(Task):
    """
        Turn a ROS action into a Task.
    """
    def __init__(self, name, action, action_type, blackboard, rate=5, connect_timeout=10, result_timeout=60, reset_after=False, active_cb=None, done_cb=None, feedback_cb=None):
        super(DynamicActionTask, self).__init__(name)
        
        self.action = action
        self.blackboard = blackboard
        self.tick = 1.0 / rate
        self.rate = rospy.Rate(rate)

        self.result = None
        self.connect_timeout = connect_timeout
        self.result_timeout = result_timeout
        self.reset_after = reset_after
        
        self.final_status = None
        
        if done_cb:
            self.user_done_cb = done_cb
        else:
            self.user_done_cb = None
        
        self.done_cb = self.default_done_cb
        
        if active_cb == None:
            active_cb = self.default_active_cb
        self.active_cb = active_cb
        
        if feedback_cb == None:
            feedback_cb = self.default_feedback_cb
        self.feedback_cb = feedback_cb
                
        self.action_started = False
        self.action_finished = False
        self.goal_status_reported = False
        self.time_so_far = 0.0
        
        # Goal state return values
        self.goal_states = ['PENDING', 'ACTIVE', 'PREEMPTED', 
                            'SUCCEEDED', 'ABORTED', 'REJECTED',
                            'PREEMPTING', 'RECALLING', 'RECALLED',
                            'LOST']
        
        self.retry_goal_states = [GoalStatus.PREEMPTED]
            
        rospy.loginfo("Connecting to action " + action)

        # Subscribe to the base action server
        self.action_client = actionlib.SimpleActionClient(action, action_type)

        rospy.loginfo("Waiting for action server...")
        
        # Wait up to timeout seconds for the action server to become available
        try:
            self.action_client.wait_for_server(rospy.Duration(self.connect_timeout))
        except:
            rospy.loginfo("Timed out connecting to the action server " + action)
    
        rospy.loginfo("Connected to action server")

    def run(self):
        # Send the goal
        if not self.action_started:
            rospy.loginfo("Sending " + str(self.name) + " goal to action server")
            self.action_client.send_goal(self.blackboard.target_goal, done_cb=self.done_cb, active_cb=self.active_cb, feedback_cb=self.feedback_cb)
            self.action_started = True
            self.activate_time = rospy.Time.now()
        
        ''' We cannot use the wait_for_result() method here as it will block the entire
            tree so we break it down in time slices of duration 1 / rate.
        '''
        if not self.action_finished:
            self.time_so_far += self.tick
            self.rate.sleep()
            if self.time_so_far > self.result_timeout:
                self.action_client.cancel_goal()
                rospy.loginfo("Timed out achieving goal")
                self.action_finished = True
                return TaskStatus.FAILURE
            else:
                return TaskStatus.RUNNING
        else:
            # Check the final goal status returned by default_done_cb
            if self.goal_status == GoalStatus.SUCCEEDED:
                self.status = TaskStatus.SUCCESS
            
            # This case handles PREEMPTED
            elif self.goal_status in self.retry_goal_states:
                self.status = TaskStatus.RUNNING
                self.action_started = False
                self.action_finished = False
                self.time_so_far = 0
            
            # Otherwise, consider the task to have failed
            else:
                rospy.loginfo("Navigation failure code: "+str(self.goal_status))
                self.status = TaskStatus.FAILURE
            
            # Store the final status before we reset
            self.final_status = self.status

            # Reset the task if the reset_after flag is True
            if self.reset_after:
                self.reset()
        
        self.action_client.wait_for_result(rospy.Duration(10))
        #rospy.loginfo("Final Navigation Status: "+str(self.final_status))
        return self.final_status
                            
    def default_done_cb(self, result_state, result):
        """Goal Done Callback
        This callback resets the active flags and reports the duration of the action.
        Also, if the user has defined a result_cb, it is called here before the
        method returns.
        """
        self.goal_status = result_state
        self.action_finished = True
        
        if not self.goal_status_reported:
            self._duration = rospy.Time.now() - self.activate_time
            
            rospy.loginfo("Action " + self.name + " terminated after "\
                    + str(self._duration.to_sec()) + " seconds with result "\
                    + self.goal_states[self.action_client.get_state()] + ".")
            
            self.goal_status_reported = True
            
        if self.user_done_cb:
            self.user_done_cb(result_state, result)
    
    def default_active_cb(self):
        pass
        
    def default_feedback_cb(self, msg):
        pass
    
    def reset(self):
        rospy.logdebug("RESETTING " + str(self.name))
        self.action_started = False
        self.action_finished = False
        self.goal_status_reported = False
        self.status = self.final_status
        self.time_so_far = 0.0
        super(DynamicActionTask, self).reset()

class MonitorOnceTask(Task):
    """
        Turn a ROS subscriber into a Task.
    """
    def __init__(self, name, topic, msg_type, msg_cb, wait_for_message=False, timeout=5):
        super(MonitorOnceTask, self).__init__(name)
        
        self.topic = topic
        self.msg_type = msg_type
        self.timeout = timeout
        self.msg_cb = msg_cb
        self.done = False
        self.subscriber = None
        rospy.loginfo("Subscribing to topic " + topic)
        
        if wait_for_message:
            try:
                rospy.wait_for_message(topic, msg_type, timeout=self.timeout)
                rospy.loginfo("Connected.")
            except:
                rospy.loginfo("Timed out waiting for " + topic)
                
    def _msg_cb(self, msg):
        self.set_status(self.msg_cb(msg))
    def run(self):
        if self.done:
            return TaskStatus.SUCCESS
        if self.subscriber is None:
            self.subscriber = rospy.Subscriber(self.topic, self.msg_type, self._msg_cb)
        if self.status == TaskStatus.SUCCESS:
            self.done = True
        if self.status is None or self.subscriber is None:
            return TaskStatus.RUNNING
        return self.status
    def reset(self):
        rospy.loginfo("Resetting "+str(self.name))
        if self.subscriber is not None:
            self.subscriber.unregister()
            self.subscriber = None
        self.done = False
        self.status = None

if __name__ == '__main__':
    tree = Patrol()

