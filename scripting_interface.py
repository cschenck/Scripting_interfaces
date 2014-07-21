#!/usr/bin/env python

"""
Copyright 2014 Connor Schenck <connor.schenck@gmail.com>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
"""

"""
The code is orgainized as follows (numbers on left are approximate line numbers):
58 - Direct Baxter Interface Code - code relating to directly interfacing with Baxter himself (other than his screen)
396 - Keyboard & Mouse GUI Code - code relating to the keyboard and mouse GUI using TKinter
771 - On-Robot Interface Code - code relating to the on-robot interface including interfacing with Baxter's screen
1296 - Main function - the main function for the program. Execution starts here.

Sidenote - This should really be split across multiple files.

IMPORTANT: The following global variable, IMAGE_DIRECTORY, needs to be changed to wherever the images assocaited
with this script are. It's global rather than a local reference because running things using rospy makes figuring
out the working directory complicated and I'm not exactly sure what it is.

"""

IMAGE_DIRECTORY = "/home/robolab/proj/baxter/ros_ws/src/connor_code/share/images/"


import argparse
import sys

import roslib
import rospy

import baxter_interface
import baxter_external_devices

import math
import threading
import subprocess
import os
import Queue
from sensor_msgs.msg import Image
from baxter_core_msgs.msg import DigitalIOState, DigitalOutputCommand, ITBStates, ITBState

import sensor_msgs.msg as msgs
import cv2
import cv_bridge

import numpy as np

from Tkinter import *

from collections import namedtuple


############################ Direct Baxter Interface Code #######################################



# This class watches a ROS topic associated with a button on Baxter, which in the RSDK use the
# structure DigitalIO (it basically just says whether a button is up or down). This class can
# either be tested directly (by calling isButtonPressed) or you can set a callback function
# for either up or down pressed that will be called when the state of the button changes.
class DigitalIOMonitor:
    def __init__(self, topic):
        self.state = 0
        self.buttonDownCB = None
        self.buttonUpCB = None
        self.topic = topic
        
        def cb(data):
            self.__callback(data)
        
        rospy.Subscriber(topic, DigitalIOState, cb)
            
    def __callback(self, data):
        oldState = self.state
        self.state = data.state
        
        if oldState != self.state:
            if self.state == 1: #we are now down, so call all the button down callbacks
                if self.buttonDownCB is not None: 
                    self.buttonDownCB(self.topic)
            else: 
                if self.buttonUpCB is not None:
                    self.buttonUpCB(self.topic)
        
    def isButtonPresssed(self):
        return (self.state == 1)
        
    def setButtonDownListener(self, cb):
        self.buttonDownCB = cb
        
    def setButtonUpListener(self, cb):
        self.buttonUpCB = cb

# This class is similar to the above except that it monitors the knobs on the robot rather
# than the buttons. It takes arguments to set the minimum and maximum knob values (e.g., 0
# to 100). Normally the knobs go from 0 to 256 and then roll over back to 0 if the knob
# is spun further, but this class prevents that rollover and holds the value at the maximum
# or minimum without rolling over. The invert argument simply changes which direction increases
# the value and which decreases. This class allows you to directly get the current value
# on the knob, as well as set a callback function that is called whenever that value changes.        
class KnobMonitor:
    def __init__(self, topic, min_val, max_val, invert=False):
        self.wheel = 0
        self.state = 0
        self.changeCB = None
        self.topic = topic
        self.min_val = min_val
        self.max_val = max_val
        self.invert = invert
        
        #rospy.Subscriber(topic, ITBState, self.__callback)
        #this class actually listens to the state of all the knobs on the robot,
        #but it uses the passed topic argument to figure out which knob to
        #pay attention to.
        rospy.Subscriber("/robot/itb_states", ITBStates, self.__callback)
            
    def __callback(self, data):
        
        for i in range(len(data.names)):
            if data.names[i] in self.topic:
                data = data.states[i]
                break
        
        oldWheel = self.wheel
        self.wheel = data.wheel
        
        #check for wraparounds
        if abs(oldWheel - self.wheel) > 200: #arbitrary check, should work though
            if self.wheel < oldWheel: #we were going up, but then wrapped around
                oldWheel = 0
            else: #we were going down, but then wrapped around
                oldWheel = 255 #the max value for the wheel
        
        oldState = self.state
        if self.invert:
            self.state = min(self.max_val, max(self.min_val, oldState - (oldWheel - self.wheel)))
        else:
            self.state = min(self.max_val, max(self.min_val, oldState + (oldWheel - self.wheel)))
        
        if oldState != self.state and self.changeCB is not None:
                self.changeCB(self.topic)
        
    def get(self):
        return self.state
        
    def set(self, val):
        self.state = val
        
    def setChangeListener(self, cb):
        self.changeCB = cb
        
    def setMinVal(self, val):
        self.min_Val = val
        
    def setMaxVal(self, val):
        self.max_val = val


Waypoint = namedtuple("Waypoint", ["name", "ljpos", "rjpos", "lgrip", "rgrip", "vel"])

global_left_grip = None
global_right_grip = None

# This class represnets an ordered list of waypoints. The above 3 global variables are designed for
# use by this class. It abstracts away all the details of working specifically with Baxter and
# allows the user to simply call functions like playback, addwaypoint, or moveToWaypoint (functions
# that begin with __ are designed to be internal to the class). The only external Baxter-related
# variable to this class is the velocity variable, which must be passed to functions that edit or
# add waypoints. This should be a percent (0 - 100), but it does not represent the full velocity
# range for Baxter. For safety reasons, the maximum velocity has been set lower. If you'd like
# to change it, change the MAX_VEL variable below. Waypoints simply save the entire state of the
# robot, including both its arms and both its grippers (although it only considers the grippers
# open or closed).
class Script:

    MAX_VEL = 0.3 #keep in the range (0.0, 1.0]. Default 0.3

    def __init__(self):
        global global_left_grip, global_right_grip
        self.left_arm = baxter_interface.Limb('left')
        self.right_arm = baxter_interface.Limb('right')
        self.left_grip = global_left_grip
        self.right_grip = global_right_grip
        
        self.cancel_move = False
        self.currently_executing_waypoint = -1
        
        self.__waypoint_counter = 0
    
        self.poss = []
        """form of poss: [list of Waypoint]"""
    
    #By default plays back the entire script. Specifying a start or end index will change the
    #start or end point respectively.
    def playback(self, start_index = 0, end_index = -1):
        self.cancel_move = False
        if end_index == -1:
            end_index = len(self.poss)
        i = start_index
        while i < end_index:
            self.currently_executing_waypoint = i
            self.__move(self.poss[i])
            if self.cancel_move:
                break
            i = i + 1
        self.currently_executing_waypoint = -1
        
    def moveToWaypoint(self, index):
        self.playback(start_index = index, end_index = index + 1)
        
    def __generateWaypoint(self, vel):
        name = "Waypoint " + str(self.__waypoint_counter)
        ljpos = dict([(j, self.left_arm.joint_angle(j)) for j in self.left_arm.joint_names()])
        rjpos = dict([(j, self.right_arm.joint_angle(j)) for j in self.right_arm.joint_names()])
        lgrip = self.left_grip.position()
        rgrip = self.right_grip.position()
        return Waypoint(name, ljpos, rjpos, lgrip, rgrip, vel)
        
    #adds a waypoint. Requires the velocity to be specified.
    def addWaypoint(self, vel):
        self.poss.append(self.__generateWaypoint(vel))
        self.__waypoint_counter = self.__waypoint_counter + 1
        
    #Basically just saves the current state of the robot as the given index in the list with the
    #given velocity.
    def editWaypoint(self, index, vel):
        name = self.poss[index].name
        self.poss[index] = self.__generateWaypoint(vel)
        self.editWaypointName(index, name)
    
    def __move(self, wp, accuracy=1.0):
        
        rate = rospy.Rate(10.0)
        
        global global_right_grip, global_left_grip
        
        if wp.lgrip < 50:
            global_left_grip.close()
        else:
            global_left_grip.open()
        if wp.rgrip < 50:
            global_right_grip.close()
        else:
            global_right_grip.open()
        
        
        done = False
        self.left_arm.set_joint_position_speed(max(min(Script.MAX_VEL*wp.vel/100.0,1.0),0.0)) #default 0.3, range 0.0 - 1.0
        self.right_arm.set_joint_position_speed(max(min(Script.MAX_VEL*wp.vel/100.0,1.0),0.0)) #default 0.3, range 0.0 - 1.0
        dd = [9999.0, 9999.0, 9999.0, 9999.0, 9999.0]
        while not done:
            if self.cancel_move:
                break
        
            self.left_arm.set_joint_positions(wp.ljpos)
            self.right_arm.set_joint_positions(wp.rjpos)
            dist = 0.0
            for j in self.left_arm.joint_names():
                dist = dist + (wp.ljpos[j] - self.left_arm.joint_angle(j))*(wp.ljpos[j] - self.left_arm.joint_angle(j))
            for j in self.right_arm.joint_names():
                dist = dist + (wp.rjpos[j] - self.right_arm.joint_angle(j))*(wp.rjpos[j] - self.right_arm.joint_angle(j))
                
            dist = math.sqrt(dist)
            ss = 0.0
            for d in dd:
                ss = ss + math.fabs(d - dist)
            if ss/(wp.vel/100.0) < 0.1/accuracy:
                done = True
            elif rospy.is_shutdown():
                print("ROS shutdown, cancelled movement")
                done = True
            elif baxter_external_devices.getch() in ['\x1b', '\x03']:
                print("stopped")
                done = True
            else:
                dd.append(dist)
                del dd[0]
                rate.sleep()
        self.left_arm.set_joint_position_speed(0.3)
        self.right_arm.set_joint_position_speed(0.3)
        while global_right_grip.moving() or global_left_grip.moving():
            rate.sleep()
        
    
    #Returns true if the robot is currently moving. Or it should at least. I'm not sure if it actually works.
    def currentMovement(self):
        return self.currently_executing_waypoint
        
    def numWaypoints(self):
        return len(self.poss)
        
    #Allows the user to change the name of a given waypoint for easier referencing (Waypoint 11 isn't very informative).
    def editWaypointName(self, index, new_name):
        self.poss[index] = self.poss[index]._replace(name=new_name)
        
    def getWaypointName(self, index):
        return self.poss[index].name
        
    def getWaypointVel(self, index):
        return self.poss[index].vel
        
    #Swaps the waypoint at index with the waypoint at index-1
    def moveWaypointUp(self, index):
        if index <= 0:
            return
        temp = self.poss[index-1]
        self.poss[index-1] = self.poss[index]
        self.poss[index] = temp
        
    #swaps the waypoint at index with the waypoint at index+1
    def moveWaypointDown(self, index):
        if index >= len(self.poss) - 1:
            return
        temp = self.poss[index+1]
        self.poss[index+1] = self.poss[index]
        self.poss[index] = temp
        
    def delWaypoint(self, index):
        del self.poss[index]
        
    #stops the robot moving (or it should at least).
    def haltMovement(self):
        self.cancel_move = True
    
    def __str__(self):
        return self.__class__.__name__
        

#helper function to get a line of text, usually used as a substitute for "pause".
def getKeyboardLine():
    return raw_input(">")
    

#given a string list (lst), prompts the user to select one of the option in the list.
def select_from_list(lst):
    print("Select one-")
    for i in range(len(lst)):
        print("\t" + str(i) + ": " + str(lst[i]))
    while not rospy.is_shutdown():
        c = baxter_external_devices.getch()
        if c:
            #catch Esc or ctrl-c
            if c in ['\x1b', '\x03']:
                return -1
            elif c >= '0' and c <= '9':
                print("Selected " + str(lst[int(c)]))
                return int(c)
       
#Helper function for opening and closing the gripper.        
def gripper_callback(topic):
    global global_right_grip, global_left_grip
    if "right" in topic:
        grip = global_right_grip
    else:
        grip = global_left_grip

    if grip.moving():
        return
    
    if "lower" in topic:
        grip.open()
    else:
        grip.close()
    
    """
    if grip.position() < 50:
        grip.open()
    else:
        grip.close()
    """
    subprocess.call(["stty", "sane"])
    
#helper function for turning the lights on the robot on/off when the cuff is grabbed.
def cuff_callback(data):
    global right_arm_moving
    if right_arm_moving == (data.state == 1):
        return
    
    right_arm_moving = (data.state == 1)
    if right_arm_moving:
        print("Right arm beginning movement")
    else:
        print("Right arm ending movement")
    subprocess.call(["stty", "sane"])
    
######################## End Direct Baxter Interface Code ######################################




######################## Keybaord & Mouse GUI Code #############################################    

#class taken from the tutorial at http://effbot.org/tkinterbook/tkinter-dialog-windows.htm
class Dialog(Toplevel, object):

    def __init__(self, parent, title = None):

        Toplevel.__init__(self, parent)
        self.transient(parent)

        if title:
            self.title(title)

        self.parent = parent

        self.result = None

        body = Frame(self)
        self.initial_focus = self.body(body)
        body.pack(padx=5, pady=5)

        self.buttonbox()

        self.grab_set()

        if not self.initial_focus:
            self.initial_focus = self

        self.protocol("WM_DELETE_WINDOW", self.cancel)

        self.geometry("+%d+%d" % (parent.winfo_rootx()+50,
                                  parent.winfo_rooty()+50))

        self.initial_focus.focus_set()

        self.wait_window(self)

    #
    # construction hooks

    def body(self, master):
        # create dialog body.  return widget that should have
        # initial focus.  this method should be overridden

        pass

    def buttonbox(self):
        # add standard button box. override if you don't want the
        # standard buttons

        box = Frame(self)

        w = Button(box, text="OK", width=10, command=self.ok, default=ACTIVE, font=("Times", 30, "bold"))
        w.pack(side=LEFT, padx=5, pady=5)
        w = Button(box, text="Cancel", width=10, command=self.cancel, font=("Times", 30, "bold"))
        w.pack(side=LEFT, padx=5, pady=5)

        self.bind("<Return>", self.ok)
        self.bind("<Escape>", self.cancel)

        box.pack()

    #
    # standard button semantics

    def ok(self, event=None):

        if not self.validate():
            self.initial_focus.focus_set() # put focus back
            return

        self.withdraw()
        self.update_idletasks()

        self.apply()

        self.cancel()

    def cancel(self, event=None):

        # put focus back to the parent window
        self.parent.focus_set()
        self.destroy()

    #
    # command hooks

    def validate(self):

        return 1 # override

    def apply(self):

        pass # override
    
#globals for K&M interface
edit_selection = -1

#This function is a giant mess that uses TKinter to create a GUI for interfacing with the robot
def init_km_gui():
    root = Tk()

    frame = Frame(root)
    frame.pack()
    
    script = Script()

    ############################Delete button code#####################################
    class delete_dialog(Dialog):
        def body(self, master):
            self.selection = int(wpList.curselection()[0])
            Label(master, text="Are you sure you want to delete the waypoint \'" + script.getWaypointName(self.selection) + "\'?", font=("Times", 30, "normal")).pack()

        def apply(self):
            script.delWaypoint(self.selection)
            populate_list()
    def delete_callback():
        if len(wpList.curselection()) == 0: return
        delete_dialog(root)
    delete = Button(frame, text="Delete", fg="red", font=("Times", 30, "bold"), command=delete_callback)
    delete.grid(row=0, column=0, pady=10, padx=10)
    ###################################################################################
    
    ############################Edit button code#######################################
    def edit_callback():
        global edit_selection
        if len(wpList.curselection()) == 0: return
        edit_selection = int(wpList.curselection()[0])
        name = ">>>" + wpList.get(edit_selection) + "<<<"
        velScale.set(script.getWaypointVel(edit_selection))
        wpList.delete(edit_selection)
        wpList.insert(edit_selection, name)
        
        delete.config(state=DISABLED)
        rename.config(state=DISABLED)
        moveto.config(state=DISABLED)
        up.config(state=DISABLED)
        down.config(state=DISABLED)
        playBeg.config(state=DISABLED)
        playCur.config(state=DISABLED)
        
        edit.config(command=cancel_edit_callback, text="Cancel")
        add.config(command=set_callback, text="Set")
        
    def cancel_edit_callback():
        global edit_selection
        edit.config(command=edit_callback, text="Edit")
        add.config(command=add_callback, text="Add")
        
        delete.config(state=NORMAL)
        rename.config(state=NORMAL)
        moveto.config(state=NORMAL)
        up.config(state=NORMAL)
        down.config(state=NORMAL)
        playBeg.config(state=NORMAL)
        playCur.config(state=NORMAL)
        
        populate_list()
        wpList.selection_set(edit_selection)
        wpList.see(edit_selection)
        return
        
    def set_callback():
        global edit_selection
        script.editWaypoint(edit_selection, velScale.get())
        cancel_edit_callback()
        return
    edit = Button(frame, text="Edit", font=("Times", 30, "bold"), command=edit_callback)
    edit.grid(row=0, column=3, sticky=E, pady=10, padx=10)
    ###################################################################################
    
    ############################Rename button code#####################################
    class rename_dialog(Dialog):
        def body(self, master):
            self.selection = int(wpList.curselection()[0])
            self.entry = Entry(master, font=("Times", 30, "normal"))
            self.entry.insert(0, script.getWaypointName(self.selection))
            self.entry.selection_range(0, END)
            self.entry.pack()
            return self.entry

        def apply(self):
            name = self.entry.get()
            script.editWaypointName(self.selection, name)
            populate_list()
    def rename_callback():
        if len(wpList.curselection()) == 0: return
        rename_dialog(root)
    rename = Button(frame, text="Rename", font=("Times", 30, "bold"), command=rename_callback)
    rename.grid(row=1, column=3, sticky=E, pady=10, padx=10)
    ###################################################################################
    
    ############################Move To button code#####################################
    class move_dialog(Dialog):
        def __init__(self, parent, title = None, start_index=0, end_index=-1):
            self.start_index = start_index
            self.end_index = end_index
            self.cancelled = False
            super(move_dialog, self).__init__(parent, title)
    
        def body(self, master):
            self.ulabel = Label(master, fg="red", font=("Times", 30, "bold"))
            self.ulabel.grid(row=0, column=0)
            self.llabel = Label(master, font=("Times", 20, "normal"))
            self.llabel.grid(row=1, column=0)
            t = threading.Thread(target=self.progress, args = ())
            t.daemon = True
            t.start()
            
        def progress(self):
            rate = rospy.Rate(1.0)
            for i in [3, 2, 1]:
                self.ulabel.configure(text="WARNING! Beginning movement in " + str(i))
                rate.sleep()
                if self.cancelled:
                    self.__cleanup()
                    return
                
            rate = rospy.Rate(100.0)
            thread = threading.Thread(target=script.playback, args=(self.start_index, self.end_index))
            thread.daemon = True
            thread.start()
            
            self.ulabel.configure(text="WARNING! Robot moving, stand back. ")
            while thread.isAlive():
                cur = script.currentMovement()
                if cur >= 0:
                    self.llabel.configure(text=script.getWaypointName(cur) + " @ " + str(script.getWaypointVel(cur)) + "%")
                    velScale.set(script.getWaypointVel(cur))
                rate.sleep()
            self.__cleanup()                
            
        def __cleanup(self):
            self.withdraw()
            self.update_idletasks()
            self.parent.focus_set()
            self.destroy()
            
        def buttonbox(self):
            box = Frame(self)

            w = Button(box, text="Cancel", width=10, command=self.apply, default=ACTIVE, font=("Times", 30, "bold"))
            w.pack(side=LEFT, padx=5, pady=5)

            self.bind("<Return>", self.apply)
            self.bind("<Escape>", self.apply)

            box.pack()

        def apply(self):
            self.cancelled = True
            script.haltMovement()
            
        def cancel(self):
            self.apply()
            
    def moveto_callback():
        if len(wpList.curselection()) == 0: return
        sel = int(wpList.curselection()[0])
        move_dialog(root, start_index=sel, end_index=sel+1)
    moveto = Button(frame, text="Move To", font=("Times", 30, "bold"), command=moveto_callback)
    moveto.grid(row=2, column=3, sticky=E, pady=10, padx=10)
    ###################################################################################
    
    ############################Add button code#####################################
    def add_callback():
        script.addWaypoint(velScale.get())
        populate_list()
        wpList.selection_set(script.numWaypoints() - 1)
        wpList.see(script.numWaypoints() - 1)
    add = Button(frame, text="Add", font=("Times", 50, "bold"), command=add_callback)
    add.grid(row=3, column=3, sticky=E, pady=30, padx=10)
    ###################################################################################
    
    ############################Velocity adjust code#####################################
    vel = Label(frame, text="Velocity", font=("Times", 30, "bold"))
    vel.grid(row=3, column=2, sticky=W, pady=10)
    
    velLabel = Label(frame, text="50%", font=("Times", 30, "bold"))
    velLabel.grid(row=3, column=0, sticky=E, pady=10)
    
    def velScale_callback(arg):
        velLabel.configure(text=str(velScale.get()) + "%")
    
    velScale = Scale(frame, from_=100, to=1, font=("Times", 30, "bold"), showvalue=0, command=velScale_callback, width=20)
    velScale.grid(row=2, rowspan=3, column=1, sticky=NW+SW)
    velScale.set(50)
    ###################################################################################
    
    ############################Waypoint List code#####################################
    def populate_list():
        wpList.delete(0, END)
        for i in range(script.numWaypoints()):
            wpList.insert(END, script.getWaypointName(i) + " @ " + str(script.getWaypointVel(i)) + "%")
    scrollbar = Scrollbar(frame, orient=VERTICAL, width=50)
    wpList = Listbox(frame, font=("Times", 20, "normal"), yscrollcommand=scrollbar.set, selectmode=BROWSE)
    scrollbar.config(command=wpList.yview)
    scrollbar.grid(row=0, rowspan=4, column=8, sticky=W+N+S)
    wpList.grid(row=0, rowspan=4, column=4, columnspan=4, sticky=N+S+E+W)
    ###################################################################################
    
    ############################Waypoint Reording code#####################################
    def up_callback():
        sel = int(wpList.curselection()[0])
        script.moveWaypointUp(sel)
        populate_list()
        if sel > 0:
            sel = sel - 1
        wpList.selection_set(sel)
        wpList.see(sel)
    upArrow = PhotoImage(file=IMAGE_DIRECTORY + "up_arrow.gif")
    up = Button(frame, image=upArrow, font=("Times", 30, "bold"), command=up_callback)
    up.image = upArrow
    up.grid(row=4, column=5, pady=10, padx=10)
    
    def down_callback():
        sel = int(wpList.curselection()[0])
        script.moveWaypointDown(sel)
        populate_list()
        if sel < script.numWaypoints() - 1:
            sel = sel + 1
        wpList.selection_set(sel)
        wpList.see(sel)
    downArrow = PhotoImage(file=IMAGE_DIRECTORY + "down_arrow.gif")
    down = Button(frame, image=downArrow, font=("Times", 30, "bold"), command=down_callback)
    down.image = downArrow
    down.grid(row=4, column=6, pady=10, padx=10)
    ###################################################################################
    
    Label(frame, text="", width=25).grid(row=4, column=4, sticky=N+S+E+W)
    Label(frame, text="", width=25).grid(row=4, column=7, sticky=N+S+E+W)
    
    Label(frame, text="", height=10).grid(row=5, column=0, columnspan=8, sticky=N+S+E+W)
    
    ############################Playback code#####################################
    Label(frame, text="Begin Playback:", font=("Times", 30, "bold")).grid(row=6, column=0, columnspan=8)
    
    def playBeg_callback():
        print("Playing back from the beginning, " + str(script.numWaypoints()) + " waypoints.")
        move_dialog(root)
    playBeg = Button(frame, text="From Beginning", font=("Times", 30, "bold"), command=playBeg_callback)
    playBeg.grid(row=7, column=0, columnspan=4, sticky=E, pady=10, padx=10)
    
    def playCur_callback():
        if len(wpList.curselection()) == 0: return
        sel = int(wpList.curselection()[0])
        move_dialog(root, start_index=0, end_index=sel+1)
    playCur = Button(frame, text="To Selection", font=("Times", 30, "bold"), command=playCur_callback)
    playCur.grid(row=7, column=4, columnspan=4, sticky=W, pady=10, padx=10)
    ###################################################################################
    
    interface_running = True
    def view_loop():
        img = cv2.imread(IMAGE_DIRECTORY + "test.jpg")
        main_message = cv_bridge.CvBridge().cv_to_imgmsg(cv2.cv.fromarray(img))
        xpub_img = rospy.Publisher('/robot/xdisplay', msgs.Image)
        view_rate = rospy.Rate(1.0)
        while interface_running:
           xpub_img.publish(main_message)
           view_rate.sleep()
            
    view_thread = threading.Thread(target=view_loop, args=())
    view_thread.daemon = True
    view_thread.start()

    root.mainloop()
    
    interface_running = False
    view_thread.join()

######################## End Keyboard & Mouse GUI Code ###########################################




####################### On-Robot Interface Code ##################################################

# This class handles drawing the interface on the robot. Since there's no good way to use a GUI
# package and then get it on to the robot's face, which requires sending an image on a ROS topic,
# the interface is drawn entirely using OpenCV draw commands (oh fun!). The static variables
# defined immediately below (lKnobPress, lUpper,...) are the references to the functions
# displayed on the robot's screen. You'll need to refrence the variables if you want to change
# what is displayed (e.g., RobotInterface.rKnobPress refers to the knob press function of the knob
# on the robot's right arm). The constructor requries a reference to the current Script instance.
# The interface has worker threads, so make sure to call stop_interface before quitting.
class RobotInterface:

    lKnobPress = 1
    lUpper = 2
    lLower = 3
    rKnobPress = 4
    rUpper = 5
    rLower = 6
    rKnobTurn = 7
    lKnobTurn = 8
    
    def __init__(self, script):
        self.main_message = None
        self.interface_running = True
        self.master_selection = 5
        self.list_view_start = 0
        
        self.master_script = script
            
        self.background = cv2.imread(IMAGE_DIRECTORY + "robot_interface_background.png")
        #self.downpress_left = cv2.imread(IMAGE_DIRECTORY + "downpress_left.jpg")
        #self.downpress_right = cv2.imread(IMAGE_DIRECTORY + "downpress_right.jpg")
        self.downpress_left = cv2.imread(IMAGE_DIRECTORY + "downpress_arrow.png")
        self.downpress_right = self.downpress_left
        self.circle_arrows = cv2.imread(IMAGE_DIRECTORY + "circle_arrows.png")
        
        self.background = self.background[::-1,:,:]
        self.downpress_left = self.downpress_left[::-1,:,:]
        self.downpress_right = self.downpress_right[::-1,:,:]
        self.circle_arrows = self.circle_arrows[::-1,:,:]
        
        self.buttonMap = {"/robot/digital_io/right_itb_button0/state" : RobotInterface.rKnobPress,
                          "/robot/digital_io/right_itb_button1/state" : RobotInterface.rUpper,
                          "/robot/digital_io/right_itb_button2/state" : RobotInterface.rLower,
                          "/robot/digital_io/left_itb_button0/state" : RobotInterface.lKnobPress,
                          "/robot/digital_io/left_itb_button1/state" : RobotInterface.lUpper,
                          "/robot/digital_io/left_itb_button2/state" : RobotInterface.lLower,
                          "/robot/itb/right_itb/state" : RobotInterface.rKnobTurn,
                          "/robot/itb/light_itb/state" : RobotInterface.lKnobTurn}
        
        self.textMap = {}
        self.highlightMap = {}
        for v in self.buttonMap.itervalues():
           self.textMap[v] = ""
           self.highlightMap[v] = False
           
        for k in self.buttonMap.iterkeys():
            diom = DigitalIOMonitor(k)
            diom.setButtonUpListener(self.__button_up_listener)
            diom.setButtonDownListener(self.__button_down_listener)
            
        self.statusColor = (0,255,0)
        self.statusText = ""
        
        self.locked = None
        for v in ["/robot/digital_io/right_lower_cuff/state", "/robot/digital_io/left_lower_cuff/state"]:
            diom = DigitalIOMonitor(v)
            diom.setButtonUpListener(self.__cuff_release_listener)
            diom.setButtonDownListener(self.__cuff_grab_listener)
            
        self.redraw = True
        thread = threading.Thread(target=self.__view_loop, args=())
        thread.daemon = True
        thread.start()
        self.view_thread = thread
        
    #Stops all worker threads.
    def stop_interface(self):
        self.interface_running = False
        if self.view_thread.isAlive():
            self.view_thread.join()
        
    def __button_up_listener(self, topic):
        self.highlightMap[self.buttonMap[topic]] = False
        self.call_for_redraw()
        
    def __button_down_listener(self, topic):
        self.highlightMap[self.buttonMap[topic]] = True
        self.call_for_redraw()
        
    def __cuff_grab_listener(self, topic):
        self.setLocked("Backdriving")
        self.call_for_redraw()
    
    def __cuff_release_listener(self, topic):
        self.unlock()
        self.call_for_redraw()

    ################### View Loop #############################################
    def __view_loop(self):
        xpub_img = rospy.Publisher('/robot/xdisplay', msgs.Image)
        light_pub = rospy.Publisher('/robot/digital_io/command', DigitalOutputCommand)
        view_rate = rospy.Rate(100.0)
        while self.interface_running:
            if self.redraw:
                self.redraw = False
                self.__redraw_interface()
            if self.main_message is not None:
                xpub_img.publish(self.main_message)
            for light in ['left_itb_light_inner', 'left_itb_light_outer', 'right_itb_light_inner', 'right_itb_light_outer']:
                out = DigitalOutputCommand()
                out.name = light
                out.value = (self.locked is None)
                light_pub.publish(out)
            view_rate.sleep()
            
        for light in ['left_itb_light_inner', 'left_itb_light_outer', 'right_itb_light_inner', 'right_itb_light_outer']:
            out = DigitalOutputCommand()
            out.name = light
            out.value = False
            light_pub.publish(out)
    ##########################################################################
    
    #################### Draw Function #######################################
    #Tells the interface to redraw itself. All functions that change what is on the interface call this, but don't worry,
    #many calls in quick succession will not cause many redraws to happen at once. The worker thread waits until it is
    #finished drawing to start drawing again.
    def call_for_redraw(self):
        self.redraw = True
    
    def __redraw_interface(self):
        img = np.copy(self.background)
        lx = 125
        rx = 855
        uy = 600 - 450
        ly = 600 - 500
        
        fontFace = cv2.FONT_HERSHEY_DUPLEX
        fontSize = 1.5
        thickness = 3
        
        if self.locked is None:
            img[uy:uy+self.downpress_left.shape[0],lx:lx+self.downpress_left.shape[1]] = self.downpress_left
            img[uy:uy+self.downpress_right.shape[0],rx+5:rx+5+self.downpress_right.shape[1]] = self.downpress_right
            img[ly:ly+self.circle_arrows.shape[0],lx:lx+self.circle_arrows.shape[1]] = self.circle_arrows
            img[ly:ly+self.circle_arrows.shape[0],rx:rx+self.circle_arrows.shape[1]] = self.circle_arrows
            

            def draw_right_text(image, x, y, text, highlight):
                if highlight:
                    ((width, height), under) = cv2.getTextSize(text, fontFace, fontSize, thickness)
                    cv2.rectangle(image, (x, y - under), (x + width, y + height + 5), color=(200, 200, 200), thickness=-1)
                    cv2.putText(image, text, (x, y), fontFace, fontSize, color=(0,0,255), thickness=thickness, bottomLeftOrigin=True)
                else:
                    cv2.putText(image, text, (x, y), fontFace, fontSize, color=(0,0,0), thickness=thickness, bottomLeftOrigin=True)
            
            #rKnobPress
            draw_right_text(img, lx+40, uy+5, self.textMap[RobotInterface.rKnobPress], self.highlightMap[RobotInterface.rKnobPress])
            #rKnobTurn
            draw_right_text(img, lx+40, ly+3, self.textMap[RobotInterface.rKnobTurn], self.highlightMap[RobotInterface.rKnobTurn])
            #rUpper
            draw_right_text(img, lx+10, uy+90, self.textMap[RobotInterface.rUpper], self.highlightMap[RobotInterface.rUpper])
            #rLower
            draw_right_text(img, lx+10, ly-75, self.textMap[RobotInterface.rLower], self.highlightMap[RobotInterface.rLower])
            
            def draw_left_text(image, x, y, text, highlight):
                width = cv2.getTextSize(text, fontFace, fontSize, thickness)[0][0]
                x = x - width
                draw_right_text(image, x, y, text, highlight)
                    
            #lKnobPress
            draw_left_text(img, rx, uy+5, self.textMap[RobotInterface.lKnobPress], self.highlightMap[RobotInterface.lKnobPress])
            #lKnobTurn
            draw_left_text(img, rx, ly+3, self.textMap[RobotInterface.lKnobTurn], self.highlightMap[RobotInterface.lKnobTurn])
            #lUpper
            draw_left_text(img, rx+30, uy+90, self.textMap[RobotInterface.lUpper], self.highlightMap[RobotInterface.lUpper])
            #lLower
            draw_left_text(img, rx+30, ly-75, self.textMap[RobotInterface.lLower], self.highlightMap[RobotInterface.lLower])
        else:
            (width, height) = cv2.getTextSize(self.locked, cv2.FONT_HERSHEY_DUPLEX, self.lockedFontSize, thickness=int(self.lockedFontSize/1.5*3.0))[0]
            cv2.putText(img, self.locked, (lx+10+(710-width)/2, ly+max(0, (75-height)/2)), cv2.FONT_HERSHEY_DUPLEX, self.lockedFontSize, color=(0,0,255), thickness=int(self.lockedFontSize/1.5*3.0), bottomLeftOrigin=True)
        
        list_x = 500
        list_y = 600 - 40
        max_view = 7
        
        cv2.rectangle(img, (list_x-5, list_y+35), (list_x+400, list_y-(40*max_view)+25), color=(0,0,0), thickness=thickness)
        if self.master_script.numWaypoints() > 0:
            #let's make sure the selection is in view
            while self.master_selection < self.list_view_start:
                self.list_view_start = self.list_view_start - 1
            while self.master_selection >= self.list_view_start + max_view:
                self.list_view_start = self.list_view_start + 1
                
            cv2.rectangle(img, (list_x-5, list_y - 40*(self.master_selection - self.list_view_start - 1) - 10), (list_x+400, list_y - 40*(self.master_selection - self.list_view_start) - 10), color=(200,200,200), thickness=-1)
            
            for i in range(self.list_view_start, min(self.list_view_start + max_view, self.master_script.numWaypoints())):
                cv2.putText(img, self.master_script.getWaypointName(i) + " @ " + str(self.master_script.getWaypointVel(i)) + "%", 
                            (list_x, list_y - 40*(i - self.list_view_start)), fontFace, fontSize-0.5, color=(0,0,0), thickness=thickness-1, bottomLeftOrigin=True)
                        
            scroll_height = int(min(1.0, 1.0*max_view/self.master_script.numWaypoints())*((list_y+35) - (list_y-(40*max_view)+25)))
            scroll_y = (list_y+35) - int(1.0*self.list_view_start/self.master_script.numWaypoints()*40.0*max_view)
            cv2.rectangle(img, (list_x+400+thickness, scroll_y), (list_x+400+20, scroll_y-scroll_height), color=(100,100,100), thickness=-1)
        
        text_x = 50
        text_y = 600 - 100
        box_width = 400
        """ Fuck it, this is too complex, we'll handle newlines manually
        def draw_text(image, y, text):
            def find_split(break_list, left, right, full_text):
                if left >= right:
                    return break_list[right]
                i = (right - left)/2 + left
                width = cv2.getTextSize(full_text[0:break_list[i]], fontFace, fontSize-0.5, thickness-1)[0][0]
                if width > box_width:
                    return find_split(break_list, left, i-1, full_text)
                else
                    return find_split(break_list, i, right, full_text)
        """
        y = text_y
        for line in self.statusText.split('\n'):
            cv2.putText(img, line, (text_x, y), fontFace, fontSize-0.5, color=self.statusColor, thickness=thickness-1, bottomLeftOrigin=True)
            y = y - 40
        
        
        img = np.copy(img[::-1,:,:])
        #cv2.imwrite(IMAGE_DIRECTORY + "robot_interface_screenshot.png", img)
        self.main_message = cv_bridge.CvBridge().cv_to_imgmsg(cv2.cv.fromarray(img))
    ##########################################################################
    
    #Sets the text for each of the action functions displayed on the robot's screen. Takes
    #a map of RobotInterface.VAR -> String, where VAR is one of the static variables defined
    #above. Only changes the passed actions functions, leaves the others unchanged.
    def setText(self, textMap):
        for (k, v) in textMap.iteritems():
            self.textMap[k] = v
        self.call_for_redraw()
        
    #Sets the text displayed in the upper left box of the robot's screen. The text
    #must have newline variables in it, otherwise it will draw all the way across
    #screen to its heart's content. May optionally set the color to something other
    #than green.
    def setStatusText(self, text, color=(0,255,0)):
        self.statusColor = color
        self.statusText = text
        self.call_for_redraw()
       
    #Sets which of the waypoints is currently highlighted in the list. 
    def setSelection(self, sel):
        if sel < 0:
            sel = 0
        if sel >= self.master_script.numWaypoints():
            sel = self.master_script.numWaypoints() - 1
        self.master_selection = sel
        self.call_for_redraw()
     
    #Returns which waypoint is currently highlighted in the list   
    def getSelection(self):
        return self.master_selection
       
    #Locks out all the actions on the robot's interface and displays the given text in red in the
    #bottom center of the screen. 
    def setLocked(self, text):
        self.locked = text
        self.lockedFontSize = 3.0
        width = cv2.getTextSize(text, cv2.FONT_HERSHEY_DUPLEX, self.lockedFontSize, thickness=int(self.lockedFontSize/1.5*3.0))[0][0]
        while width > 710:
            self.lockedFontSize = self.lockedFontSize - 0.25
            width = cv2.getTextSize(text, cv2.FONT_HERSHEY_DUPLEX, self.lockedFontSize, thickness=int(self.lockedFontSize/1.5*3.0))[0][0]
        self.call_for_redraw()
    
    #Unlocks the robot's interface.
    def unlock(self):
        self.locked = None
        self.call_for_redraw()

#robot interface globals
move_cancelled = False

#This function is also a giant mess that sets up the On-Robot Interface
def init_robot_interface():

    master_script = Script()
    interface = RobotInterface(master_script)
                       
    lUpperButton = DigitalIOMonitor("/robot/digital_io/left_itb_button1/state")
    right_knob = KnobMonitor("/robot/itb/right_itb/state", 1, 100, invert=True)
    left_knob = KnobMonitor("/robot/itb/left_itb/state", 0, max(0, master_script.numWaypoints()-1), invert=False)
    rKnobButton = DigitalIOMonitor("/robot/digital_io/right_itb_button0/state")
    lLowerButton = DigitalIOMonitor("/robot/digital_io/left_itb_button2/state")
    rUpperButton = DigitalIOMonitor("/robot/digital_io/right_itb_button1/state")
    lKnobButton = DigitalIOMonitor("/robot/digital_io/left_itb_button0/state")
    rLowerButton = DigitalIOMonitor("/robot/digital_io/right_itb_button2/state")
                       
    def clear_all_buttons():
        interface.setText({RobotInterface.lUpper : "", RobotInterface.lLower : "", RobotInterface.lKnobPress : "", RobotInterface.lKnobTurn : "",
                       RobotInterface.rUpper : "", RobotInterface.rLower : "", RobotInterface.rKnobPress : "", RobotInterface.rKnobTurn : ""})
        right_knob.setChangeListener(None)
        left_knob.setChangeListener(None)
        rKnobButton.setButtonUpListener(None)
        rKnobButton.setButtonDownListener(None)
        lLowerButton.setButtonUpListener(None)
        lLowerButton.setButtonDownListener(None)
        lUpperButton.setButtonUpListener(None)
        lUpperButton.setButtonDownListener(None)
        lKnobButton.setButtonUpListener(None)
        lKnobButton.setButtonDownListener(None)
        rLowerButton.setButtonUpListener(None)
        rLowerButton.setButtonDownListener(None)
        rUpperButton.setButtonUpListener(None)
        rUpperButton.setButtonDownListener(None)
        return
    def reset_all_buttons():
        interface.setText({RobotInterface.lUpper : "Playback", RobotInterface.lLower : "Move To", RobotInterface.lKnobPress : "Reorder", RobotInterface.lKnobTurn : "Change Sel",
                       RobotInterface.rUpper : "Delete", RobotInterface.rLower : "Edit", RobotInterface.rKnobPress : "Add", RobotInterface.rKnobTurn : "rKnobTurn"})
        
        interface.setText({RobotInterface.rKnobTurn : str(right_knob.get()) + "%"})
        right_knob.setChangeListener(vel_knob_callback)
        left_knob.setChangeListener(sel_knob_callback)
        rKnobButton.setButtonUpListener(add_callback)
        rKnobButton.setButtonDownListener(None)
        lLowerButton.setButtonUpListener(moveto_callback)
        lLowerButton.setButtonDownListener(None)
        lUpperButton.setButtonUpListener(playback_callback)
        lUpperButton.setButtonDownListener(None)
        lKnobButton.setButtonUpListener(reorder_callback)
        lKnobButton.setButtonDownListener(None)
        rLowerButton.setButtonUpListener(edit_callback)
        rLowerButton.setButtonDownListener(None)
        rUpperButton.setButtonUpListener(del_callback)
        rUpperButton.setButtonDownListener(None)
    
    ####################### Change Velocity #############################################
    def vel_knob_callback(topic):
        interface.setText({RobotInterface.rKnobTurn : str(right_knob.get()) + "%"})
    
    interface.setText({RobotInterface.rKnobTurn : "50%"})
    right_knob.set(50)
    right_knob.setChangeListener(vel_knob_callback)
    #####################################################################################
    
    ####################### Change Selection ############################################
    def sel_knob_callback(topic):
        interface.setSelection(left_knob.get())
    
    left_knob.setChangeListener(sel_knob_callback)
    interface.setText({RobotInterface.lKnobTurn : "Change Sel"})
    interface.setSelection(0)
    left_knob.set(0)
    #####################################################################################
    
    ####################### Add #########################################################
    def add_callback(topic):
        master_script.addWaypoint(right_knob.get())
        interface.setSelection(master_script.numWaypoints() - 1)
        left_knob.setMaxVal(master_script.numWaypoints() - 1)
        left_knob.set(master_script.numWaypoints() - 1)
        interface.setStatusText(master_script.getWaypointName(master_script.numWaypoints() - 1) + " added at\n" + str(master_script.getWaypointVel(master_script.numWaypoints() - 1)) + "% velocity.")
    
    rKnobButton.setButtonUpListener(add_callback)
    interface.setText({RobotInterface.rKnobPress : "Add"})
    #####################################################################################
    
    ####################### Move To #########################################################
    def move_arms(start_index, end_index=-1):
        if end_index < 0:
            end_index = start_index + 1
            
        clear_all_buttons()
        rate = rospy.Rate(1.0)
        interface.setStatusText("Press any button\nto cancel")
        global move_cancelled
        move_cancelled = False
        
        def cancel_callback(topic):
            global move_cancelled
            move_cancelled = True
            master_script.haltMovement()
            
        for button in [rKnobButton, lLowerButton, lUpperButton, lKnobButton, rLowerButton, rUpperButton]:
            button.setButtonUpListener(cancel_callback)
        
        for i in [3, 2, 1]:
            interface.setLocked("WARNING! Beginning movement in " + str(i))
            rate.sleep()
            if move_cancelled:
                reset_all_buttons()
                interface.unlock()
                interface.setStatusText("Movement cancelled")
                return
            
        rate = rospy.Rate(100.0)
        thread = threading.Thread(target=master_script.playback, args=(start_index, end_index))
        thread.daemon = True
        thread.start()
        
        interface.setLocked("WARNING! Robot moving, stand back.")
        while thread.isAlive():
            cur = master_script.currentMovement()
            if cur >= 0:
                interface.setStatusText("Moving to\n" + master_script.getWaypointName(cur) + " @ " + str(master_script.getWaypointVel(cur)) + "%\nPress any button\nto cancel")
            rate.sleep()
        
        reset_all_buttons()
        interface.unlock()
        if move_cancelled:
            interface.setStatusText("Movement cancelled")
        else:
            interface.setStatusText("Finished movement at\n" + master_script.getWaypointName(end_index - 1))
        
    def moveto_callback(topic):
        if master_script.numWaypoints() <= 0: return
        def moveto_thread():
            move_arms(interface.getSelection())
        thread = threading.Thread(target=moveto_thread, args=())
        thread.daemon = True
        thread.start()
    
    lLowerButton.setButtonUpListener(moveto_callback)
    interface.setText({RobotInterface.lLower : "Move To"})
    #####################################################################################
    
    ####################### Playback ####################################################
    def playback_beginning(topic):
        print("Playing back from the beginning, " + str(master_script.numWaypoints()) + " waypoints.")
        def moveto_thread():
            move_arms(0, end_index=master_script.numWaypoints())
        thread = threading.Thread(target=moveto_thread, args=())
        thread.daemon = True
        thread.start()
    def playback_selection(topic):
        def moveto_thread():
            move_arms(0, end_index=interface.getSelection()+1)
        thread = threading.Thread(target=moveto_thread, args=())
        thread.daemon = True
        thread.start()
    def playback_cancel(topic):
        reset_all_buttons()
    def playback_callback(topic):
        if master_script.numWaypoints() <= 0: return
        clear_all_buttons()
        interface.setText({RobotInterface.lUpper : "From Beginning", RobotInterface.lLower : "To Selected", RobotInterface.lKnobPress : "Cancel", RobotInterface.lKnobTurn : "Change Sel"})
        left_knob.setChangeListener(sel_knob_callback)
        lUpperButton.setButtonUpListener(playback_beginning)
        lLowerButton.setButtonUpListener(playback_selection)
        lKnobButton.setButtonUpListener(playback_cancel)    
    
    lUpperButton.setButtonUpListener(playback_callback)
    interface.setText({RobotInterface.lUpper : "Playback"})
    #####################################################################################
    
    ####################### Reorder #####################################################
    def reorder_up(topic):
        master_script.moveWaypointUp(interface.getSelection())
        interface.setSelection(interface.getSelection()-1)
        left_knob.set(interface.getSelection())
        interface.call_for_redraw()
    def reorder_down(topic):
        master_script.moveWaypointDown(interface.getSelection())
        interface.setSelection(interface.getSelection()+1)
        left_knob.set(interface.getSelection())
        interface.call_for_redraw()
    def reorder_done(topic):
        reset_all_buttons()
    def reorder_callback(topic):
        if master_script.numWaypoints() <= 0: return
        clear_all_buttons()
        interface.setText({RobotInterface.lUpper : "Up", RobotInterface.lLower : "Down", RobotInterface.lKnobPress : "Done", RobotInterface.lKnobTurn : "Change Sel"})
        left_knob.setChangeListener(sel_knob_callback)
        lUpperButton.setButtonUpListener(reorder_up)
        lLowerButton.setButtonUpListener(reorder_down)
        lKnobButton.setButtonUpListener(reorder_done)    
    
    lKnobButton.setButtonUpListener(reorder_callback)
    interface.setText({RobotInterface.lKnobPress : "Reorder"})
    #####################################################################################
    
    ####################### Edit ########################################################
    def edit_save(topic):
        master_script.editWaypoint(interface.getSelection(), right_knob.get())
        interface.setStatusText(master_script.getWaypointName(interface.getSelection()) + " @ " + str(master_script.getWaypointVel(interface.getSelection())) + "%\nsaved successfully.")
        reset_all_buttons()
    def edit_cancel(topic):
        reset_all_buttons()
    def edit_callback(topic):
        if master_script.numWaypoints() <= 0: return
        clear_all_buttons()
        right_knob.set(master_script.getWaypointVel(interface.getSelection()))
        interface.setText({RobotInterface.rLower : "Cancel", RobotInterface.rKnobPress : "Save", RobotInterface.rKnobTurn : str(right_knob.get()) + "%"})
        right_knob.setChangeListener(vel_knob_callback)
        rLowerButton.setButtonUpListener(edit_cancel)
        rKnobButton.setButtonUpListener(edit_save)    
    
    rLowerButton.setButtonUpListener(edit_callback)
    interface.setText({RobotInterface.rLower : "Edit"})
    #####################################################################################
    
    ####################### Delete ######################################################
    def del_no(topic):
        interface.setStatusText("")
        reset_all_buttons()
    def del_yes(topic):
        interface.setStatusText(master_script.getWaypointName(interface.getSelection()) + " deleted")
        master_script.delWaypoint(interface.getSelection())
        if interface.getSelection() >= master_script.numWaypoints():
            interface.setSelection(interface.getSelection() - 1)
        reset_all_buttons()
    def del_callback(topic):
        if master_script.numWaypoints() <= 0: return
        clear_all_buttons()
        interface.setText({RobotInterface.rLower : "Yes", RobotInterface.rUpper : "No"})
        interface.setStatusText("Are you sure you want to\ndelete " + master_script.getWaypointName(interface.getSelection()) + "?\nThis cannot be undone.")
        rLowerButton.setButtonUpListener(del_yes)
        rUpperButton.setButtonUpListener(del_no)
    
    rUpperButton.setButtonUpListener(del_callback)
    interface.setText({RobotInterface.rUpper : "Delete"})
    #####################################################################################

    interface_running = True
	print("Press enter to quit")
    getKeyboardLine()
    interface.stop_interface()
    
######################### End On-Robot Interface Code ##########################################

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-k", "--km", action="store_true", help="Use the keyboard & mouse interface")
    parser.add_argument("-r", "--robot", action="store_true", help="Use the on-robot interface [default]")
    args = parser.parse_args()
    
    print("Initializing node... ")
    rospy.init_node("scripting_interface")
    print("Getting robot state... ")
    rs = baxter_interface.RobotEnable()
    print("Enabling robot... ")
    rs.enable()
    
    global global_left_grip, global_right_grip
    
    global_left_grip = baxter_interface.Gripper('left')
    global_right_grip = baxter_interface.Gripper('right')
    
    #global_left_grip.calibrate()
    #global_right_grip.calibrate()
    
    #These set up the grippers so that pushing the wrist buttons open and close them
    rlb = DigitalIOMonitor("robot/digital_io/right_lower_button/state")
    rlb.setButtonDownListener(gripper_callback)
    rlb.setButtonUpListener(gripper_callback)
    
    rub = DigitalIOMonitor("robot/digital_io/right_upper_button/state")
    rub.setButtonDownListener(gripper_callback)
    rub.setButtonUpListener(gripper_callback)
    
    llb = DigitalIOMonitor("robot/digital_io/left_lower_button/state")
    llb.setButtonDownListener(gripper_callback)
    llb.setButtonUpListener(gripper_callback)
    
    lub = DigitalIOMonitor("robot/digital_io/left_upper_button/state")
    lub.setButtonDownListener(gripper_callback)
    lub.setButtonUpListener(gripper_callback)
    
    #starts the correct interface depending on the command line argument
    if args.km:
        init_km_gui()
    else:
        init_robot_interface()    
    
    rospy.signal_shutdown("finished.")
    
    
    
if __name__ == '__main__':
    main()
    
    
    
    
    
    
    
    
    
    
    
    
    
    
    
