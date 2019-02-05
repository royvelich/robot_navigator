#!/usr/bin/env python
# -*- coding: utf-8 -*-
import sys
import socket
import json
import time
import errno
import numpy
import threading
import Queue
from errnames import get_error_name

def get_ip():
    from netifaces import interfaces, ifaddresses, AF_INET
    for interface in interfaces():
        try:
            for link in ifaddresses(interface)[AF_INET]:
                ip=str(link['addr'])
                if ip.startswith('192.168.1.'):
                    return ip
        except KeyError:
            pass
    return '127.0.0.1'

class RClient(object):
    """ 
    Robot python interface class
    Typical usage involves:
        
        r=RClient("192.168.1.151",2777)
        if not r.connect(): print error and exit
        while main_loop:
            r.drive(left_speed,right_speed)
            sensors=r.sense()
            some_calculations()
        r.terminate()
        
    """
    def __init__(self,host,port,user_deprecate='',id_deprecate=''):
        self.ip = get_ip()
        #self.ip = '127.0.0.1'
        self.robot=(host,port)
        self.lock=threading.RLock()
        self.done=False
        self.sensors=[0.0,0.0,0.0,0.0,0.0]
        
    def connect(self):
        """ Connect to server and create processing thread """
        try:
            self.recv_thread=threading.Thread(target=self.recv_loop)
            self.recv_thread.start()
            return True
        except socket.error,e:
            reason=get_error_name(e[0])
            print "Socket Error: "+reason
        return False
            
    def recv_loop(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((self.ip,9209))
        sock.setblocking(0)
        while not self.done:
            try:
                data, addr = sock.recvfrom(256)
                if len(data)==0:
                    time.sleep(0.05)
                else:
                    # print "Received from '{}' data='{}'".format(addr,data)
                    try:
                        self.sensors=[float(s) for s in data.split()]
                    except ValueError:
                        pass
            except socket.error,e:
                # errnum=e[0]
                # if errnum!=errno.EAGAIN:
                #     reason=get_error_name(errnum)
                #     print "Socket Error ({}): {}".format(errnum,reason)
                # time.sleep(0.05)
                y = 6
            
    def sendmsg(self,msg):
        with self.lock:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.sendto(msg, self.robot)
                return True
            except socket.error:
                return False
            
    def terminate(self):
        """ Call before your program ends, for a clean exit """
        self.done=True
        self.recv_thread.join()
        
    def drive(self,left,right):
        """ Make the robot move.  Send 2 integers for motors [-1000 : 1000] """
        self.sendmsg("{} {}".format(left,right))
        
    def sense(self):
        """ Get a list of sensor readings.  5 floating point values:  X,Y, 3 sonars """
        return self.sensors


#
#  Following code is a simple test main that allows to control the robot
#  from the keyboard, and see raw sersor readings on the screen
#
done=False
cmd=''

def kbd():
    global cmd
    while cmd!='q':
        cmd=sys.stdin.readline().strip()
    cmd=''
    global done
    done = True
    Alex


class Planner:
    def __init__(self, cm_per_unit, grid_size_cm):
        self.cm_per_unit = cm_per_unit
        self.grid_size_cm = grid_size_cm
        self.grid_size = int(self.grid_size_cm / self.cm_per_unit)
        self.grid_width = self.grid_size
        self.grid_height = self.grid_size
        self.center_grid_x = int(self.grid_size / 2)
        self.center_grid_y = int(self.grid_size / 2)
        self.obstacle_grid = numpy.zeros((self.grid_width, self.grid_height))
        self.rclient = RClient("192.168.1.152", 2777)
        self.connected = False
        if self.rclient.connect():
            self.connected = True
            self.state = 'idle'
            time.sleep(1)


    def in_bounds(self, x, y, grid_width, grid_height):
        return x >= 0 and x < grid_width and y >= 0 and y < grid_height


    def is_clear(self, x, y, grid_width, grid_height):
        if self.in_bounds(x, y, grid_width, grid_height):
            return self.grid[x][y] == 0
        return False


    def set_if_clear(self, x, y, grid_width, grid_height, value, q):
        if self.is_clear(x, y, grid_width, grid_height):
            self.grid[x][y] = value
            q.put((x, y, value))


    def populate_grid(self, grid_goal_x, grid_goal_y):
        self.grid = numpy.copy(self.obstacle_grid)
        q = Queue.Queue()
        if self.is_clear(grid_goal_x, grid_goal_y, self.grid_width, self.grid_height):
            self.grid[grid_goal_x][grid_goal_y] = 2;
            q.put((grid_goal_x, grid_goal_y, 2))
            while not q.empty():
                (x_grid, y_grid, value) = q.get()
                value = value + 1
                self.set_if_clear(x_grid + 1, y_grid, self.grid_width, self.grid_height, value, q)
                self.set_if_clear(x_grid, y_grid + 1, self.grid_width, self.grid_height, value, q)
                self.set_if_clear(x_grid - 1, y_grid, self.grid_width, self.grid_height, value, q)
                self.set_if_clear(x_grid, y_grid - 1, self.grid_width, self.grid_height, value, q)


    def get_cell_helper(self, world_units, grid_units):
        return int(world_units / self.cm_per_unit + grid_units)


    def get_cell(self, x_world, y_world):
        return self.get_cell_helper(x_world, self.center_grid_x), self.get_cell_helper(y_world, self.center_grid_y)


    def travelable_cell(self, x_grid, y_grid, grid_value):
        return self.in_bounds(x_grid, y_grid, self.grid_size, self.grid_size) and self.grid[x_grid, y_grid] < grid_value


    def plan(self):
        while self.state is not 'done':
            data = self.rclient.sense()
            # print data

            world_x = data[0]
            world_y = data[1]

            world_dir_x = data[2]
            world_dir_y = data[3]

            (grid_x, grid_y) = self.get_cell(world_x, world_y)

            print "grid position: (" + str(grid_x) + ", " + str(grid_y) + ") [grid center: (" + str(self.center_grid_x) + ", " + str(self.center_grid_y) + ")]"


            if abs(grid_x - self.grid_goal_x) < 3 and abs(grid_y - self.grid_goal_y) < 3:
                self.state = 'done'

            if grid_x >= 0 and grid_y >= 0:
                grid_value = self.grid[grid_x, grid_y]

                if self.state == 'idle':
                    movement_dir = numpy.array([0, 0])
                    if self.travelable_cell(grid_x + 1, grid_y, grid_value):
                        movement_dir = numpy.array([1, 0])
                    elif self.travelable_cell(grid_x - 1, grid_y, grid_value):
                        movement_dir = numpy.array([-1, 0])
                    elif self.travelable_cell(grid_x, grid_y + 1, grid_value):
                        movement_dir = numpy.array([0, 1])
                    elif self.travelable_cell(grid_x, grid_y - 1, grid_value):
                        movement_dir = numpy.array([0, -1])

                robot_dir = numpy.array([world_dir_x, world_dir_y])

                # movement_dir = numpy.array([1, 0])

                dot = numpy.dot(movement_dir, robot_dir)
                cross = numpy.cross(movement_dir, robot_dir)

                if dot < 0.98 and self.state == 'idle':
                    if cross > 0:
                        self.state = 'cw'
                    else:
                        self.state = 'ccw'
                else:
                    self.state = 'idle'

                speed = 400
                rot_speed = 400

                if self.state == 'cw':
                    self.speed_right = rot_speed
                    self.speed_left = -rot_speed
                elif self.state == 'ccw':
                    self.speed_right = -rot_speed
                    self.speed_left = rot_speed
                elif self.state == 'idle':
                    self.speed_right = speed
                    self.speed_left = speed

                self.rclient.drive(self.speed_left, self.speed_right)
                time.sleep(0.08)

        self.rclient.terminate()

    def drive(self):
        while not self.state is 'done':
            self.rclient.drive(self.speed_left, self.speed_right)
            time.sleep(0.2)


    def go(self, goal_x, goal_y):
        # (grid_goal_x, grid_goal_y) = self.get_cell(goal_x, goal_y)
        self.grid_goal_x = goal_x
        self.grid_goal_y = goal_y
        self.populate_grid(self.grid_goal_x, self.grid_goal_y)
        self.plan()

        # self.plan_thread = threading.Thread(target=self.plan)
        # self.plan_thread.start()
        #
        # self.drive_thread = threading.Thread(target=self.drive)
        # self.drive_thread.start()


if __name__=='__main__':
    planner = Planner(cm_per_unit=4, grid_size_cm=400)
    planner.go(78, 33)

