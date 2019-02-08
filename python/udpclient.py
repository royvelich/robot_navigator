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
    done=True


class Planner:
    def __init__(self, cm_per_unit, grid_size_cm):
        # self.attraction_factor = 0.1
        # self.repulsion_factor = 60

        self.attraction_factor = 0.1
        self.repulsion_factor = 60

        # self.cm_per_unit = cm_per_unit
        # self.grid_size_cm = grid_size_cm
        # self.grid_size = int(self.grid_size_cm / self.cm_per_unit)
        # self.grid_width = self.grid_size
        # self.grid_height = self.grid_size
        # self.center_grid_x = int(self.grid_size / 2)
        # self.center_grid_y = int(self.grid_size / 2)
        # self.obstacle_grid = numpy.zeros((self.grid_width, self.grid_height))
        self.rclient = RClient("192.168.1.152", 2777)
        self.connected = False
        if self.rclient.connect():
            self.connected = True
            self.state = 'forward'
            time.sleep(1)


    # def in_bounds(self, x, y, grid_width, grid_height):
    #     return x >= 0 and x < grid_width and y >= 0 and y < grid_height
    #
    #
    # def is_clear(self, x, y, grid_width, grid_height):
    #     if self.in_bounds(x, y, grid_width, grid_height):
    #         return self.grid[x][y] == 0
    #     return False
    #
    #
    # def set_if_clear(self, x, y, grid_width, grid_height, value, q):
    #     if self.is_clear(x, y, grid_width, grid_height):
    #         self.grid[x][y] = value
    #         q.put((x, y, value))
    #
    #
    # def populate_grid(self, grid_goal_x, grid_goal_y):
    #     self.grid = numpy.copy(self.obstacle_grid)
    #     q = Queue.Queue()
    #     if self.is_clear(grid_goal_x, grid_goal_y, self.grid_width, self.grid_height):
    #         self.grid[grid_goal_x][grid_goal_y] = 2;
    #         q.put((grid_goal_x, grid_goal_y, 2))
    #         while not q.empty():
    #             (x_grid, y_grid, value) = q.get()
    #             value = value + 1
    #             self.set_if_clear(x_grid + 1, y_grid, self.grid_width, self.grid_height, value, q)
    #             self.set_if_clear(x_grid, y_grid + 1, self.grid_width, self.grid_height, value, q)
    #             self.set_if_clear(x_grid - 1, y_grid, self.grid_width, self.grid_height, value, q)
    #             self.set_if_clear(x_grid, y_grid - 1, self.grid_width, self.grid_height, value, q)
    #
    #
    # def get_cell_helper(self, world_units, grid_units):
    #     return int(world_units / self.cm_per_unit + grid_units)
    #
    #
    # def get_cell(self, x_world, y_world):
    #     return self.get_cell_helper(x_world, self.center_grid_x), self.get_cell_helper(y_world, self.center_grid_y)
    #
    #
    # def travelable_cell(self, x_grid, y_grid, grid_value):
    #     return self.in_bounds(x_grid, y_grid, self.grid_size, self.grid_size) and self.grid[x_grid, y_grid] < grid_value


    def plan(self):
        while True:
            data = self.rclient.sense()
            print data

            # print "grid position: (" + str(self.grid_x) + ", " + str(self.grid_y) + ") [grid center: (" + str(self.center_grid_x) + ", " + str(self.center_grid_y) + ")]"

            world_x = data[0]
            world_y = data[1]

            world_dir_x = data[2]
            world_dir_y = data[3]

            # right
            world_right_obst = data[4]

            # center
            world_center_obst = data[5]

            #left
            world_left_obst = data[6]


            # print "Obstacles: " + str((world_right_obst, world_center_obst, world_left_obst))

            obst_list = [world_right_obst, world_center_obst, world_left_obst]

            theta = numpy.degrees(-45)
            c, s = numpy.cos(theta), numpy.sin(theta)
            R_ccw = numpy.array(((c, -s), (s, c)))

            theta = numpy.degrees(45)
            c, s = numpy.cos(theta), numpy.sin(theta)
            R_cw = numpy.array(((c, -s), (s, c)))

            robot_dir = numpy.array([world_dir_x, world_dir_y])

            center_dir = robot_dir
            left_dir = numpy.matmul(R_ccw, robot_dir)
            right_dir = numpy.matmul(R_cw, robot_dir)

            dir_list = [right_dir, center_dir, left_dir]

            # min_obst = min(obst_list)
            max_obst = max(obst_list)
            min_obst = max_obst

            if max_obst < 0:
                closest_obst_dir = numpy.array([0, 0])
            else:
                for i in range(0, 3):
                    current_obst = obst_list[i]
                    print i
                    if current_obst < min_obst and current_obst > 0:
                        min_obst = current_obst
                min_obst_index = obst_list.index(min_obst)
                closest_obst_dir = dir_list[min_obst_index]

                print min_obst
                print min_obst_index

            goal_pos = numpy.array([self.goal_x, self.goal_y])

            robot_pos = numpy.array([world_x, world_y])

            goal_dir = goal_pos - robot_pos

            if numpy.linalg.norm(goal_dir) < 15:
                break

            attraction_vec = self.attraction_factor * (goal_dir / numpy.linalg.norm(goal_dir))

            repulsion_force = self.repulsion_factor / pow(float(min_obst) / 100.0, 2)

            if max_obst > 0:
                repulsion_vec = repulsion_force * (closest_obst_dir / numpy.linalg.norm(closest_obst_dir))
            else:
                repulsion_vec = numpy.array([0, 0])

            movement_dir = attraction_vec - repulsion_vec
            movement_dir_norm = movement_dir / numpy.linalg.norm(movement_dir)

            # print movement_dir_norm

            dot = numpy.dot(movement_dir_norm, robot_dir)
            cross = numpy.cross(movement_dir_norm, robot_dir)



            collision = True
            if world_right_obst < 0 and world_center_obst < 0 and world_left_obst < 0:
                collision = False

            # if dot < 0.9 and collision:
            #     if cross > 0:
            #         self.state = 'cw'
            #     else:
            #         self.state = 'ccw'
            # else:
            #     self.state = 'forward'

            rotation_speed = 300
            forward_speed = 300

            # if self.state == 'cw':
            #     self.speed_right = rot_speed
            #     self.speed_left = -350
            # elif self.state == 'ccw':
            #     self.speed_right = 0
            #     self.speed_left = rot_speed
            # elif self.state == 'forward':

            # print abs(dot)

            if dot > 0.9:
                self.speed_right = forward_speed
                self.speed_left = forward_speed
            else:
                if cross > 0:
                    self.speed_right = rotation_speed
                    self.speed_left = 0
                else:
                    self.speed_right = 0
                    self.speed_left = rotation_speed

            self.rclient.drive(self.speed_left, self.speed_right)
            time.sleep(0.1)

        self.rclient.terminate()

    # def drive(self):
    #     while not self.state is 'done':
    #         self.rclient.drive(self.speed_left, self.speed_right)
    #         time.sleep(0.2)


    def go(self, goal_x, goal_y):
        # (grid_goal_x, grid_goal_y) = self.get_cell(goal_x, goal_y)
        self.goal_x = goal_x
        self.goal_y = goal_y
        # self.grid_goal_x = goal_x
        # self.grid_goal_y = goal_y
        # self.populate_grid(self.grid_goal_x, self.grid_goal_y)
        self.plan()

        # self.plan_thread = threading.Thread(target=self.plan)
        # self.plan_thread.start()
        #
        # self.drive_thread = threading.Thread(target=self.drive)
        # self.drive_thread.start()


if __name__=='__main__':
    planner = Planner(cm_per_unit=20, grid_size_cm=400)
    planner.go(0, 0)

