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
import random
from errnames import get_error_name


def get_ip():
    from netifaces import interfaces, ifaddresses, AF_INET
    for interface in interfaces():
        try:
            for link in ifaddresses(interface)[AF_INET]:
                ip = str(link['addr'])
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

    def __init__(self, host, port, user_deprecate='', id_deprecate=''):
        self.ip = get_ip()
        # self.ip = '127.0.0.1'
        self.robot = (host, port)
        self.lock = threading.RLock()
        self.done = False
        self.sensors = [0.0, 0.0, 0.0, 0.0, 0.0]

    def connect(self):
        """ Connect to server and create processing thread """
        try:
            self.recv_thread = threading.Thread(target=self.recv_loop)
            self.recv_thread.start()
            return True
        except socket.error, e:
            reason = get_error_name(e[0])
            print "Socket Error: " + reason
        return False

    def recv_loop(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((self.ip, 9209))
        sock.setblocking(0)
        while not self.done:
            try:
                data, addr = sock.recvfrom(256)
                if len(data) == 0:
                    time.sleep(0.05)
                else:
                    # print "Received from '{}' data='{}'".format(addr,data)
                    try:
                        self.sensors = [float(s) for s in data.split()]
                    except ValueError:
                        pass
            except socket.error, e:
                # errnum=e[0]
                # if errnum!=errno.EAGAIN:
                #     reason=get_error_name(errnum)
                #     print "Socket Error ({}): {}".format(errnum,reason)
                time.sleep(0.05)
                y = 6

    def sendmsg(self, msg):
        with self.lock:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
                sock.sendto(msg, self.robot)
                return True
            except socket.error:
                return False

    def terminate(self):
        """ Call before your program ends, for a clean exit """
        self.done = True
        self.recv_thread.join()

    def drive(self, left, right):
        """ Make the robot move.  Send 2 integers for motors [-1000 : 1000] """
        self.sendmsg("{} {}".format(left, right))

    def sense(self):
        """ Get a list of sensor readings.  5 floating point values:  X,Y, 3 sonars """
        return self.sensors


#
#  Following code is a simple test main that allows to control the robot
#  from the keyboard, and see raw sersor readings on the screen
#
done = False
cmd = ''


def kbd():
    global cmd
    while cmd != 'q':
        cmd = sys.stdin.readline().strip()
    cmd = ''
    global done
    done = True


class Planner:
    def __init__(self, cm_per_unit, grid_size_cm):
        self.orig_attraction_factor = 2000
        self.attraction_factor = self.orig_attraction_factor
        self.orig_repulsion_factor = 350
        self.repulsion_factor = self.orig_repulsion_factor
        self.rclient = RClient("192.168.1.157", 2777)
        self.connected = False
        self.counter = 0
        if self.rclient.connect():
            self.connected = True
            self.state = 'forward'
            time.sleep(1)

    def plan(self):
        while True:
            data = self.rclient.sense()
            print data

            world_x = data[0]
            world_y = data[1]

            while world_x < -9000 or world_y < -9000:
                self.rclient.drive(-310, -310)
                data = self.rclient.sense()
                print data
                world_x = data[0]
                world_y = data[1]
                time.sleep(0.2)

            world_dir_x = data[2]
            world_dir_y = data[3]

            # right
            world_right_obst = data[4]

            # center
            world_center_obst = data[5]

            # left
            world_left_obst = data[6]

            obst_list = [world_right_obst, world_center_obst, world_left_obst]

            theta = numpy.degrees(-45)
            c, s = numpy.cos(theta), numpy.sin(theta)
            R_ccw = numpy.array(((c, -s), (s, c)))

            theta = numpy.degrees(45)
            c, s = numpy.cos(theta), numpy.sin(theta)
            R_cw = numpy.array(((c, -s), (s, c)))

            theta = numpy.degrees(45)
            c, s = numpy.cos(theta), numpy.sin(theta)
            R_bias = numpy.array(((c, -s), (s, c)))

            robot_dir = numpy.array([world_dir_x, world_dir_y])

            # center_dir = robot_dir
            center_dir = numpy.matmul(R_bias, robot_dir)
            left_dir = numpy.matmul(R_ccw, robot_dir)
            right_dir = numpy.matmul(R_cw, robot_dir)

            dir_list = [right_dir, center_dir, left_dir]

            # min_obst = min(obst_list)
            max_obst = max(obst_list)
            min_obst = max_obst

            front = False

            if max_obst < 0:
                closest_obst_dir = numpy.array([0, 0])
            else:
                for i in range(0, 3):
                    current_obst = obst_list[i]
                    if current_obst < min_obst and current_obst > 0:
                        min_obst = current_obst

                min_obst_index = obst_list.index(min_obst)

                if min_obst_index == 1:
                    if obst_list[0] < 0 and obst_list[2] > 0:
                        min_obst_index = 2
                    elif obst_list[2] < 0 and obst_list[0] > 0:
                        min_obst_index = 0
                    elif obst_list[2] > 0 and obst_list[0] > 0:
                        if obst_list[0] < obst_list[2]:
                            min_obst_index = 0
                        elif obst_list[2] < obst_list[0]:
                            min_obst_index = 2
                    else:
                        front = True

                closest_obst_dir = dir_list[min_obst_index]
                closest_obst_dist = obst_list[min_obst_index]

                if closest_obst_dist < 40:
                    self.repulsion_factor = 5000
                else:
                    self.repulsion_factor = self.orig_repulsion_factor

            goal_pos = numpy.array([self.goal_x, self.goal_y])
            robot_pos = numpy.array([world_x, world_y])
            goal_dir = goal_pos - robot_pos

            # if numpy.linalg.norm(goal_dir) < 80:
            #     self.attraction_factor = 4000
            # else:
            #     self.attraction_factor = self.orig_attraction_factor

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

            dot = numpy.dot(movement_dir_norm, robot_dir)
            cross = numpy.cross(movement_dir_norm, robot_dir)

            forward_speed = 260
            backward_rot_speed = 0
            forward_rot_speed = 350

            # if front is True and min_obst < 16:
            #     while world_right_obst < 0 and world_left_obst < 0:
            #         data = self.rclient.sense()
            #         print data
            #         world_right_obst = data[4]
            #         world_center_obst = data[5]
            #         world_left_obst = data[6]
            #         self.rclient.drive(-310, 310)
            # else:
            if max_obst > 0 and min_obst < 34:
                backward_rot_speed = -(forward_rot_speed - 70)

            if dot > 0.94:
                self.speed_right = forward_speed
                self.speed_left = forward_speed
            else:
                if cross > 0:
                    self.speed_right = forward_rot_speed
                    self.speed_left = backward_rot_speed
                else:
                    self.speed_right = backward_rot_speed
                    self.speed_left = forward_rot_speed

            self.rclient.drive(self.speed_left, self.speed_right)
            time.sleep(0.24)

    def go(self, goal_x, goal_y):
        self.goal_x = goal_x
        self.goal_y = goal_y
        self.plan()

    def end(self):
        self.rclient.terminate()


if __name__ == '__main__':
    planner = Planner(cm_per_unit=20, grid_size_cm=400)
    for i in range(1,10):
        planner.go(0, 0)
        planner.go(114, -62)
        # planner.go(-129, -156)
    planner.end()
