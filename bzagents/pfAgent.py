#!/usr/bin/python -tt

# An incredibly simple agent.  All we do is find the closest enemy tank, drive
# towards it, and shoot.  Note that if friendly fire is allowed, you will very
# often kill your own tanks with this code.

# ################################################################
# NOTE TO STUDENTS
# This is a starting point for you.  You will need to greatly
# modify this code if you want to do anything useful.  But this
# should help you to know how to interact with BZRC in order to
# get the information you need.
#
# After starting the bzrflag server, this is one way to start
# this code:
# python agent0.py [hostname] [port]
#
# Often this translates to something like the following (with the
# port name being printed out by the bzrflag server):
# python agent0.py localhost 49857
#################################################################

import sys
import math
import time

from bzrc import BZRC, Command
import matplotlib.pyplot as plt
from pylab import *
import math
import random


class Agent(object):
    """Class handles all command and control logic for a teams tanks."""

    def __init__(self, bzrc):
        self.bzrc = bzrc
        self.constants = bzrc.get_constants()
        self.commands = []
        self.flags = bzrc.get_flags()
        self.obstacles = bzrc.get_obstacles()
        self.hasFlag = 0

        # 0 - Blue
        # 1 - Purple
        # 2 - Green
        # 3 - Red
        color = (self.constants['team'])
        if color == 'blue':
            self.ourBase = 0
        elif color == 'purple':
            self.ourBase = 1
        elif color == 'green':
            self.ourBase = 2
        else:
            self.ourBase = 3

        self.opponentBase = 2

    def plot_fields(self):
        # Draw the visualization for the map
        self.plot_single(self.calc_attractive_v, self.bzrc.get_obstacles(), 'attractive2.png')
        self.plot_single(self.calc_repulsive_v, self.bzrc.get_obstacles(), 'repulsive2.png')
        self.plot_single(self.calc_tangential_v, self.bzrc.get_obstacles(), 'tangential2.png')
        self.plot_single(self.combined_fields, self.bzrc.get_obstacles(), 'combined2.png')

    def tick(self, time_diff):
        """Some time has passed; decide what to do next."""
        mytanks, othertanks, flags, shots = self.bzrc.get_lots_o_stuff()
        self.mytanks = mytanks
        self.othertanks = othertanks
        self.flags = flags
        self.shots = shots
        self.enemies = [tank for tank in othertanks if tank.color !=
                        self.constants['team']]
        self.obstacles = self.bzrc.get_obstacles()

        self.commands = []

        for i in range(10):
            tank = mytanks[i]
            if (tank.flag != '-'):
                self.hasFlag = 1
                break
            else:
                self.hasFlag = 0

        for i in range(10):
            tank = mytanks[i]
            if i < 7:
                self.pf_attack(tank)
            else:
                self.pf_defend(tank)

        results = self.bzrc.do_commands(self.commands)

    def pf_attack(self, tank):
        # calculate attractive and repulsive vectors
        (attractive_x, attractive_y) = self.calc_attractive_v(tank.x, tank.y)
        (repulsive_x, repulsive_y) = self.calc_repulsive_v(tank.x, tank.y)

        f_del_x = attractive_x + repulsive_x
        f_del_y = attractive_y + repulsive_y
        self.move_to_position(tank, f_del_x, f_del_y)

    def pf_defend(self, tank):
        # calculate attractive and repulsive vectors
        (tangential_x, tangential_y) = self.calc_tangential_v(tank.x, tank.y)

        f_del_x = tangential_x
        f_del_y = tangential_y
        self.move_to_position(tank, f_del_x, f_del_y)

    def move_to_position(self, tank, del_x, del_y):
        """Set command to move to given coordinates."""
        target_angle = math.atan2(del_y,
                                  del_x)
        relative_angle = self.normalize_angle(target_angle - tank.angle)
        command = Command(tank.index, self.vectorMagnitude(del_x, del_y), 2 * relative_angle, True)
        self.commands.append(command)

    def calc_attractive_v(self, x, y):
        # set up an attractive field for each flag
        list_del_x = []
        list_del_y = []

        # set up attractive field for enemy flag or our base
        if (self.hasFlag == 1):
            flag_to_get = self.ourBase
        else:
            flag_to_get = self.opponentBase

        flag = self.flags[flag_to_get]
        del_x = None
        del_y = None

        # position and radius of flag
        xG = flag.x
        yG = flag.y
        r = 1

        # Calculate distance and angle between our tank and the flag
        d = math.sqrt((xG - x) ** 2 + (yG - y) ** 2)
        a = math.atan2(yG - y, xG - x)

        # Set our spread and scaling constant
        s = 50
        o = 1.0

        # Set our delta x and y accordingly
        if d < r:
            del_x = 0
            del_y = 0
        elif r <= d <= (s + r):
            del_x = o * (d - r) * math.cos(a)
            del_y = o * (d - r) * math.sin(a)
        elif d > (s + r):
            del_x = o * s * math.cos(a)
            del_y = o * s * math.sin(a)

        list_del_x.append(del_x)
        list_del_y.append(del_y)

        # Combine the deltas for finished attractive vector
        f_del_x = 0.0
        f_del_y = 0.0
        for del_x in list_del_x:
            f_del_x += del_x
        for del_y in list_del_y:
            f_del_y += del_y

        # return the final attractive vector
        return f_del_x, f_del_y

    def calc_repulsive_v(self, x, y):
        # set up a repulsive field for each obstacle and enemy tank
        list_del_x = []
        list_del_y = []

        # obstacles first
        #counter = 0
        for obstacle in self.obstacles:
            #if counter >= 1:
                #break
            #counter += 1
            del_x = None
            del_y = None

            # Calculate bounding box of obstacle
            max_x = -10000
            max_y = -10000
            min_x = 10000
            min_y = 10000
            for (x_corn, y_corn) in obstacle:

                # Calculate max values
                if x_corn > max_x:
                    max_x = x_corn
                if y_corn > max_y:
                    max_y = y_corn

                # Calculate min values
                if x_corn < min_x:
                    min_x = x_corn
                if y_corn < min_y:
                    min_y = y_corn

            # Calculate radius of bounding box
            side_a = math.fabs((math.fabs(max_x) - math.fabs(min_x)) / 2.0)
            side_b = math.fabs((math.fabs(max_y) - math.fabs(min_y)) / 2.0)
            if ((math.copysign(1, max_x) != math.copysign(1, min_x)) or (math.copysign(1, max_y) != math.copysign(1, min_y))):
                side_a = (max_x - min_x) / 2.0
                side_b = (max_y - min_y) / 2.0
            #r = math.sqrt((side_a ** 2) + (side_b ** 2))
            r = 1

            # position of the obstacle
            xO = min_x + side_a
            yO = min_y + side_b

            # Calculate distance and angle between our tank and the obstacle
            d = math.sqrt((xO - x) ** 2 + (yO - y) ** 2)
            a = math.atan2(yO - y, xO - x)

            # Set our spread and scaling constant
            s = 75
            b = 1.0

            # Set our delta x and y accordingly
            if d < r:
                del_x = 1 #(-math.copysign(1, math.cos(a))) * 10000  # big number
                del_y = 1 #(-math.copysign(1, math.sin(a))) * 10000
            elif r <= d <= (s + r):
                del_x = -b * (s + r - d) * math.cos(a)
                del_y = -b * (s + r - d) * math.sin(a)
            elif d > (s + r):
                del_x = 0
                del_y = 0

            list_del_x.append(del_x)
            list_del_y.append(del_y)

        # Combine the deltas for finished repulsive vector
        f_del_x = 0.0
        f_del_y = 0.0
        for del_x in list_del_x:
            f_del_x += del_x
        for del_y in list_del_y:
            f_del_y += del_y

        # return the final attractive vector
        return (f_del_x, f_del_y)

    def calc_tangential_v(self, x, y):
        # set up an attractive field for each flag
        list_del_x = []
        list_del_y = []

        flag = self.flags[self.ourBase]
        del_x = None
        del_y = None

        # position and radius of flag
        xG = flag.x
        yG = flag.y
        r = 1

        # Calculate distance and angle between our tank and the flag
        d = math.sqrt((xG - x) ** 2 + (yG - y) ** 2)
        a = math.atan2(yG - y, xG - x) + 90

         # Set our spread and scaling constant
        s = 75
        b = 1.0

        # Set our delta x and y accordingly
        if d < r:
            del_x = (-math.copysign(1, math.cos(a))) * 10000  # big number
            del_y = (-math.copysign(1, math.sin(a))) * 10000
        elif r <= d <= (s + r):
            del_x = -b * (s + r - d) * math.cos(a)
            del_y = -b * (s + r - d) * math.sin(a)
        elif d > (s + r):
            del_x = 0
            del_y = 0

        list_del_x.append(del_x)
        list_del_y.append(del_y)

        # attractive portion
        a = math.atan2(yG - y, xG - x)
        s = 10
        # Set our delta x and y accordingly
        if d < r:
            del_x = 0
            del_y = 0
        elif r <= d <= (s + r):
            del_x = b * (d - r) * math.cos(a)
            del_y = b * (d - r) * math.sin(a)
        elif d > (s + r):
            del_x = b * s * math.cos(a)
            del_y = b * s * math.sin(a)

        list_del_x.append(del_x)
        list_del_y.append(del_y)

        # Combine the deltas for finished repulsive vector
        f_del_x = 0.0
        f_del_y = 0.0
        for del_x in list_del_x:
            f_del_x += del_x
        for del_y in list_del_y:
            f_del_y += del_y

        # return the final attractive vector
        return (f_del_x, f_del_y)

    def combined_fields(self, x, y):
        (attractive_x, attractive_y) = self.calc_attractive_v(x, y)
        (repulsive_x, repulsive_y) = self.calc_repulsive_v(x, y)

        return attractive_x + repulsive_x, attractive_y + repulsive_y

    def normalize_angle(self, angle):
        """Make any angle be between +/- pi."""
        angle -= 2 * math.pi * int(angle / (2 * math.pi))
        if angle <= -math.pi:
            angle += 2 * math.pi
        elif angle > math.pi:
            angle -= 2 * math.pi
        return angle

    def vectorMagnitude(self, vx, vy):
        return math.sqrt((vx ** 2) + (vy ** 2))

    """ *****************************
    Plotting code
     *******************************"""

    def show_obstacle(self, plot, points):
        """Draw a polygon. Points is a list if [x,y] tuples
        """
        for p1, p2 in zip(points, [points[-1]] + list(points)):
            plot.plot([p1[0], p2[0]], [p1[1], p2[1]], 'b')


    def show_arrows(self, plot, potential_func, xlim=(-400, 400), ylim=(-400, 400), res=20):
        """
        Arguments:
            fns: a list of potential field functions
            xlim, ylim: the limits of the plot
            res: resolution for (spacing between) arrows
        """
        plot.set_xlim(xlim)
        plot.set_ylim(ylim)
        for x in range(xlim[0], xlim[1] + res, res):
            for y in range(ylim[0], ylim[1] + res, res):
                dx, dy = potential_func(x, y)
                dx /= (res/5)
                dy /= (res/5)
                if dx + dy == 0: continue
                plot.arrow(x, y, dx, dy, head_width=res / 7.0, color='red', linewidth=.3)


    def plot_single(self, potential_func, obstacles, filename, xlim=(-400, 400), ylim=(-400, 400)):
        """Plot a potential function and some obstacles, and write the resulting
        image to a file"""
        print "Generating", filename
        fig = plt.figure()
        plot = plt.subplot(111)
        self.show_arrows(plot, potential_func, xlim=xlim, ylim=ylim)
        for obstacle in obstacles:
            self.show_obstacle(plot, obstacle)
        fig.savefig(filename, format='png')


def main():
    # Process CLI arguments.
    try:
        execname, host, port = sys.argv
    except ValueError:
        execname = sys.argv[0]
        print >> sys.stderr, '%s: incorrect number of arguments' % execname
        print >> sys.stderr, 'usage: %s hostname port' % sys.argv[0]
        sys.exit(-1)

    # Connect.
    #bzrc = BZRC(host, int(port), debug=True)
    bzrc = BZRC(host, int(port))

    agent = Agent(bzrc)

    agent.plot_fields()

    prev_time = time.time()

    # Run the agent
    try:
        while True:
            time_diff = time.time() - prev_time
            agent.tick(time_diff)
    except KeyboardInterrupt:
        print "Exiting due to keyboard interrupt."
        bzrc.close()


if __name__ == '__main__':
    main()

# vim: et sw=4 sts=4
