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
sys.path.append("/home/caleb/PycharmProjects/grid/visualization")
from bzrc import BZRC, Command
import show_grid

import math
import time
import matplotlib.pyplot as plt
from pylab import *
import math
import random
from numpy import *


class Agent(object):
    """Class handles all command and control logic for a teams tanks."""

    def __init__(self, bzrc):
        self.bzrc = bzrc
        self.commands = []

        # Set tank move-to goals
        self.goal_list = []
        self.init_goals()
        self.current_goal_index = 0

        # Initialize grid as numpy array
        self.grid = np.zeros((800, 800))
        self.grid.fill(.7)

        # Initialize visualization
        show_grid.init_window(800, 800)

    def init_goals(self):
        is_odd = True
        for i in range(8):
            for j in range(8):
                if is_odd:
                    self.goal_list.append((-350 + 100 * j, 350 - 100 * i))
                else:
                    self.goal_list.append((350 - 100 * j, 350 - 100 * i))
            is_odd = not is_odd

    def plot_fields(self):
        # Draw the visualization for the map
        self.plot_single(self.calc_attractive_v, 'attractive.png')
        #self.plot_single(self.calc_repulsive_v, self.bzrc.get_obstacles(), 'repulsive2.png')
        self.plot_single(self.calc_tangential_v, self.bzrc.get_obstacles(), 'tangential.png')
        #self.plot_single(self.combined_fields, self.bzrc.get_obstacles(), 'combined2.png')

    def tick(self, time_diff):
        """Some time has passed; decide what to do next."""
        mytanks = self.bzrc.get_mytanks()
        self.mytanks = mytanks


        tank = mytanks[0]
        pos, size, grid = self.bzrc.get_occgrid(0)
        self.process_grid(pos, size, grid)
        show_grid.update_grid(self.grid)
        show_grid.draw_grid()

        self.update_goal(tank)
        self.move_to_goal(tank)

        results = self.bzrc.do_commands(self.commands)

    def process_grid(self, pos, size, grid):
        #print 'at ' + str(pos[0]) + ',' + str(pos[1])
        #print 'size ' + str(size[0]) + 'x' + str(size[1])
        for i in range(len(grid)):
            for j in range(len(grid[i])):
                #print str(grid[i][j]),
                y = pos[1] + 400 + j
                x = pos[0] + 400 + i
                self.bayesian_filter(y, x, grid[i][j])
            #print '\n'

    def bayesian_filter(self, y, x, obs):
        p_occ = self.grid[y][x]
        p_obs_given_occ = .97
        p_not_obs_given_occ = .03

        p_not_occ = 1 - p_occ
        p_obs_given_not_occ = .10
        p_not_obs_given_not_occ = .90

        # Bayes Theorem p(a|b) = p(b|a) * p(a) / p(b)
        if obs == 1:
            self.grid[y][x] = p_obs_given_occ * p_occ / (p_occ * p_obs_given_occ + p_not_occ * p_obs_given_not_occ)
        else:
            self.grid[y][x] = p_not_obs_given_occ * p_occ / (p_occ * p_not_obs_given_occ + p_not_occ * p_not_obs_given_not_occ)

    def update_goal(self, tank):
        goal_x = self.goal_list[self.current_goal_index][0]
        goal_y = self.goal_list[self.current_goal_index][1]
        if (goal_x - 100) < tank.x < (goal_x + 100) \
            and (goal_y - 100) < tank.y < (goal_y + 100):
            # This the case where we update the goal
            self.current_goal_index += 1

    def move_to_goal(self, tank):
        print 'Moving to goal: ' + str(self.goal_list[self.current_goal_index][0]) + ', ' + str(self.goal_list[self.current_goal_index][1])
        # calculate attractive and repulsive vectors
        (attractive_x, attractive_y) = self.calc_attractive_v(tank.x, tank.y)
        #(tangential_x, tangential_y) = self.calc_tangential_v(tank.x, tank.y)

        f_del_x = attractive_x #+ tangential_x
        f_del_y = attractive_y #+ tangential_y
        self.move_to_position(tank, f_del_x, f_del_y)

    def move_to_position(self, tank, del_x, del_y):
        """Set command to move to given coordinates."""
        target_angle = math.atan2(del_y,
                                  del_x)
        relative_angle = self.normalize_angle(target_angle - tank.angle)
        command = Command(tank.index, self.vectorMagnitude(del_x, del_y), 2 * relative_angle, False)
        self.commands.append(command)

    def calc_attractive_v(self, x, y):
        # set up an attractive field for the goal
        del_x = None
        del_y = None

        # position and radius of goal
        xG = self.goal_list[self.current_goal_index][0]
        yG = self.goal_list[self.current_goal_index][1]
        r = 1

        # Calculate distance and angle between our tank and the goal
        d = math.sqrt((xG - x) ** 2 + (yG - y) ** 2)
        a = math.atan2(yG - y, xG - x)

        print 'distance: ' + str(d)

        # Set our spread and scaling constant
        s = 30
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

        # return the final attractive vector
        return del_x, del_y

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
        list_del_x = []
        list_del_y = []
        for i in range(len(self.grid)):
            for j in range(len(self.grid[i])):
                if self.grid[i][j] > .99:
                    # Convert to world coordinates
                    yG = i - 400
                    xG = j - 400

                    del_x = None
                    del_y = None

                    r = 1

                    d = math.sqrt((xG - x) ** 2 + (yG - y) ** 2)
                    a = math.atan2(yG - y, xG - x) + 90

                     # Set our spread and scaling constant
                    s = 1
                    b = .2

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

    def show_arrows(self, plot, potential_func, xlim=(-400, 400), ylim=(-400, 400), res=20):
        """
        Arguments:
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


    def plot_single(self, potential_func, filename, xlim=(-400, 400), ylim=(-400, 400)):
        """Plot a potential function and some obstacles, and write the resulting
        image to a file"""
        print "Generating", filename
        fig = plt.figure()
        plot = plt.subplot(111)
        self.show_arrows(plot, potential_func, xlim=xlim, ylim=ylim)
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

    #agent.plot_fields()

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
