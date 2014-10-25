#!/usr/bin/python -tt

# An incredibly simple agent.  All we do is find the closest enemy tank, drive
# towards it, and shoot.  Note that if friendly fire is allowed, you will very
# often kill your own tanks with this code.

#################################################################
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
import random

from bzrc import BZRC, Command

class Agent(object):
    """Class handles all command and control logic for a teams tanks."""

    def __init__(self, bzrc):
        """ Move timer contains move start time and move threshold value. Likewise for shoot timer"""
        # A 0 as the third value in a moveTimer tuple indicates moving, 1 indicates turning
        self.moveTimer = {}
        self.prevAngles = {}
        self.shootTimer = {}
        self.bzrc = bzrc
        self.constants = self.bzrc.get_constants()
        self.commands = []

    def tick(self, time_diff):
        """Some time has passed; decide what to do next."""
        mytanks, othertanks, flags, shots = self.bzrc.get_lots_o_stuff()
        self.mytanks = mytanks
        self.othertanks = othertanks
        self.flags = flags
        self.shots = shots
        self.enemies = [tank for tank in othertanks if tank.color !=
                        self.constants['team']]

        self.commands = []

        for i in range(10):
            tank = mytanks[i]
            # Initializing timers
            if tank.index not in self.moveTimer:
                self.initTimers(tank)
            # Check if a tank is turning
            elif tank.index in self.prevAngles:
                self.checkTurnAngle(tank)
            # Checking timers
            else:
                self.checkTimers(tank)

        results = self.bzrc.do_commands(self.commands)

    def initTimers(self, tank):
        # Initializing stuff
        mThreshold = self.createMoveThreshold()
        sThreshold = self.createShootThreshold()
        currentTime = time.time()
        self.moveTimer[tank.index] = (currentTime, mThreshold)
        self.shootTimer[tank.index] = (currentTime, sThreshold)

        # Add command to start moving tank
        command = Command(tank.index, 1, 0, False)
        self.commands.append(command)

    def checkTurnAngle(self, tank):
        # check if the tank has turned 60 degrees
        # Angles are in radians between -pi and pi
        prevAngle = self.prevAngles[tank.index]
        currentAngle = tank.angle

        # Use absolute values to compute if we have turned left 60 degrees
        # 60 degrees = pi / 3
        turnDistance = math.fabs(math.fabs(currentAngle) - math.fabs(prevAngle))

        # pi/4 rather than pi/3 because it takes a second for the tank to stop turning
        if turnDistance >= (math.pi / 4):
            # case where we have turned 60 degrees
            # stop turning and start moving forward again, resetting the move timer
            command = Command(tank.index, 0, 0, False)
            self.commands.append(command)
            command = Command(tank.index, 1, 0, False)
            self.commands.append(command)

            # update threshold to new random moving threshold
            threshold = self.createMoveThreshold()

            # store tuple in dictionary
            self.moveTimer[tank.index] = (time.time(), threshold)

            # remove tank from angle dic because it is no longer turning
            self.prevAngles.pop(tank.index, None)

    def checkTimers(self, tank):
        """ Check if elapsed time has passed threshold values for tank move and tank shoot"""
        current_time = time.time()
        prev_time = self.moveTimer[tank.index][0]
        threshold = self.moveTimer[tank.index][1]
        if current_time - prev_time > threshold:
            self.doMoveTimerExpired(tank)

        # Check the shooting threshold
        prev_time = self.shootTimer[tank.index][0]
        threshold = self.shootTimer[tank.index][1]
        if current_time - prev_time > threshold:
            self.doShootTimerExpired(tank)

    def doMoveTimerExpired(self, tank):
        # Add command to stop moving
        command = Command(tank.index, 0, 0, False)
        self.commands.append(command)

        # Add command to start turning the tank
        command = Command(tank.index, 0, 1, False)
        self.commands.append(command)

        #store the current angle of the tank
        currentAngle = tank.angle
        self.prevAngles[tank.index] = currentAngle

    def doShootTimerExpired(self, tank):
        # store the time we shoot
        currentTime = time.time()

        # Issue a command to shoot
        self.bzrc.shoot(tank.index)

        # create new threshold
        threshold = self.createShootThreshold()

        # store tuple in dictionary
        self.shootTimer[tank.index] = (currentTime, threshold)

    def createMoveThreshold(self):
        return random.uniform(3, 8)

    def createShootThreshold(self):
        return random.uniform(1.5, 2.5)

def main():
    # Process CLI arguments.
    try:
        execname, host, port = sys.argv
    except ValueError:
        execname = sys.argv[0]
        print >>sys.stderr, '%s: incorrect number of arguments' % execname
        print >>sys.stderr, 'usage: %s hostname port' % sys.argv[0]
        sys.exit(-1)

    # Connect.
    #bzrc = BZRC(host, int(port), debug=True)
    bzrc = BZRC(host, int(port))

    agent = Agent(bzrc)

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
