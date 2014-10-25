#!/usr/bin/env python

"""
README

This should get some of the boilerplate out of the way for you in visualizing
your potential fields.

Notably absent from this code is anything dealing with vectors, the
interaction of mutliple fields, etc. Your code will be a lot easier if you
define your potential fields in terms of vectors (angle, magnitude). For
plotting, however, you'll need to turn them back into dx and dy.
"""




from math import atan2, cos, sin, sqrt, pi


##### PLOTTING FUNCTIONS #####



#### TRIVIAL EXAMPLE FUNCTIONS ####


def random_field(x, y, res):
    """
    NOTE: Your potential field calculator should probably work in vectors
    (angle, magnitude), but you need to return dx, dy for plotting.

    Arguments:
        x:   the x position for which to calculate the potential field
        y:   the y position for which to calculate the potential field
        res: current plotting resolution (helpful for scaling down your
             vectors for display, so they don't all overlap each other)

    Returns:
        dx, dy: the change in x and y for the arrow to point.
    """

    return random.randint(-res, res), random.randint(-res, res)


def unidirectional(x, y, res):
    """Another simple example"""

    return res, res/2


def bidirectional(x, y, res):
    if x > 0:
        return res, res/2
    else:
        return -res, res/2