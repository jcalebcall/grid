#!/usr/bin/env python

import OpenGL
OpenGL.ERROR_CHECKING = False
from OpenGL.GL import *
from OpenGL.GLUT import *
from OpenGL.GLU import *
from numpy import zeros

grid = None

def draw_grid():
    glClear(GL_COLOR_BUFFER_BIT)
    # This assumes you are using a numpy array for your grid
    width, height = grid.shape
    glRasterPos2f(-1, -1)
    # ras_x = -.95
    # ras_y = -.95
    # # 50 x 60
    # for i in range(70, 130, 1):
    #     glRasterPos2f(ras_x, ras_y);
    #     for j in range(70, 120, 1):
    #         if grid[i][j] < .95:
    #             glColor3f(0.0, 0.0, 1.0)
    #         else:
    #             glColor3f(1.0, 0.0, 0.0)
    #         glutBitmapString(GLUT_BITMAP_TIMES_ROMAN_10, ("%.2f" % grid[i][j]) + '  ')
    #     ras_x = -.95
    #     ras_y += .03

    glDrawPixels(width, height, GL_LUMINANCE, GL_FLOAT, grid)
    glFlush()
    glutSwapBuffers()

def update_grid(new_grid):
    global grid
    grid = new_grid

def init_window(width, height):
    global window
    global grid
    grid = zeros((width, height))
    glutInit(())
    glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_ALPHA | GLUT_DEPTH)
    glClearColor(0.0, 0.0, 0.0, 1.0)
    glutInitWindowSize(width, height)
    glutInitWindowPosition(0, 0)
    window = glutCreateWindow("Grid filter")
    glutDisplayFunc(draw_grid)
    glMatrixMode(GL_PROJECTION)
    glLoadIdentity()
    glMatrixMode(GL_MODELVIEW)
    glLoadIdentity()
    #glutMainLoop()

# vim: et sw=4 sts=4