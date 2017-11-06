import numpy as np
from OpenGL.GL import *
from OpenGL.GLU import *
import pygame
from pygame.locals import *
from Auxs import *

def drawLine(dir=[1, 1, 1], pos=(0, 0, 0)):

    # Draw a test line to evaluate the directions
    # and positions previously calculated. (Only debuging)

    glLineWidth(1)
    glBegin(GL_LINES)
    glColor3f(.5, .5, .5)
    glVertex3fv(pos)
    glVertex3fv((dir[0] + pos[0], dir[1] + pos[1], dir[2] + pos[2]))
    glEnd()


def drawCoord(origin=(0, 0, 0)):

    # Draw coordinate systems in the given position

    axis1 = (1,0,0)
    axis1m = np.linalg.norm(axis1)
    axis2 = np.cross(axis1 / axis1m, (0, 1, 0))
    axis2m = np.linalg.norm(axis2)
    axis3 = np.cross(axis1 / axis1m, axis2 / axis2m)
    axis3m = np.linalg.norm(axis3)

    glLineWidth(4)
    glBegin(GL_LINES)
    glColor(0.5, 0, 0)
    glVertex(origin)
    glVertex(sumtup(tuple(axis1), origin))
    glColor(0, 1, 0)
    glVertex(origin)
    glVertex(sumtup(tuple(axis2), origin))
    glColor(0, 0, 1)
    glVertex(origin)
    glVertex(sumtup(tuple(axis3), origin))
    glEnd()


def drawCurve(Curve):

    # Draw a given curve (taking as argument a list of 3D points (tuples))

    glLineWidth(1)
    glBegin(GL_LINE_STRIP)
    # glBegin(GL_POINTS)
    # glLineWidth(2)
    glColor3f(1.0, 1.0, 1.0)
    for t in range(len(Curve)):
        glVertex3fv(Curve[t])
    glEnd()


def drawPlane():

    # Draw a simple plane to follow the path

    drawCoord()
    glPushMatrix()

    # Adjust the initial position of the plane

    glMultMatrixf(np.hstack((0, 0, -1/2.0, 0,
                             -1/2.0, 0, 0, 0,
                             0, -1/2.0, 0, 0,
                             0, -1/2.0, 0, 1)))

    glLineWidth(1)
    glBegin(GL_TRIANGLE_STRIP)
    points = ((-2, 2, 0), (-1, 0, 0), (-1, 2, 0),
              (0, 0, -1), (0, 2, -1), (0, 2, -1),
              (1, 2, 0), (0, 0, -1), (1, 0, 0),
              (2, 2, 0), (1, 2, 0))

    glColor3f(1.0, 0.0, 0.0)
    [glVertex3fv(i) for i in points]
    glEnd()
    glPopMatrix()


def drawGrid(zpos):

    glLineWidth(1)
    glBegin(GL_LINES)
    glColor3f(.2, .2, .2)
    for i in range(-20, 20):
        [glVertex3fv((i/2.0, j/2.0, zpos)) for j in range(-20, 20)]
    for i in range(-20, 20):
        [glVertex3fv((j / 2.0, i / 2.0, zpos)) for j in range(-20, 20)]
    glEnd()

def drawLink(x=1, y=1, z=1, color = (.2, 0, 0)):

    glPushMatrix()
    glMultMatrixf(np.hstack((x, 0, 0, 0,
                             0, y, 0, 0,
                             0, 0, z, 0,
                             0, 0, 0, 1)))
    vertices = (
        (.5, -.5, 0),
        (.5, .5, 0),
        (-.5, .5, 0),
        (-.5, -.5, 0),
        (.5, -.5, 1),
        (.5, .5, 1),
        (-.5, -.5, 1),
        (-.5, .5, 1))
    edges = (
        (0,1),
        (0,3),
        (0,4),
        (2,1),
        (2,3),
        (2,7),
        (6,3),
        (6,4),
        (6,7),
        (5,1),
        (5,4),
        (5,7)
    )
    glBegin(GL_TRIANGLE_FAN)
    points = (())
    # glBegin(GL_LINES)
    glColor3f(color[0], color[1], color[2])
    for edge in edges:
        for vertex in edge:
            glVertex3fv(vertices[vertex])
    glEnd()
    glPopMatrix()