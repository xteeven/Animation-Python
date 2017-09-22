import numpy as np
from OpenGL.GL import *
from OpenGL.GLU import *
import pygame
from pygame.locals import *

def moveInCurve(curve, i):

    # Performs both rotation and translation
    # by calculating the direction between two points
    # and then calculating the second and third axis
    # with cross product between vectors.

    vectorx = np.subtract(curve[i + 1], curve[i])
    modulex = np.linalg.norm(np.subtract(curve[i + 1], curve[i]))
    vectory = np.cross(vectorx / modulex, (1, 0, 0))
    moduley = np.linalg.norm(vectory)
    vectorz = np.cross(vectorx / modulex, vectory / moduley)
    modulez = np.linalg.norm(vectorz)

    # Next it builds the Mult Matrix to apply in the object(s)

    glMultMatrixf(np.hstack((vectorx / modulex, 0,
                             vectory / moduley, 0,
                             vectorz, 0,
                             curve[i], 1)))

def interpcurve(N, curve):

    # This function performs an interpolation based in the length
    # of the curve. First it calculates the length, next the values are normalized
    # in a range (0, 1), then a cumulative vector is built, and after a few adjust
    # in the end and beggining of the vector, it is calculated the new points by using
    # a "search" in the cumulative values, returning the index of each one of the values.

    N = np.transpose(np.linspace(0, 1, N))
    n = len(curve)
    curve = np.array(curve)
    pstart = curve[0, :]
    pend = curve[-1, :]
    curveLen = (np.sum(np.diff(curve, axis=0)**2, axis=1))**(1/2)
    curveLen = curveLen/np.sum(curveLen)
    cumLen = np.append(0, np.cumsum(curveLen))
    tbins= np.digitize(N, cumLen)
    tbins[np.where(tbins <= 0 | (N <= 0))] = 1
    tbins[np.where(tbins >= n | (N >= 1))] = n - 1
    s = np.divide((N - cumLen[tbins]), curveLen[tbins-1])
    pt = curve[tbins, :] + np.multiply((curve[tbins, :] - curve[tbins-1, :]), (np.vstack([s]*3)).T)
    return [tuple(i) for i in pt]

def sumtup(a, b):

    # simple function to sum 3d points

    return tuple(map(sum, zip(a, b)))


def mouseMove():

    # handles the mouse events to move the camera

    mouse = pygame.mouse.get_rel()
    if pygame.mouse.get_pressed()[1] & ((mouse[0] != 0) | (mouse[0] != 0)):
        vel = np.power(mouse[0] ** 2 + mouse[1] ** 2, 1 / 2.0)
        glRotatef(int(np.abs(vel)), mouse[1], 0, mouse[0])


def ketboardMove():

    # handles the keyboard events to move the camera
    # in the viewer direction

    keyboard = pygame.key.get_pressed()
    modelView = glGetFloatv(GL_MODELVIEW_MATRIX)
    if keyboard[K_LEFT]:
        glTranslatef(modelView[0][0]/2.0,
                     modelView[1][0]/2.0,
                     modelView[2][0]/2.0)

    if keyboard[K_RIGHT]:
        glTranslatef(modelView[0][0]*(-1)/2.0,
                     modelView[1][0]*(-1)/2.0,
                     modelView[2][0]*(-1)/2.0)

    if keyboard[K_UP]:
        glTranslatef(modelView[0][2]*(-1),
                     modelView[1][2]*(-1),
                     modelView[2][2]*(-1))

    if keyboard[K_DOWN]:
        glTranslatef(modelView[0][2],
                     modelView[1][2],
                     modelView[2][2])

