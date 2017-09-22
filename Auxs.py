import numpy as np
from OpenGL.GL import *
from OpenGL.GLU import *
import pygame
from pygame.locals import *

def rotateInCurve(curve, i):
    vectorx = np.subtract(curve[i + 1], curve[i])
    modulex = np.linalg.norm(np.subtract(curve[i + 1], curve[i]))
    vectory = np.cross(vectorx / modulex, (1, 0, 0))
    moduley = np.linalg.norm(vectory)
    vectorz = np.cross(vectorx / modulex, vectory / moduley)
    modulez = np.linalg.norm(vectorz)

    glMultMatrixf(np.hstack((vectorx / modulex, 0,
                             vectory / moduley, 0,
                             vectorz, 0,
                             curve[i], 1)))

def interpcurve(N, curve):
    # equally spaced in arclength

    N = np.transpose(np.linspace(0, 1, N))
    nt = N.size

    # number of points on the curve
    n = len(curve)
    pxy = np.array(curve)
    p1 = pxy[0,:]
    pend = pxy[-1,:]
    last_segment = np.linalg.norm(np.subtract(p1, pend))
    epsilon = 10*np.finfo(float).eps


    chordlen = (np.sum(np.diff(pxy,axis=0)**2,axis=1))**(1/2)
    # Normalize the arclengths to a unit total
    chordlen = chordlen/np.sum(chordlen)
    # cumulative arclength
    cumarc = np.append(0, np.cumsum(chordlen))
    tbins= np.digitize(N, cumarc)
    tbins[np.where(tbins<=0 | (N<=0))]=1
    tbins[np.where(tbins >= n | (N >= 1))] = n - 1

    s = np.divide((N - cumarc[tbins]), chordlen[tbins-1])
    pt = pxy[tbins, :] + np.multiply((pxy[tbins, :] - pxy[tbins-1, :]), (np.vstack([s]*3)).T)

    return [tuple(i) for i in pt]

def sumtup(a, b):
    return tuple(map(sum, zip(a, b)))


def mouseMove():
    mouse = pygame.mouse.get_rel()
    if pygame.mouse.get_pressed()[1] & ((mouse[0] != 0) | (mouse[0] != 0)):
        vel = np.power(mouse[0] ** 2 + mouse[1] ** 2, 1 / 2.0)
        glRotatef(int(np.abs(vel)), mouse[1], 0, mouse[0])


def ketboardMove():
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

