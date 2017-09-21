import pygame
from pygame.locals import *
import numpy as np
from OpenGL.GL import *
from OpenGL.GLU import *
from Auxs import *

# Drawing a Curve
parametric = []
for t in np.linspace(-5, 5, 400):
    parametric.append((1*np.cos(t), t**2*np.sin(t), t))

def sumtup(a, b):
    return tuple(map(sum, zip(a, b)))


def drawCoord(origin=(0, 0, 0)):

    axis1 = (1,0,0)
    axis1m = np.linalg.norm(axis1)
    axis2 = np.cross(axis1 / axis1m, (0, 0, 1))
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
    glLineWidth(1)
    # glBegin(GL_LINE_STRIP)
    glBegin(GL_POINTS)
    # glLineWidth(2)
    glColor3f(1.0, 1.0, 1.0)
    for t in range(len(Curve)):
        glVertex3fv(Curve[t])
    glEnd()


def drawTriangle():
    drawCoord()
    glPushMatrix()
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


def drawGrid():
    glLineWidth(1)
    glBegin(GL_LINES)
    glColor3f(.2, .2, .2)
    for i in range(-20, 20):
        [glVertex3fv((i/2.0, j/2.0, -5)) for j in range(-20, 20)]
    for i in range(-20, 20):
        [glVertex3fv((j / 2.0, i / 2.0, -5)) for j in range(-20, 20)]
    glEnd()


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


def setup():
    pygame.init()
    display = (800,600)
    pygame.display.set_mode(display, DOUBLEBUF | OPENGL)
    glMatrixMode(GL_MODELVIEW)
    gluPerspective(45, (display[0]/display[1]), 0.1, 250.0)
    glTranslatef(0.0, 0, -10)


def drawLine(dir=[1, 1, 1], pos=(0, 0, 0)):
    glLineWidth(1)
    glBegin(GL_LINES)
    glColor3f(.5, .5, .5)
    glVertex3fv(pos)
    glVertex3fv((dir[0]+pos[0], dir[1]+pos[1], dir[2]+pos[2]))
    glEnd()


def main():

    setup()
    iterar = 0

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                quit()


        mouseMove()
        ketboardMove()
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glTranslatef(0.0, 0.0, 0)

        drawCurve(parametric)

        glPushMatrix()

        vectorx = np.subtract(parametric[iterar+1], parametric[iterar])
        modulex = np.linalg.norm(np.subtract(parametric[iterar + 1], parametric[iterar]))
        vectory = np.cross(vectorx/modulex, (1, 0, 0))
        moduley = np.linalg.norm(vectory)
        vectorz = np.cross(vectorx/modulex, vectory/moduley)
        modulez = np.linalg.norm(vectorz)
        glMultMatrixf(np.hstack((vectorx/modulex,    0,
                                vectory/moduley,    0,
                                vectorz,    0,
                                parametric[iterar],      1)))


        drawTriangle()

        glPopMatrix()

        drawLine(vectorz / modulez, parametric[iterar])
        drawLine(vectory / moduley, parametric[iterar])
        drawLine(vectorx/modulex, parametric[iterar])


        drawGrid()

        pygame.display.flip()
        iterar = iterar + 1 if iterar < len(parametric) - 2 else 0
        pygame.time.wait(10)


main()