import pygame
from pygame.locals import *
import numpy as np
from OpenGL.GL import *
from OpenGL.GLU import *
from OpenGL.GLUT import glutSolidTeapot, glutInit
glutInit()
from Auxs import OBJ, translationMatrix

def sumtup(a, b):
    return tuple(map(sum, zip(a, b)))

parametric = []
for t in np.linspace(-25, 25, 400):
    parametric.append((np.cos(t/5), ((t/ np.pi))/2 , 2*np.cos(t / np.pi)))


def drawCoord(origin=(0, 0, 0)):

    direccion = (1,0,0)
    magnitud = np.linalg.norm(direccion)
    direccion2 = np.cross(direccion / magnitud, (0, 1, 0))
    magnitud2 = np.linalg.norm(direccion2)
    direccion3 = np.cross(direccion / magnitud, direccion2 / magnitud2)
    magnitud3 = np.linalg.norm(direccion3)
    glLineWidth(4)
    glBegin(GL_LINES)
    glColor(0.5, 0, 0)
    glVertex(origin)
    glVertex(sumtup(tuple(direccion), origin))
    glColor(0, 1, 0)
    glVertex(origin)
    glVertex(sumtup(tuple(direccion2), origin))
    glColor(0, 0, 1)
    glVertex(origin)
    glVertex(sumtup(tuple(direccion3), origin))
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
    glLineWidth(1)
    # drawCoord()
    glBegin(GL_TRIANGLE_STRIP)
    points = ((-1, 0, 0), (0, 0, 1), (1, 0, 0))
    glColor3f(1.0, 0.0, 0.0)
    [glVertex3fv(i) for i in points]
    glEnd()


def drawGrid():
    glLineWidth(1)
    glBegin(GL_LINES)
    glColor3f(.5, .5, .5)
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
    glTranslatef(0.0,0, -10)


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

        glTranslatef(parametric[iterar][0], parametric[iterar][1], parametric[iterar][2])
        direccion = np.subtract(parametric[iterar+1], parametric[iterar])
        magnitud = np.linalg.norm(np.subtract(parametric[iterar + 1], parametric[iterar]))
        direccion2 = np.cross(direccion/magnitud, (1, 0, 0))
        magnitud2 = np.linalg.norm(direccion2)
        direccion3 = np.cross(direccion/magnitud, direccion2/magnitud2)
        magnitud3 = np.linalg.norm(direccion3)

        glMultMatrixf(np.hstack((direccion,     0,
                                 direccion2,    0,
                                 direccion3,    0,
                                 0, 0,  0,      1)))
        glutSolidTeapot(2)
        drawTriangle()
        glPopMatrix()


        drawLine(direccion3 / magnitud3, parametric[iterar])
        drawLine(direccion2 / magnitud2, parametric[iterar])
        drawLine(direccion/magnitud, parametric[iterar])


        drawGrid()
        iterar = iterar + 1 if iterar < len(parametric)-2 else 0

        pygame.display.flip()

        pygame.time.wait(10)


main()