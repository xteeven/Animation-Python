import pygame
from pygame.locals import *
import numpy as np
from OpenGL.GL import *
from OpenGL.GLU import *
from Auxs import OBJ, translationMatrix

parametric = []
for t in range(50):
    parametric.append((np.sin(t / np.pi), ((t/ np.pi)) ,2*np.cos(t / np.pi)))


def drawCurve(Curve):
    glBegin(GL_LINE_STRIP)
    # glLineWidth(2)
    glColor3f(1.0, 1.0, 1.0)
    for t in range(len(Curve)):
        glVertex3fv(Curve[t])
    glEnd()

def drawTriangle():
    glBegin(GL_TRIANGLE_STRIP)
    points = ((0, 1, 0), (1, 0, 0), (0, -1, 0))
    glColor3f(1.0, 0.0, 0.0)
    [glVertex3fv(i) for i in points]
    glEnd()


def drawGrid():

    glBegin(GL_LINES)
    glColor3f(.5, .5, .5)
    for i in range(-20, 20):
        [glVertex3fv((i/2.0, j/2.0, -5)) for j in range(-20, 20)]
    for i in range(-20, 20):
        [glVertex3fv((j / 2.0, i / 2.0, -5)) for j in range(-20, 20)]
    glEnd()


def main():
    pygame.init()
    display = (800,600)
    pygame.display.set_mode(display, DOUBLEBUF | OPENGL)
    glMatrixMode(GL_MODELVIEW)
    gluPerspective(45, (display[0]/display[1]), 0.1, 250.0)
    glTranslatef(0.0,0.0, -10)

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                quit()
        mouse = pygame.mouse.get_rel()

        if pygame.mouse.get_pressed()[0] & ((mouse[0] != 0) | (mouse[0] != 0)):
            vel = np.power(mouse[0] ** 2 + mouse[1] ** 2, 1 / 2.0)
            glRotatef(int(np.abs(vel)), mouse[0], 0, mouse[1])


        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        glTranslatef(0.0, 0.0, 0)

        drawCurve(parametric)


        # glPushMatrix()
        # # glRotatef(pygame.time.get_ticks()/10.0,1,0,0)
        #
        #
        drawGrid()
        drawTriangle()

        # glPopMatrix()

        pygame.display.flip()
        pygame.time.wait(10)


main()