import pygame
from pygame.locals import *
import numpy as np
from OpenGL.GL import *
from OpenGL.GLU import *
from Auxs import OBJ

parametric = []
for t in range(1000):
    parametric.append((10*np.tan(t/np.pi)+np.sin(t / np.pi), (np.log(t+1/ np.pi)),2*np.cos(t / np.pi)))


def drawCurve(Curve):
    glBegin(GL_LINE_STRIP)
    for t in range(len(Curve)):
        glVertex3fv(Curve[t])
    glEnd()




def main():
    pygame.init()
    display = (800,600)
    pygame.display.set_mode(display, DOUBLEBUF | OPENGL)

    gluPerspective(45, (display[0]/display[1]), 0.1, 500.0)

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
        # Cube()
        drawCurve(parametric)
        pygame.display.flip()
        pygame.time.wait(10)


main()