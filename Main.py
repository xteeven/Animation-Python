import pygame
from OpenGL.GL import *
from OpenGL.GLU import *
from Drawings import *

# Drawing a Curve
parametric = []
for t in np.linspace(-10, 10, 500):
    parametric.append((5*np.cos(t), t, 2*np.sin(t/2)))

parametric = interpcurve(200, parametric)

def setup():
    pygame.init()
    display = (800,600)
    pygame.display.set_mode(display, DOUBLEBUF | OPENGL)
    glMatrixMode(GL_MODELVIEW)
    gluPerspective(45, (display[0]/display[1]), 0.1, 250.0)
    glTranslatef(0.0, 5, -30)
    glRotate(-45, 1, 0, 1)

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

        drawCurve(parametric)

        glPushMatrix()

        rotateInCurve(parametric, iterar)

        drawTriangle()

        glPopMatrix()

        drawGrid()

        pygame.display.flip()
        iterar = iterar + 1 if iterar < len(parametric) - 2 else 0
        pygame.time.wait(10)


main()