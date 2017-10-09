import pygame
from OpenGL.GL import *
from OpenGL.GLU import *
from Drawings import *






def setup():
    # Setting Up  the screen resolution and the buffers and then the initial camera position
    pygame.init()
    display = (800, 600)
    pygame.display.set_mode(display, DOUBLEBUF | OPENGL)
    glMatrixMode(GL_MODELVIEW)
    gluPerspective(45, (display[0]/display[1]), 0.1, 250.0)
    glTranslatef(0.0, 5, -30)
    glRotate(-45, 1, 0, 1)



def main():

    setup()

    while True: # Catch close window event

        eventsHandle()
        # Clear buffers
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        # Draw a grid
        drawGrid(0)
        glPushMatrix()

        drawLink(.5, .5, 5, (1, 0, 0))  # First Joint


        glPushMatrix()
        glTranslatef(0, 0, 5)
        glRotatef(10, 1, 0, 0)
        drawLink(.5, .5, 4, (0, 1, 0))  # Second Joint


        glPushMatrix()
        glTranslatef(0, 0, 4)
        glRotatef(70, 1, 0, 0)
        drawLink(.5, .5, 2, (0, 0, 1))  # Third Joint



        glPopMatrix()
        glPopMatrix()
        glPopMatrix()
        pygame.display.flip()

        pygame.time.wait(10)


if __name__ == "__main__":
    main()