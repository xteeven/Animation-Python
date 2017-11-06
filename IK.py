import pygame
from OpenGL.GL import *
from OpenGL.GLU import *
from Drawings import *





def setup():
    # Setting Up  the screen resolution and the buffers and then the initial camera position
    pygame.init()
    display = (800, 600)
    pygame.display.set_mode(display, DOUBLEBUF | OPENGL)
    # glEnable(GL_CULL_FACE)
    # glEnable(GL_LIGHTING)
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE)
    glEnable(GL_COLOR_MATERIAL)
    glEnable(GL_LIGHT0)
    glLightfv(GL_LIGHT0, GL_POSITION, [10, 10, 10])
    glMatrixMode(GL_MODELVIEW)
    gluPerspective(45, (display[0]/display[1]), 0.1, 250.0)
    glTranslatef(0.0, 5, -30)
    glRotate(-45, 1, 0, 1)



def main():

    setup()

    link0 = Link((1, 1, 1), (1, 0, 0), direction=[0, 0, 1])
    link1 = Link((.2, .2, 3), (0, 1, 0))
    link2 = Link((.2, .2, 1), (0, 0, 1))
    link3 = Link((.2, .2, 1), (1, 0, 1))
    link4 = Link((.2, .2, 1), (0, 1, 1))
    link5 = Link((.2, .2, 1), (1, 1, 0))
    link6 = Link((.2, .2, 1), (1, 1, 5))

    robot = Arm(link0, link1, link2, link3, link4, link5, link6)

    point = [3, 3, 3]

    robot.update()

    pygame.display.set_caption('IK')

    while True:  # Catch close window event

        target = eventsHandle(returnCoords=True)

        # Clear buffers
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        # Draw a grid
        drawGrid(0)

        robot.follow(point)

        drawCoord(point)

        if target[1]:
            point = target[0]

        pygame.display.flip()

        pygame.time.wait(10)

if __name__ == "__main__":
    main()