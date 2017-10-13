import pygame
from OpenGL.GL import *
from OpenGL.GLU import *
from Drawings import *
from math import sin, cos, pi, radians

class Link:

    def __init__(self, size=(0, 0, 1), color=(0, 0, 0)):
        self.size = size
        self.color = color
        self.child = None
        self.arm = None
        self.angle = 0
        self.rotation = 0
        self.currentpos = [0, 0, 0]

    def appendArm(self, arm):
        self.child = True
        self.arm = arm

    def update(self):
        glPushMatrix()

        glRotatef(self.angle, True, 0, 0)
        glRotatef(self.rotation, 0, 0, True)
        self.draw(self.size, self.color)
        self.currentpos[2] = 0
        if self.child:
            glPushMatrix()
            glTranslatef(0, 0, self.size[2])
            self.arm.update()
            glPopMatrix()
        glPopMatrix()
        self.currentpos[2] += self.size[2] * cosd(self.angle)

    def draw(self, size, color):

        glPushMatrix()
        glMultMatrixf(np.hstack((size[0], 0, 0, 0,
                                 0, size[1], 0, 0,
                                 0, 0, size[2], 0,
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
            (0, 1),
            (0, 3),
            (0, 4),
            (2, 1),
            (2, 3),
            (2, 7),
            (6, 3),
            (6, 4),
            (6, 7),
            (5, 1),
            (5, 4),
            (5, 7)
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

class Arm:

    def __init__(self, *args):
        self.links = args
        self.length = [link.size[2] for link in self.links]
        self.angles = [link.angle for link in self.links]
        self.currentPos = [0, 0, 0]

    def update(self):
        self.angles = [link.angle for link in self.links]
        xy = -sum([self.length[link] * sind(sum(self.angles[:link + 1])) for link in range(len(self.links))])
        self.currentPos[0] = xy * sind(-self.links[0].rotation)
        self.currentPos[1] = xy * cosd(-self.links[0].rotation)
        self.currentPos[2] = \
            sum([self.length[link] * cosd(sum(self.angles[:link + 1])) for link in range(len(self.links))])
        self.links[0].update()


def cosd(x): return cos(radians(x))


def sind(x): return sin(radians(x))


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
    base = Link((1, 1, 1), (1, 0, 0))
    link1 = Link((.5, .5, 3), (0, 1, 0))
    link2 = Link((.5, .5, 2), (0, 0, 1))
    link3 = Link((.5, .5, 1), (1, 0, 1))

    base.appendArm(link1)
    link1.appendArm(link2)
    link2.appendArm(link3)

    robot = Arm(base, link1, link2, link3)

    i = 0
    while True:  # Catch close window event

        eventsHandle()
        # Clear buffers
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        # Draw a grid
        drawGrid(0)

        robot.links[0].rotation = i
        robot.links[1].angle = 90*cos(i/50.0-1)
        robot.links[2].angle = 90*cos(i/50.0+1)
        robot.links[3].angle = 90*cos(i/50.0)
        robot.update()

        drawCoord(robot.currentPos)
        i += 1

        pygame.display.flip()

        pygame.time.wait(10)


if __name__ == "__main__":
    main()