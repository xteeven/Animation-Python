import numpy as np
from OpenGL.GL import *
from OpenGL.GLU import *
import pygame
from pygame.locals import *
import numpy as np
from math import sin, cos, radians, degrees, atan


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

def interpIndex(curve):
    curve = np.array(curve)
    segments = np.diff(curve, 1, axis=0)
    norm = np.linalg.norm(segments, axis=1)
    total = np.cumsum(norm)
    totaluni = total / total[-1]
    num = np.linspace(0, 1, len(total))
    inter = np.digitize(num, totaluni)
    return inter


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

def mouseMove(event):
    # handles the mouse events to move the camera
    mouse = pygame.mouse.get_rel()
    mousepos = pygame.mouse.get_pos()
    mousepressed = pygame.mouse.get_pressed()
    modelView = glGetDoublev(GL_MODELVIEW_MATRIX)
    projection = glGetDoublev(GL_PROJECTION_MATRIX)
    viewport = glGetIntegerv(GL_VIEWPORT)
    vel = np.power(mouse[0] ** 2 + mouse[1] ** 2, 1 / 2.0)
    keyboard = pygame.key.get_pressed()

    p3d = gluUnProject(mousepos[0], 600-mousepos[1], 0.997, modelView, projection, viewport)
    # print p3d

    if mousepressed[2] & keyboard[K_LSHIFT] & ((mouse[0] != 0)):
        glRotatef(int((vel)), 0, 0, mouse[0])

    elif mousepressed[2] & ((mouse[0] != 0) | (mouse[1] != 0)):
        glRotatef(int(np.abs(vel)), mouse[1], mouse[0], 0)

    if mousepressed[1]:

        glTranslatef(modelView[0][0] * mouse[0] / 100.0,
                     modelView[1][0] * mouse[0] / 100.0,
                     modelView[2][0] * mouse[0] / 100.0)

        glTranslatef(modelView[0][1] * (-1) * mouse[1] / 100.0,
                     modelView[1][1] * (-1) * mouse[1] / 100.0,
                     modelView[2][1] * (-1) * mouse[1] / 100.0)

    if event.type == pygame.MOUSEBUTTONDOWN:

        if event.button == 4:
            glTranslatef(modelView[0][2] * (-1) * 2,
                         modelView[1][2] * (-1) * 2,
                         modelView[2][2] * (-1) * 2)

        elif event.button == 5:
            glTranslatef(modelView[0][2]*2,
                         modelView[1][2]*2,
                         modelView[2][2]*2)
    return p3d, mousepressed[0]


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
        glTranslatef(modelView[0][1]*(-1),
                     modelView[1][1]*(-1),
                     modelView[2][1]*(-1))

    if keyboard[K_DOWN]:
        glTranslatef(modelView[0][1],
                     modelView[1][1],
                     modelView[2][1])


def eventsHandle(mouse=mouseMove, keyboard=ketboardMove, returnCoords = False):
    coords = 0
    key = 0
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            quit()
        elif returnCoords is True:
            coords, key = mouse(event)
        else:
            mouse(event)
    keyboard()
    if returnCoords is True:
        return coords, key

class Link:

    def __init__(self, size=(0, 0, 1), color=(0, 0, 0), direction = [1, 0, 0]):
        self.size = size
        self.color = color
        self.child = None
        self.arm = None
        self.angle = 0
        self.rotation = 0
        self.currentpos = [0, 0, 0]
        self.direction = direction

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

        for link in range(len(args)-1):
            args[link].appendArm(args[link+1])

        self.length = [link.size[2] for link in self.links]
        self.angles = [link.angle for link in self.links]
        self.currentPos = np.zeros([len(self.links), 3])
        self.xy = 0

    def setRotation(self, args):
        for angle in enumerate(args):
            self.links[angle[0]].rotation -= angle[1]

    def setAngle(self, args):
        for angle in enumerate(args):
            self.links[angle[0]].angle -= angle[1]

    def follow(self, point):

        endEffector = self.currentPos[0]
        joints = np.copy(self.currentPos[::-1])
        joints[-1] = [0, 0, 0]
        J = [np.cross(self.links[joint].direction, endEffector-joints[joint-1]) for joint in range(len(self.links))]
        newAngles = 2 * np.matrix(J) * np.matrix(self.currentPos[0] - point).transpose()
        rotate = []
        for rotation in enumerate(newAngles):
            rotate.append(float(rotation[1]) * (self.links[rotation[0]].direction[2]))
        self.setRotation(rotate)
        spin = []
        for angle in enumerate(newAngles):
            spin.append(float(angle[1])*(self.links[angle[0]].direction[0] | self.links[angle[0]].direction[1]))
        self.setAngle(spin)
        self.update()

    def update(self):
        self.angles = [link.angle for link in self.links]
        for j in range(len(self.links)):
            self.xy = -sum([self.length[link] * sind(sum(self.angles[:link + 1])) for link in range(len(self.links)-j)])
            self.currentPos[j][0] = self.xy * sind(-self.links[0].rotation)
            self.currentPos[j][1] = self.xy * cosd(-self.links[0].rotation)
            self.currentPos[j][2] = \
                sum([self.length[link] * cosd(sum(self.angles[:link + 1])) for link in range(len(self.links)-j)])

        self.links[0].update()

def cosd(x): return cos(radians(x))

def sind(x): return sin(radians(x))


if __name__ == "__main__":
    pass