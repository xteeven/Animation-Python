import pygame
from OpenGL.GL import *
import numpy as np
from matplotlib.pyplot import plot, show


def MTL(filename):
    contents = {}
    mtl = None
    for line in open(filename, "r"):
        if line.startswith('#'): continue
        values = line.split()
        if not values: continue
        if values[0] == 'newmtl':
            mtl = contents[values[1]] = {}
        elif mtl is None:
            raise ValueError, "mtl file doesn't start with newmtl stmt"
        elif values[0] == 'map_Kd':
            # load the texture referred to by this declaration
            mtl[values[0]] = values[1]
            surf = pygame.image.load(mtl['map_Kd'])
            image = pygame.image.tostring(surf, 'RGBA', 1)
            ix, iy = surf.get_rect().size
            texid = mtl['texture_Kd'] = glGenTextures(1)
            glBindTexture(GL_TEXTURE_2D, texid)
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER,
                GL_LINEAR)
            glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER,
                GL_LINEAR)
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, ix, iy, 0, GL_RGBA,
                GL_UNSIGNED_BYTE, image)
        else:
            mtl[values[0]] = map(float, values[1:])
    return contents

class OBJ:
    def __init__(self, filename, swapyz=False):
        """Loads a Wavefront OBJ file. """
        self.vertices = []
        self.normals = []
        self.texcoords = []
        self.faces = []

        material = None
        for line in open(filename, "r"):
            if line.startswith('#'): continue
            values = line.split()
            if not values: continue
            if values[0] == 'v':
                v = map(float, values[1:4])
                if swapyz:
                    v = v[0], v[2], v[1]
                self.vertices.append(v)
            elif values[0] == 'vn':
                v = map(float, values[1:4])
                if swapyz:
                    v = v[0], v[2], v[1]
                self.normals.append(v)
            elif values[0] == 'vt':
                self.texcoords.append(map(float, values[1:3]))
            elif values[0] in ('usemtl', 'usemat'):
                material = values[1]
            elif values[0] == 'mtllib':
                self.mtl = MTL(values[1])
            elif values[0] == 'f':
                face = []
                texcoords = []
                norms = []
                for v in values[1:]:
                    w = v.split('/')
                    face.append(int(w[0]))
                    if len(w) >= 2 and len(w[1]) > 0:
                        texcoords.append(int(w[1]))
                    else:
                        texcoords.append(0)
                    if len(w) >= 3 and len(w[2]) > 0:
                        norms.append(int(w[2]))
                    else:
                        norms.append(0)
                self.faces.append((face, norms, texcoords, material))

        self.gl_list = glGenLists(1)
        glNewList(self.gl_list, GL_COMPILE)
        glEnable(GL_TEXTURE_2D)
        glFrontFace(GL_CCW)
        for face in self.faces:
            vertices, normals, texture_coords, material = face

            mtl = self.mtl[material]
            if 'texture_Kd' in mtl:
                # use diffuse texmap
                glBindTexture(GL_TEXTURE_2D, mtl['texture_Kd'])
            else:
                # just use diffuse colour
                glColor(*mtl['Kd'])

            glBegin(GL_POLYGON)
            for i in range(len(vertices)):
                if normals[i] > 0:
                    glNormal3fv(self.normals[normals[i] - 1])
                if texture_coords[i] > 0:
                    glTexCoord2fv(self.texcoords[texture_coords[i] - 1])
                glVertex3fv(self.vertices[vertices[i] - 1])
            glEnd()
        glDisable(GL_TEXTURE_2D)
        glEndList()




# https://github.com/petercollingridge/Pygame3D

def translationMatrix(dx=0, dy=0, dz=0):
    """ Return matrix for translation along vector (dx, dy, dz). """

    return np.array([[1, 0, 0, 0],
                     [0, 1, 0, 0],
                     [0, 0, 1, 0],
                     [dx, dy, dz, 1]])


def translateAlongVectorMatrix(vector, distance):
    """ Return matrix for translation along a vector for a given distance. """

    unit_vector = np.hstack([unitVector(vector) * distance, 1])
    return np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], unit_vector])


def scaleMatrix(s, cx=0, cy=0, cz=0):
    """ Return matrix for scaling equally along all axes centred on the point (cx,cy,cz). """

    return np.array([[s, 0, 0, 0],
                     [0, s, 0, 0],
                     [0, 0, s, 0],
                     [cx * (1 - s), cy * (1 - s), cz * (1 - s), 1]])


def rotateXMatrix(radians):
    """ Return matrix for rotating about the x-axis by 'radians' radians """

    c = np.cos(radians)
    s = np.sin(radians)
    return np.array([[1, 0, 0, 0],
                     [0, c, -s, 0],
                     [0, s, c, 0],
                     [0, 0, 0, 1]])


def rotateYMatrix(radians):
    """ Return matrix for rotating about the y-axis by 'radians' radians """

    c = np.cos(radians)
    s = np.sin(radians)
    return np.array([[c, 0, s, 0],
                     [0, 1, 0, 0],
                     [-s, 0, c, 0],
                     [0, 0, 0, 1]])


def rotateZMatrix(radians):
    """ Return matrix for rotating about the z-axis by 'radians' radians """

    c = np.cos(radians)
    s = np.sin(radians)
    return np.array([[c, -s, 0, 0],
                     [s, c, 0, 0],
                     [0, 0, 1, 0],
                     [0, 0, 0, 1]])


def rotateAboutVector((cx, cy, cz), (x, y, z), radians):
    """ Rotate wireframe about given vector by 'radians' radians. """

    # Find angle and matrix needed to rotate vector about the z-axis such that its y-component is 0
    rotZ = np.arctan2(y, x)
    rotZ_matrix = rotateZMatrix(rotZ)

    # Find angle and matrix needed to rotate vector about the y-axis such that its x-component is 0
    (x, y, z, _) = np.dot(np.array([x, y, z, 1]), rotZ_matrix)
    rotY = np.arctan2(x, z)

    matrix = translationMatrix(dx=-cx, dy=-cy, dz=-cz)
    matrix = np.dot(matrix, rotZ_matrix)
    matrix = np.dot(matrix, rotateYMatrix(rotY))
    matrix = np.dot(matrix, rotateZMatrix(radians))
    matrix = np.dot(matrix, rotateYMatrix(-rotY))
    matrix = np.dot(matrix, rotateZMatrix(-rotZ))
    matrix = np.dot(matrix, translationMatrix(dx=cx, dy=cy, dz=cz))

    return matrix


class Wireframe:
    """ An array of vectors in R3 and list of edges connecting them. """

    def __init__(self, nodes=None):
        self.nodes = np.zeros((0, 4))
        self.edges = []
        self.faces = []

        if nodes:
            self.addNodes(nodes)

    def addNodes(self, node_array):
        """ Append 1s to a list of 3-tuples and add to self.nodes. """

        ones_added = np.hstack((node_array, np.ones((len(node_array), 1))))
        self.nodes = np.vstack((self.nodes, ones_added))

    def addEdges(self, edge_list):
        """ Add edges as a list of 2-tuples. """

        # Is it better to use a for loop or generate a long list then add it?
        # Should raise exception if edge value > len(self.nodes)
        self.edges += [edge for edge in edge_list if edge not in self.edges]

    def addFaces(self, face_list, face_colour=(255, 255, 255)):
        for node_list in face_list:
            num_nodes = len(node_list)
            if all((node < len(self.nodes) for node in node_list)):
                # self.faces.append([self.nodes[node] for node in node_list])
                self.faces.append((node_list, np.array(face_colour, np.uint8)))
                self.addEdges([(node_list[n - 1], node_list[n]) for n in range(num_nodes)])

    def output(self):
        if len(self.nodes) > 1:
            self.outputNodes()
        if self.edges:
            self.outputEdges()
        if self.faces:
            self.outputFaces()

    def outputNodes(self):
        print "\n --- Nodes --- "
        for i, (x, y, z, _) in enumerate(self.nodes):
            print "   %d: (%d, %d, %d)" % (i, x, y, z)

    def outputEdges(self):
        print "\n --- Edges --- "
        for i, (node1, node2) in enumerate(self.edges):
            print "   %d: %d -> %d" % (i, node1, node2)

    def outputFaces(self):
        print "\n --- Faces --- "
        for i, nodes in enumerate(self.faces):
            print "   %d: (%s)" % (i, ", ".join(['%d' % n for n in nodes]))

    def transform(self, transformation_matrix):
        """ Apply a transformation defined by a transformation matrix. """

        self.nodes = np.dot(self.nodes, transformation_matrix)

    def findCentre(self):
        """ Find the spatial centre by finding the range of the x, y and z coordinates. """

        min_values = self.nodes[:, :-1].min(axis=0)
        max_values = self.nodes[:, :-1].max(axis=0)
        return 0.5 * (min_values + max_values)

    def sortedFaces(self):
        return sorted(self.faces, key=lambda face: min(self.nodes[f][2] for f in face[0]))

    def update(self):
        """ Override this function to control wireframe behaviour. """
        pass


class WireframeGroup:
    """ A dictionary of wireframes and methods to manipulate them all together. """

    def __init__(self):
        self.wireframes = {}

    def addWireframe(self, name, wireframe):
        self.wireframes[name] = wireframe

    def output(self):
        for name, wireframe in self.wireframes.items():
            print name
            wireframe.output()

    def outputNodes(self):
        for name, wireframe in self.wireframes.items():
            print name
            wireframe.outputNodes()

    def outputEdges(self):
        for name, wireframe in self.wireframes.items():
            print name
            wireframe.outputEdges()

    def findCentre(self):
        """ Find the central point of all the wireframes. """

        # There may be a more efficient way to find the minimums for a group of wireframes
        min_values = np.array([wireframe.nodes[:, :-1].min(axis=0) for wireframe in self.wireframes.values()]).min(
            axis=0)
        max_values = np.array([wireframe.nodes[:, :-1].max(axis=0) for wireframe in self.wireframes.values()]).max(
            axis=0)
        return 0.5 * (min_values + max_values)

    def transform(self, matrix):
        for wireframe in self.wireframes.values():
            wireframe.transform(matrix)

    def update(self):
        for wireframe in self.wireframes.values():
            wireframe.update()



def predic(parametric, i):
    d = .5
    dist = 0
    for j in range(100):
        mod = np.linalg.norm(np.subtract(parametric(i+np.abs(dist)), parametric(i)))
        dist = dist+0.0001 if abs(mod-d)+mod-d else dist -0.0001

        print 'modulo', mod, 'objetivo', d, 'error', (mod - d), 'distancia', dist
    return dist


def param(t):
    return (1*np.cos(t), t**2*np.sin(t), t**2)



def interpcurve(N, curve):
    # equally spaced in arclength

    N = np.transpose(np.linspace(0, 1, N))

    # how many points will be uniformly interpolated?
    nt = N.size

    # number of points on the curve
    n = len(curve)
    pxy = np.array(curve)
    p1 = pxy[0,:]
    pend = pxy[-1,:]
    last_segment = np.linalg.norm(np.subtract(p1, pend))
    epsilon = 10*np.finfo(float).eps

    # IF the two end points are not close enough lets close the curve
    if last_segment > epsilon*np.linalg.norm(np.amax(abs(pxy), axis=0)):
        pxy = np.vstack((pxy, p1))
        nt = nt + 1
    else:
        print('Contour already closed')

    pt=np.zeros((nt, 2))

    # Compute the chordal arclength of each segment.
    chordlen = (np.sum(np.diff(pxy,axis=0)**2,axis=1))**(1/2)
    # Normalize the arclengths to a unit total
    chordlen = chordlen/np.sum(chordlen)
    # cumulative arclength
    cumarc = np.append(0, np.cumsum(chordlen))

    tbins= np.digitize(N, cumarc)  # bin index in which each N is in

    #catch any problems at the ends
    tbins[np.where(tbins<=0 | (N<=0))]=1
    tbins[np.where(tbins >= n | (N >= 1))] = n - 1

    s = np.divide((N - cumarc[tbins]), chordlen[tbins-1])
    pt = pxy[tbins, :] + np.multiply((pxy[tbins, :] - pxy[tbins-1, :]), (np.vstack([s]*3)).T)

    return pt

parametric = []
for t in np.linspace(-2.5, 0.0, 90):
    parametric.append((1*np.cos(t), t**2*np.sin(t), t**2))

t = np.linspace(0,10,len(parametric))
x = t**2
y = t/2
z = t

it = interpcurve(10, parametric)

# print [tuple(i) for i in it]

# plot(it[:,0],it[:,1],'b^',x,y, 'g^')
# show()