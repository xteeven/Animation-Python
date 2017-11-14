#!/usr/bin/env python
import time
from Drawings import *
import sys
import math

from PyQt4 import QtCore, QtGui, QtOpenGL

def tick():
    print 'tick'

try:
    from OpenGL import GL, GLU
except ImportError:
    app = QtGui.QApplication(sys.argv)
    QtGui.QMessageBox.critical(None, "OpenGL Animation",
            "PyOpenGL must be installed to run this example.")
    sys.exit(1)



class Window(QtGui.QWidget):
    def __init__(self):
        super(Window, self).__init__()

        self.glWidget = GLWidget()
        self.textbox = QtGui.QLabel('')
        self.textg = QtGui.QLabel('Gravity')
        self.textd = QtGui.QLabel('Damping')
        self.textm = QtGui.QLabel('Mass')

        self.selectBalls = QtGui.QCheckBox("Balls", self)
        self.selectSprings = QtGui.QCheckBox("Springs", self)
        self.selectInicialLength =QtGui.QCheckBox("Rest Length", self)

        self.selectBalls.setChecked(True)

        self.selectBalls.toggled.connect(self.glWidget.selectBalls)
        self.selectSprings.toggled.connect(self.glWidget.selectSprings)
        self.selectInicialLength.toggled.connect(self.glWidget.selectL0)

        self.spintext = QtGui.QLabel('Matrix dimension')
        self.spinbox = QtGui.QSpinBox()
        self.spinbox.setValue(10)
        self.spinbox.valueChanged.connect(self.glWidget.matrixSize)

        self.gravitySlider = self.createSlider(-500, 100)
        self.gravitySlider.valueChanged.connect(self.glWidget.setGravity)

        self.dampingSlider = self.createSlider(0, 100)
        self.dampingSlider.valueChanged.connect(self.glWidget.setDamping)

        self.massSlider = self.createSlider(1, 100)
        self.massSlider.valueChanged.connect(self.glWidget.setMass)

        mainLayout = QtGui.QHBoxLayout()
        mainLayout.addWidget(self.glWidget)

        baseLayout = QtGui.QVBoxLayout()
        selecLayout = QtGui.QHBoxLayout()
        selecLayout.addWidget(self.textbox)

        glayout = QtGui.QVBoxLayout()
        glayout.addWidget(self.textg)
        glayout.addWidget(self.gravitySlider)

        dlayout = QtGui.QVBoxLayout()
        dlayout.addWidget(self.textd)
        dlayout.addWidget(self.dampingSlider)

        mlayout = QtGui.QVBoxLayout()
        mlayout.addWidget(self.textm)
        mlayout.addWidget(self.massSlider)

        mainLayout.addLayout(glayout)
        mainLayout.addLayout(dlayout)
        mainLayout.addLayout(mlayout)

        selecLayout.addSpacing(100)
        selecLayout.addWidget(self.selectBalls)
        selecLayout.addWidget(self.selectSprings)
        selecLayout.addWidget(self.selectInicialLength)
        selecLayout.addSpacing(100)
        selecLayout.addWidget(self.spintext)
        selecLayout.addWidget(self.spinbox)

        baseLayout.addLayout(selecLayout)


        baseLayout.addLayout(mainLayout)

        self.setLayout(baseLayout)

        self.spintext.setStyleSheet("color: white; font: bold 14px;")
        self.spinbox.setStyleSheet("color: white; font: bold 14px;")

        self.selectBalls.setStyleSheet("color: white; font: bold 14px;")
        self.selectSprings.setStyleSheet("color: white; font: bold 14px;")
        self.selectInicialLength.setStyleSheet("color: white; font: bold 14px;")

        self.setWindowTitle("Deformavel")
        self.setStyleSheet("background-color: black")
        self.gravitySlider.setStyleSheet("background-color: black")
        self.dampingSlider.setStyleSheet("background-color: black")
        self.textg.setStyleSheet("background-color: black; color: white; font: bold 14px;")
        self.textd.setStyleSheet("background-color: black; color: white; font: bold 14px;")
        self.textm.setStyleSheet("background-color: black; color: white; font: bold 14px;")
        self.textbox.setStyleSheet("background-color: black; color: white; font: bold 14px;")


        self.glWidget.distance.connect(self.textbox.setText)


    def createSlider(self, xmin=-10, xmax=10):
        slider = QtGui.QSlider(QtCore.Qt.Vertical)
        slider.setRange(xmin, xmax)
        slider.setSingleStep(0.5)
        slider.setPageStep(1 * 1)
        slider.setTickInterval((xmax-xmin)/100)
        slider.setTickPosition(QtGui.QSlider.TicksRight)

        return slider


class GLWidget(QtOpenGL.QGLWidget):

    times = QtCore.pyqtSignal(str)
    distance = QtCore.pyqtSignal(str)


    def __init__(self, parent=None):
        super(GLWidget, self).__init__(parent)
        self.object = 0
        self.xRot = 0
        self.yRot = 0
        self.zRot = 0
        self.zoom = 0
        self.gravity = np.array([0, 0, -9.8])
        self.damping = 0.98
        self.key = np.array([0, 0, 0]) # axis, axis1value, axis2value
        self.dt = 0.02
        self.bass = Ball(hue=85, pos=np.array([0, 0, 10]), isFixed=True)
        self.bass2 = Ball(hue=85, pos=np.array([-10, 0, 10]), isFixed=True)
        self.bass3 = Ball(hue=85, pos=np.array([0, 10, 10]), isFixed=True)
        self.ball = Ball(hue=15, pos=np.array([0, 1, 9]))
        self.ball2 = Ball(hue=20, pos=np.array([0, 0, 8]))

        self.p3d = [0,0,0, False]
        self.viewSize = []
        self.ballselected = True
        self.springselected = False
        self.L0selected = False
        self.tela = Matrix(5)
        # self.tela.arrange()

        self.cabelo = Hair()
        self.cabelo2 = Hair(x=50, y=2)
        # self.ball.pos = np.array([0, -5, 9])
        # self.bass.pos = np.array([-10, -10, 10])
        # self.bass2.pos = np.array([12, 0, 10])
        self.currentMousePosition = QtCore.QPoint()
        self.lastPos = QtCore.QPoint()
        self.retract_timer = QtCore.QTimer(self)
        self.retract_timer.setInterval(0)
        self.retract_timer.timeout.connect(self.updateGL)
        self.retract_timer.start()

        self.spring = Spring(self.ball, self.bass, k=5)
        self.spring2 = Spring(self.ball2, self.ball, k=5)
        self.spring3 = Spring(self.ball2, self.bass2, k=5)



    def minimumSizeHint(self):
        return QtCore.QSize(400, 400)

    def sizeHint(self):
        return QtCore.QSize(1024, 768)

    def setXRotation(self, angle):
        angle = self.normalizeAngle(angle)
        if angle != self.xRot:
            self.xRot = angle

    def setYRotation(self, angle):
        angle = self.normalizeAngle(angle)
        if angle != self.yRot:
            self.yRot = angle

    def setZRotation(self, angle):
        angle = self.normalizeAngle(angle)
        if angle != self.zRot:
            self.zRot = angle

    def initializeGL(self):
        GL.glShadeModel(GL.GL_FLAT)
        GL.glMatrixMode(GL.GL_MODELVIEW)
        GL.glViewport(0, 0, self.frameGeometry().width(), self.frameGeometry().height())


        GL.glRotate(-45, 1, 0, 1)
        GL.glMatrixMode(GL.GL_PROJECTION)

        GLU.gluPerspective(45, self.frameGeometry().width() / self.frameGeometry().height(), 1, 250.0)
        GL.glTranslatef(0.0, 0.0, -30)
        GL.glRotate(-45, 1, 0, 1)

        # glMaterialfv(GL_FRONT, GL_SPECULAR, [ 1.0, 1.0, 1.0, 0.5])
        # glMaterialfv(GL_FRONT, GL_SHININESS,  50.0)
        # glLightfv(GL_LIGHT0, GL_POSITION, [1.0, 1.0, 1.0, 0.0])
        # glLightfv(GL_LIGHT1, GL_POSITION, [0.0, 1.0, 1.0, 0.0])
        # glLightfv(GL_LIGHT2, GL_POSITION, [1.0, 0.0, 1.0, 0.0])
        #


        # glEnable(GL_LIGHTING)
        # glEnable(GL_LIGHT0)
        # glEnable(GL_LIGHT1)
        # glEnable(GL_LIGHT2)
        GL.glEnable(GL.GL_DEPTH_TEST)
        # GL.glEnable(GL.GL_CULL_FACE)

    def setGravity(self, g):
        self.gravity = [0, 0, g/10.0]

    def setDamping(self, d):
        self.damping = 1-d/100.0

    def setMass(self, m):
        self.tela.changeMass(m/10.0)
        self.cabelo.changeMass(m/10.0)

    def selectBalls(self, option):
        self.ballselected = option

    def selectSprings(self, option):
        self.springselected = option

    def selectL0(self, option):
        self.L0selected = option

    def matrixSize(self, arg):
        self.tela = Matrix(5, arg, arg)
        self.cabelo = Hair(x=arg)
        # self.tela.arrange()

    def moveEvents(self):
        modelView = glGetDoublev(GL_MODELVIEW_MATRIX)

        glTranslatef(modelView[0][2] * 2 * self.zoom,
                     modelView[1][2] * 2 * self.zoom,
                     modelView[2][2] * 2 * self.zoom)

        GL.glRotated(self.xRot / 16.0, 1.0, 0.0, 0.0)
        GL.glRotated(self.yRot / 16.0, 0.0, 1.0, 0.0)
        GL.glRotated(self.zRot / 16.0, 0.0, 0.0, 1.0)

        if self.key[0] == 1:
            glTranslatef(modelView[0][0] * self.key[1] / 2.0,
                         modelView[1][0] * self.key[1] / 2.0,
                         modelView[2][0] * self.key[1] / 2.0)


        if self.key[0] == 2:
            glTranslatef(modelView[0][1] * self.key[2],
                         modelView[1][1] * self.key[2],
                         modelView[2][1] * self.key[2])


        # self.xRot = 0
        # self.yRot = 0
        # self.zRot = 0
        # self.key[0] = 0
        # self.zoom = 0

    # def elasticForce(self,  balls = None, k=5, dt=0.03*2):
    #     # k factor, deltat, balls
    #     if balls:
    #         points = np.array([ball.pos for ball in balls])
    #         distance = sum(points[0]-points[1:]) - np.array([0, 0, 0])
    #         elastic = -k * distance / balls[0].mass
    #         balls[0].vel = balls[0].vel + np.array(self.gravity + elastic) * dt
    #         balls[0].pos = balls[0].pos + np.array(balls[0].vel) * dt + np.array(self.gravity + elastic) * dt ** 2
    #         # print range(1, len(balls))
    #         [drawLine(balls[i].pos - balls[0].pos, balls[0].pos) for i in range(1, len(balls))]
    #         balls[0].update()

    def paintGL(self):

        GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)
        GL.glLoadIdentity()
        self.moveEvents()
        # drawCoord((0, 0, 0))

        # self.ball.update(self.dt, self.gravity,self.tela.mass)
        # self.ball2.update(self.dt, self.gravity,self.tela.mass)
        #
        # self.bass.update(self.dt, self.gravity)
        # self.bass2.update(self.dt, self.gravity)
        # self.bass3.update(self.dt, self.gravity)
        #
        # self.spring.forces()
        # self.spring2.forces()
        # self.spring3.forces()

        # self.tela.update(self.dt, self.gravity, self.damping, self.ballselected, self.springselected, self.L0selected)

        # self.cabelo.update(self.dt, self.gravity, self.damping, self.ballselected, self.springselected, self.L0selected)

        # self.cabelo2.update(self.dt, self.gravity, self.damping, self.ballselected, self.springselected, self.L0selected)

        self.distance.emit('gravity: ' + str(self.gravity) +
                           ' damping: ' + str(self.damping) +
                           ' Mouse: ' +
                           str((self.currentMousePosition.x(), self.currentMousePosition.y()))
                            + ' Mass: ' + str(self.tela.mass)
                           )
        drawGrid(20, 20, 0)
        if self.p3d[-1]:
            drawCoord(self.p3d[:-1])



    def resizeGL(self, width, height):

        # print 'resize'
        self.viewSize = height
        side = max(width, height)
        if side < 0:
            return
        GL.glViewport((width - side) // 2, (height - side) // 2, side, side)
        GL.glMatrixMode(GL.GL_MODELVIEW)
        GL.glLoadIdentity()

    def mousePressEvent(self, event):
        self.lastPos = event.pos()

    def unproject(self, x, y):
        modelView = glGetDoublev(GL_MODELVIEW_MATRIX)
        projection = glGetDoublev(GL_PROJECTION_MATRIX)
        viewport = glGetIntegerv(GL_VIEWPORT)
        #69 - 669
        #376 - 976
        self.p3d[:-1] = gluUnProject(x, y+self.viewSize-600, 0.997, modelView, projection, viewport)
        self.p3d[-1] = True
        print self.p3d



    def mouseMoveEvent(self, event):
        dx = event.x() - self.lastPos.x()
        dy = event.y() - self.lastPos.y()
        if event.buttons() & QtCore.Qt.LeftButton:
            self.currentMousePosition = event.pos()
            self.unproject(event.x(), 600-event.y())
        if event.buttons() & QtCore.Qt.MiddleButton:
            self.setXRotation(self.xRot + 8 * dy)
            self.setYRotation(self.yRot + 8 * dx)
        elif event.buttons() & QtCore.Qt.RightButton:
            self.setXRotation(self.xRot + 8 * dy)
            self.setZRotation(self.zRot + 8 * dx)
        self.lastPos = event.pos()

    def wheelEvent(self, event):
        delta  = event.delta()/np.abs(event.delta())*0.5

        self.zoom = self.zoom + delta \
            if (self.zoom+delta>-10) \
            and (self.zoom+delta<1) \
            else self.zoom
        modelView = glGetDoublev(GL_MODELVIEW_MATRIX)

    def mouseReleaseEvent(self, *args, **kwargs):
        if type(args[0]) is QtGui.QMouseEvent:
            self.currentMousePosition = QtCore.QPoint()
            self.p3d[-1] = False


    def keyPressEvent(self, event):
        print 'e'
        key = event.key()
        if key == 16777234:
            self.key[1] += -0.2
            self.key[0] = 1

        if key == 16777235:
            self.key[2] += 0.2
            self.key[0] = 2

        if key == 16777236:
            self.key[1] += 0.2
            self.key[0] = 1

        if key == 16777237:
            self.key[2] += -0.2
            self.key[0] = 2

    def normalizeAngle(self, angle):
        while angle < 0:
            angle += 360 * 16
        while angle > 360 * 16:
            angle -= 360 * 16
        return angle

    def closeEvent(self, event):
        self.retract_timer.stop()


if __name__ == '__main__':

    app = QtGui.QApplication(sys.argv)
    window = Window()
    window.show()
    sys.exit(app.exec_())
