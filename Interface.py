#!/usr/bin/env python

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
        self.textbox = QtGui.QLabel('Click me')
        self.textbox.setStyleSheet("background-color: black; color: white; font: bold 14px;")


        self.glWidget.distance.connect(self.textbox.setText)


        mainLayout = QtGui.QHBoxLayout()

        mainLayout.addWidget(self.glWidget)


        baseLayout = QtGui.QVBoxLayout()
        baseLayout.addWidget(self.textbox)
        baseLayout.addStretch(1)
        baseLayout.addLayout(mainLayout)

        self.setStyleSheet("background-color: black")


        self.setLayout(baseLayout)

        self.setWindowTitle("Deformavel")

    def createSlider(self):
        slider = QtGui.QSlider(QtCore.Qt.Vertical)

        slider.setRange(-90, 90)
        slider.setSingleStep(1)
        slider.setPageStep(1 * 1)
        slider.setTickInterval(1 * 1)
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
        self.gravity = [0, 0, -9.8]
        self.key = [0, 0, 0]  # axis, axis1value, axis2value
        self.dt = 0
        self.bass = Ball(hue=85)
        self.bass2 = Ball(hue=85)
        self.ball = Ball(hue=15)
        self.ball2 = Ball(hue=20)
        self.tet = Matrix(5)
        self.ball.pos = np.array([0, -5, 9])
        self.bass.pos = np.array([-10, -10, 10])
        self.bass2.pos = np.array([12, 0, 10])
        self.lastPos = QtCore.QPoint()
        self.retract_timer = QtCore.QTimer(self)
        self.retract_timer.setInterval(0)
        self.retract_timer.timeout.connect(self.updateGL)
        self.retract_timer.start()


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
        GL.glEnable(GL.GL_DEPTH_TEST)
        GL.glEnable(GL.GL_CULL_FACE)

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

    def elasticForce(self, k=5, *args):
        points = np.array([ball.pos for ball in args])
        distance = sum(points[0]-points[1:])
        elastic = -k * distance / args[0].mass
        return self.gravity + elastic


    def paintGL(self):

        GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)

        GL.glLoadIdentity()

        dt = 0.03*2
        k = 5

        self.moveEvents()

        drawCoord((0, 0, 0))

        acelerationsb = self.elasticForce(8, self.ball2, self.ball)

        self.ball2.vel = self.ball2.vel + np.array(acelerationsb) * dt
        self.ball2.pos = self.ball2.pos + np.array(self.ball2.vel) * dt + np.array(acelerationsb) * dt ** 2

        acelerations = self.elasticForce(5, self.ball, self.bass, self.bass2)

        self.ball.vel = self.ball.vel + np.array(acelerations)*dt
        self.ball.pos = self.ball.pos + np.array(self.ball.vel)*dt + np.array(acelerations)*dt**2
        self.bass.pos = [0, 0, 10]

        self.bass.update()
        self.bass2.update()
        self.ball.update()
        self.ball2.update()


        # self.tet.update()
        self.distance.emit('Time: ' + str(self.dt) +
                           ' Distance: ' + str(self.ball.pos )
                           )
        drawGrid(0)
        self.dt += dt
        # GL.glPopMatrix()


    def resizeGL(self, width, height):
        # print 'resize'
        side = max(width, height)
        if side < 0:
            return
        GL.glViewport((width - side) // 2, (height - side) // 2, side, side)
        GL.glMatrixMode(GL.GL_MODELVIEW)
        GL.glLoadIdentity()

    def mousePressEvent(self, event):
        self.lastPos = event.pos()

    def mouseMoveEvent(self, event):
        dx = event.x() - self.lastPos.x()
        dy = event.y() - self.lastPos.y()
        if event.buttons() & QtCore.Qt.LeftButton:
            # self.setXRotation(8 * dy)
            # self.setYRotation(8 * dx)
            self.setXRotation(self.xRot + 8 * dy)
            self.setYRotation(self.yRot + 8 * dx)
        elif event.buttons() & QtCore.Qt.RightButton:
            # self.setXRotation(8 * dy)
            # self.setZRotation(8 * dx)
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
