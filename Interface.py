#!/usr/bin/env python
import time
from Drawings import *
import sys
import math

from PyQt4 import QtCore, QtGui, QtOpenGL

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
        self.selectEnlongation = QtGui.QCheckBox("Elasticity", self)
        self.reset = QtGui.QPushButton("Reset")
        self.reset.pressed.connect(self.glWidget.matrixSize)
        self.selectBalls.setChecked(True)
        self.selectSprings.setChecked(True)
        self.selectEnlongation.setChecked(False)
        self.model = QtGui.QComboBox()
        self.model.addItems(["Matrix", "Hair", "Points"])
        self.model.setCurrentIndex(2)
        self.model.currentIndexChanged.connect(self.glWidget.selector)
        self.selectBalls.toggled.connect(self.glWidget.selectBalls)
        self.selectSprings.toggled.connect(self.glWidget.selectSprings)
        self.selectInicialLength.toggled.connect(self.glWidget.selectL0)
        self.selectEnlongation.toggled.connect(self.glWidget.enlongation)
        self.spintext = QtGui.QLabel('Dimension')
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
        self.gravitySlider.setValue(-98)
        dlayout = QtGui.QVBoxLayout()
        dlayout.addWidget(self.textd)
        dlayout.addWidget(self.dampingSlider)
        self.dampingSlider.setValue(98)
        mlayout = QtGui.QVBoxLayout()
        mlayout.addWidget(self.textm)
        mlayout.addWidget(self.massSlider)
        mainLayout.addLayout(glayout)
        mainLayout.addLayout(dlayout)
        mainLayout.addLayout(mlayout)
        selecLayout.addStretch(0)
        selecLayout.addWidget(self.reset)
        selecLayout.addWidget(self.selectBalls)
        selecLayout.addWidget(self.selectSprings)
        selecLayout.addWidget(self.selectInicialLength)
        selecLayout.addWidget(self.selectEnlongation)
        selecLayout.addSpacing(50)
        selecLayout.addWidget(self.spintext)
        selecLayout.addWidget(self.spinbox)
        selecLayout.addWidget(self.model)
        baseLayout.addLayout(selecLayout)
        baseLayout.addLayout(mainLayout)

        self.setLayout(baseLayout)
        self.spintext.setStyleSheet("color: white; font: bold 14px;")
        self.spinbox.setStyleSheet("background-color: #222222; color: white; font: bold 14px;")
        self.reset.setStyleSheet("""
        QPushButton {
         background-color: #222222;
         color: white; 
         font: bold 14px;
        }   
        QPushButton:hover {
            background-color: #333333;
        }
        QPushButton:pressed {
        background-color: #188BC9;     
        }
        """)
        self.model.setStyleSheet("background-color: #222222; color: white; font: bold 14px;")
        self.selectBalls.setStyleSheet("color: white; font: bold 14px;")
        self.selectSprings.setStyleSheet("color: white; font: bold 14px;")
        self.selectInicialLength.setStyleSheet("color: white; font: bold 14px;")
        self.selectEnlongation.setStyleSheet("color: white; font: bold 14px;")
        self.glWidget.setStyleSheet("background-color: white")
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
        self.key = np.array([0, 0, 0])
        self.dt = 0.02
        self.mass = 0.2

        self.elasticity = False
        self.ballselected = True
        self.springselected = True
        self.L0selected = False
        self.tela = Matrix(5)
        self.options = 'Points'
        self.p3d = np.array([0,0,0, False])
        self.viewSize = []
        self.cabelo = Hair()
        self.currentMousePosition = QtCore.QPoint()
        self.lastPos = QtCore.QPoint()
        self.retract_timer = QtCore.QTimer(self)
        self.retract_timer.setInterval(0)
        self.retract_timer.timeout.connect(self.updateGL)
        self.retract_timer.start()
        self.defineDemo()



        # self.spring3 = Spring(self.ball2, self.bass2, k=5)

    def defineDemo(self):
        self.demoBalls = []
        self.demoBalls.append(Ball(hue=85, pos=np.array([0, 0, 10]), isFixed=True))
        self.demoBalls.append(Ball(hue=85, pos=np.array([-10, 0, 10]), isFixed=True))
        self.demoBalls.append(Ball(hue=85, pos=np.array([0, 10, 10]), isFixed=True))
        self.demoBalls.append(Ball(hue=15, pos=np.array([0, 1, 9])))
        self.demoBalls.append(Ball(hue=20, pos=np.array([0, 0, 8])))

        self.demoSprings = []
        self.demoSprings.append(Spring(self.demoBalls[3], self.demoBalls[0],  k=5))
        self.demoSprings.append(Spring(self.demoBalls[3], self.demoBalls[1], k=5))
        self.demoSprings.append(Spring(self.demoBalls[3], self.demoBalls[2], k=5))
        self.demoSprings.append(Spring(self.demoBalls[4], self.demoBalls[3], k=5))

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

    def enlongation(self, option):
        if self.options is 'Matrix':
            self.tela.turnElastisity(option)
        if self.options is 'Hair':
            self.cabelo.turnElastisity(option)
        if self.options is 'Points':
            for spring in self.demoSprings:
                spring.elasticoption = option
        self.elasticity = option

    def setGravity(self, g=False):
        if g is not False:
            self.gravity = np.array([0, 0, g / 10.0])

    def setDamping(self, d):
        self.damping = d/100.0

    def setMass(self, m):
        if self.options is 'Matrix':
            self.tela.changeMass(m/10.0)
        if self.options is 'Hair':
            self.cabelo.changeMass(m/10.0)
        if self.options is 'Points':
            for ball in self.demoBalls:
                ball.mass = m
        self.mass = m

    def selectBalls(self, option):
        self.ballselected = option

    def selectSprings(self, option):
        self.springselected = option

    def selectL0(self, option):
        self.L0selected = option

    def matrixSize(self, arg=10):
        if self.options is 'Matrix':
            self.tela = Matrix(5, arg, arg)
        if self.options is 'Hair':
            self.cabelo = Hair(x=arg)
        if self.options is 'Points':
            self.defineDemo()
        self.setGravity(self.gravity[-1]*10)
        self.setMass(self.mass)
        self.setDamping(self.damping*100)
        self.enlongation(self.elasticity)

    def selector(self, option):
        options = ["Matrix", "Hair", "Points"]
        self.options = options[option]
        self.matrixSize()

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

    def paintGL(self):

        GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)
        GL.glLoadIdentity()
        self.moveEvents()
        drawCoord((0, 0, 0))

        if self.options is 'Matrix':
            self.tela.update(self.dt,
                             self.gravity,
                             self.damping,
                             self.ballselected,
                             self.springselected,
                             self.L0selected,
                             self.p3d)
        if self.options is 'Hair':
            self.cabelo.update(self.dt,
                               self.gravity,
                               self.damping,
                               self.ballselected,
                               self.springselected,
                               self.L0selected,
                               self.p3d)
        if self.options is 'Points':
            for spring in enumerate(self.demoSprings):
                spring[1].forces(self.springselected, self.L0selected)

            for ball in self.demoBalls:
                ball.update(self.dt, self.gravity,  self.damping, self.ballselected, self.p3d)

        self.distance.emit('gravity: ' + str(self.gravity) +
                           ' damping: ' + str(self.damping)
                           # ' Mouse: ' +
                           # str((self.currentMousePosition.x(), self.currentMousePosition.y()))
                           #  + ' P3D: ' + str(np.round(self.p3d[:-1], 1))
        #                     + ' Mass: ' + str(self.tela.mass)
                           )
        drawGrid(20, 20, 0, solid=False)

    def resizeGL(self, width, height):
        side = max(width, height)
        self.viewSize = [height]
        if side < 0:
            return
        GL.glViewport((width - side) // 2, (height - side) // 2, side, side)
        GL.glMatrixMode(GL.GL_MODELVIEW)
        GL.glLoadIdentity()

    def mousePressEvent(self, event):
        if event.buttons() & QtCore.Qt.LeftButton:
            self.currentMousePosition = event.pos()
            self.unproject(event.x(), 600-event.y())
        self.lastPos = event.pos()

    def unProject(self, x, y):
        modelView = glGetDoublev(GL_MODELVIEW_MATRIX)
        projection = glGetDoublev(GL_PROJECTION_MATRIX)
        viewport = glGetIntegerv(GL_VIEWPORT)
        #69 - 669
        #376 - 976
        # print self.viewSize, y
        z = glReadPixels(x, y+int(self.viewSize[0]-600), 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT)
        self.p3d = np.hstack([gluUnProject(x, y+self.viewSize[0]-600, z, modelView, projection, viewport),1])

        # self.p3d[2] = glReadPixels(x, y, 1, 1, GL_DEPTH_COMPONENT, GL_FLOAT)

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
        delta = event.delta()/np.abs(event.delta())*0.5
        self.zoom = self.zoom + delta \
            if (self.zoom+delta > -15) \
            and (self.zoom+delta < 5) \
            else self.zoom

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
