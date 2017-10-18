# Animation-Python

This repository contains a little script-collection to perform basic animations in python. 
by now it has some implementations to draw a 3d grid, an paper airplane (used for the first exaple in PATH.py), coordinate systems, 3d Lines.
Moreover it cointains methods to calculate inverse kinematics, to build a 3d robot with joints with 3 degrees.

## Getting Started

All the methods are in the "Auxs.py" and "Drawings.py" Scripts, so you only need to import them

### Prerequisites & Installing

You need to install Opengl (pyOpengl) and numpy and pygame
you can get all of them from pip.
try with 
```
pip install pyopengl
pip install numpy
pip install pygame
```
in your command line.

when finished you just need to clone this repository

## Example usage

```
import pygame
from OpenGL.GL import *
from OpenGL.GLU import *
from Drawings import *

pygame.init()
display = (800, 600)
pygame.display.set_mode(display, DOUBLEBUF | OPENGL)
glMatrixMode(GL_MODELVIEW)
gluPerspective(45, (display[0]/display[1]), 0.1, 250.0)
glTranslatef(0.0, 5, -30)
glRotate(-45, 1, 0, 1)

link0 = Link((1, 1, 1), (1, 0, 0), direction= [0, 0, 1])
link1 = Link((.2, .2, 3), (0, 1, 0))
link2 = Link((.2, .2, 2), (0, 0, 1))
link3 = Link((.2, .2, 1), (1, 0, 1))
link4 = Link((.2, .2, 3), (0, 1, 1))
robot = Arm(link0, link1, link2, link3, link4)
point = np.random.random_sample(3)
while True:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            quit()
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
    # Draw a grid
    drawGrid(0)
    robot.follow(point)
    drawCoord(point)
    if target[1]:
       point = target[0]
    pygame.display.flip()
    pygame.time.wait(10)
    
```


## Authors

* **Steeven Villa** - *dsvsalazar@inf.ufrgs.br* - [Webpage](http://inf.ufrgs.br/~dsvsalazar/)

## License

Feel free to use this code in all your implementations, if you have some questions just contact me.

