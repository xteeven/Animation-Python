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
    # Setting Up  the screen resolution and the buffers and then the initial camera position
    pygame.init()
    display = (800,600)
    pygame.display.set_mode(display, DOUBLEBUF | OPENGL)
    glMatrixMode(GL_MODELVIEW)
    gluPerspective(45, (display[0]/display[1]), 0.1, 250.0)
    glTranslatef(0.0, 5, -30)
    glRotate(-45, 1, 0, 1)

def main():
    setup()
    frame = 0
    while True: # Catch close window event
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                quit()

        # Handle mouse events
        mouseMove()
        # Handle keyboard events
        ketboardMove()
        # Clear buffers
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT)
        # Drawing the path from a previously generated curve
        drawCurve(parametric)
        # Draw a grid
        drawGrid()
        # Performing the operations along the patch
        glPushMatrix()
        # Moving and rotating the ModelView in curve direction.
        moveInCurve(parametric, frame)
        # Draws a simple paper plane
        drawPlane()
        glPopMatrix()

        # Clear and refresh the screen
        pygame.display.flip()
        frame = frame + 1 if frame < len(parametric) - 2 else 0
        pygame.time.wait(10)


if __name__ == "__main__":
    main()