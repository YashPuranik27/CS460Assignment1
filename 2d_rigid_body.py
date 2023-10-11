import matplotlib

matplotlib.use('TkAgg')
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib import animation
import collision_checking
import random


class RigidBody:
    def __init__(self):
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlim([0, 2])
        self.ax.set_ylim([0, 2])
        self.car = patches.Rectangle((0.5, 0.5), 0.1, 0.05, angle=0, edgecolor='black', facecolor='white')
        self.ax.add_patch(self.car)
        self.obstacles = collision_checking.load_polygons_from_file('name_initials.npy')
        for ob in self.obstacles:
            self.ax.fill(ob[:, 0], ob[:, 1], 'b', alpha=0.5)

    def x(self):
        return self.car.get_x()

    def y(self):
        return self.car.get_y()

    def degrees(self):
        return self.car.angle

    def on_key_press(self, event):
        step = 0.05
        new_x, new_y, new_angle = self.x(), self.y(), self.degrees()

        if event.key == 'up':
            new_y += step
        elif event.key == 'down':
            new_y -= step
        elif event.key == 'right':
            new_x += step
        elif event.key == 'left':
            new_x -= step
        elif event.key == 'd':
            new_angle += 45
        elif event.key == 'a':
            new_angle -= 45

        # Define a transformation matrix for rotation
        rotation_matrix = np.array([
            [np.cos(np.radians(new_angle)), -np.sin(np.radians(new_angle))],
            [np.sin(np.radians(new_angle)), np.cos(np.radians(new_angle))]
        ])

        # Define the rectangle (without translation)
        new_rect = np.array([
            [0, 0],
            [0.1, 0],
            [0.1, 0.05],
            [0, 0.05]
        ])

        # Apply the rotation and translation to get the final coordinates
        new_rect = np.dot(new_rect, rotation_matrix.T) + [new_x, new_y]

        collision = False
        for ob in self.obstacles:
            if collision_checking.collides_SAT(new_rect, ob):
                collision = True
                break

        # Always update position and angle, regardless of potential collision
        self.car.set_x(new_x)
        self.car.set_y(new_y)
        self.car.angle = new_angle

        # Change color only based on collision status
        if collision:
            self.car.set_facecolor("red")
        else:
            self.car.set_facecolor("white")

        self.fig.canvas.draw()

    def run(self):
        self.fig.canvas.mpl_connect('key_press_event', self.on_key_press)
        plt.show()


if __name__ == "__main__":
    rb = RigidBody()
    rb.run()
