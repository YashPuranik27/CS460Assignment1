import matplotlib

matplotlib.use('TkAgg')
import numpy as np
import matplotlib.pyplot as plt
import collision_checking
from matplotlib.patches import Circle, Rectangle
import math

# Constants
L1 = 0.4
L2 = 0.25
W = 0.1
R = 0.05

theta1 = 0
theta2 = 0


def check_collision_with_polygons(shape, polygons):
    for polygon in polygons:
        if collision_checking.collides(shape, polygon):
            return True
    return False


def check_all_collisions(polygons, joint1, joint2, joint3, rect1, rect2):
    collided = [False, False, False, False, False]

    # Check each part of the arm for collision
    collided[0] = check_collision_with_polygons(rect1, polygons)
    collided[1] = check_collision_with_polygons(rect2, polygons)
    collided[2] = check_collision_with_polygons(joint1, polygons)
    collided[3] = check_collision_with_polygons(joint2, polygons)
    collided[4] = check_collision_with_polygons(joint3, polygons)

    return collided


def create_arm(joint1, theta1, theta2):
    joint2 = [joint1[0] + (L1 + R) * math.cos(math.radians(theta1)),
              joint1[1] + (L1 + R) * math.sin(math.radians(theta1))]
    joint3 = [joint2[0] + (L2 + R) * math.cos(math.radians(theta1 + theta2)),
              joint2[1] + (L2 + R) * math.sin(math.radians(theta1 + theta2))]
    return joint2, joint3


def plot_arm(ax, polygons, joint1, joint2, joint3, collided):
    ax.clear()  # Clear the current axes
    ax.set_aspect('equal', 'box')
    ax.set_xlim([0, 2])
    ax.set_ylim([0, 2])

    # polygons. This may be deleted after we import the .npy
    for polygon in polygons:
        ax.fill(polygon[:, 0], polygon[:, 1], 'y', alpha=0.5)

    # drawing the links (arm segments)
    ax.add_patch(Rectangle((joint1[0], joint1[1] - W / 2), L1, W, angle=theta1, color='r' if collided[0] else 'b'))
    ax.add_patch(
        Rectangle((joint2[0], joint2[1] - W / 2), L2, W, angle=theta1 + theta2, color='r' if collided[1] else 'b'))

    # drawing the joints
    ax.add_patch(Circle(joint1, R, color='r' if collided[2] else 'b'))
    ax.add_patch(Circle(joint2, R, color='r' if collided[3] else 'b'))
    ax.add_patch(Circle(joint3, R, color='r' if collided[4] else 'b'))


def on_key(event):
    global theta1, theta2
    if event.key == 'right':
        theta1 += 5
    elif event.key == 'left':
        theta1 -= 5
    elif event.key == 'up':
        theta2 += 5
    elif event.key == 'down':
        theta2 -= 5

    joint2, joint3 = create_arm(joint1, theta1, theta2)
    plot_arm(ax, polygons, joint1, joint2, joint3, collided)
    plt.draw()


def main():
    global joint1, polygons, collided, ax
    polygons = []  # Placeholder, replace with the .npy file

    joint1 = [1, 1]
    joint2, joint3 = create_arm(joint1, theta1, theta2)

    collided = [False, False, False, False, False]

    fig, ax = plt.subplots()
    fig.canvas.mpl_connect('key_press_event', on_key)

    plot_arm(ax, polygons, joint1, joint2, joint3, collided)
    plt.show()


if __name__ == "__main__":
    main()