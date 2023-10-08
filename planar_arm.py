import matplotlib

matplotlib.use('TkAgg')
import numpy as np
import matplotlib.pyplot as plt
import collision_checking
from matplotlib.patches import Circle, Rectangle, Polygon
from matplotlib.collections import PatchCollection

import math

# Constants
L1 = 0.4
L2 = 0.25
W = 0.1
R = 0.05

theta1 = 0
theta2 = 0


class Point:
    def __init__(self, x, y):
        self.x = x
        self.y = y


class Rect:
    def __init__(self, topLeft, topRight, bottomLeft, bottomRight):
        self.topLeft= topLeft
        self.topRight = topRight
        self.bottomLeft = bottomLeft
        self.bottomRight = bottomRight

class Joint:

    def __init__(self, center, arm):
        self.center = Point(center.x, center.y)
        self.curCenter = Point(center.x, center.y)
        self.arm = arm
        self.theta = 0.0
        self.origin = Point(0, 0)

    def draw(self, ax):
        ax.add_patch(Circle((self.curCenter.x, self.curCenter.y), R, color='blue'))

    def move(self, theta):
            angle = theta * math.pi / 180

            rotatedPoints = find_roatated_position(self.origin, self.center, angle)
            self.curCenter.x = rotatedPoints[0]
            self.curCenter.y = rotatedPoints[1]

            self.theta = angle

    def resetOrigin(self, origin):
        self.origin = Point(origin.x, origin.y)
        self.center = Point(origin.x+L2+R, origin.y)

class Arm:

    #(x, y) is the bottomLeft of the rectangle
    def __init__(self, x, y, length):
        #Arm.x = x
        #Arm.y = y
        self.length = length
        #Arm.height = 2*R

        self.rect=Rect(Point(x, y+2*R),
                            Point(x+R+length, y+2*R),
                            Point(x, y),
                            Point(x+R+length, y)
        )

        self.curRect = Rect(Point(x, y+2*R),
                            Point(x+R+length, y+2*R),
                            Point(x, y),
                            Point(x+R+length, y)
        )

        self.theta = 0.0
        self.origin = None
        self.curOrigin = None


    def draw(self, ax):

        nodes = []
        nodes.append((self.curRect.bottomLeft.x, self.curRect.bottomLeft.y))
        nodes.append((self.curRect.bottomRight.x, self.curRect.bottomRight.y))
        nodes.append((self.curRect.topRight.x, self.curRect.topRight.y))
        nodes.append((self.curRect.topLeft.x, self.curRect.topLeft.y))
        poly = np.array(nodes)

        polygon = Polygon(poly, closed=True, edgecolor='orange')
        patches = [polygon]
        p = PatchCollection(patches, edgecolor='orange', alpha=0.4)
        ax.add_collection(p)


    def draw_old(self, ax):
        polygon = []
        polygon.append((self.curRect.bottomLeft.x, self.curRect.bottomLeft.y))
        polygon.append((self.curRect.bottomRight.x, self.curRect.bottomRight.y))
        polygon.append((self.curRect.topRight.x, self.curRect.topRight.y))
        polygon.append((self.curRect.topLeft.x, self.curRect.topLeft.y))

        poly = np.array(polygon)
        ax.fill(poly[:, 0], poly[:, 1], 'g', alpha=0.9)

        polygon = np.concatenate((polygon, [polygon[0]]), axis=0)  # close the polygon
        xs, ys = zip(*polygon)
        plt.plot(xs, ys)



    def move(self, theta):
        angle = theta * math.pi / 180

        #print(" in arm_move joint x =" + str(joint.center.x) + " joint y =" + str(joint.center.y) + " theta =" + str(
         #   arm.theta))

        rotatedPoints = find_roatated_position(self.origin, self.rect.topLeft, angle)
        self.curRect.topLeft.x = rotatedPoints[0]
        self.curRect.topLeft.y = rotatedPoints[1]
        rotatedPoints = find_roatated_position(self.origin, self.rect.topRight, angle)
        self.curRect.topRight.x = rotatedPoints[0]
        self.curRect.topRight.y = rotatedPoints[1]
        rotatedPoints = find_roatated_position(self.origin, self.rect.bottomLeft, angle)
        self.curRect.bottomLeft.x = rotatedPoints[0]
        self.curRect.bottomLeft.y = rotatedPoints[1]
        rotatedPoints = find_roatated_position(self.origin, self.rect.bottomRight, angle)
        self.curRect.bottomRight.x = rotatedPoints[0]
        self.curRect.bottomRight.y = rotatedPoints[1]

        self.theta = angle

    def resetOrigin(self, origin):
        self.origin = Point(origin.x, origin.y)
        self.rect = Rect(Point(self.origin.x, self.origin.y + R),
                         Point(self.origin.x + R + self.length, self.origin.y + R),
                         Point(self.origin.x, self.origin.y-R),
                         Point(self.origin.x + R + self.length, self.origin.y-R)
                         )



# read polygon info from file
def load_polygons_from_file(filename):
    data = np.load(filename, allow_pickle=True)
    return [data[i] for i in range(len(data))]



def collision_space():
    # compute and visualize the collision-free confuguration space
    print ("in collsion_space")

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


def create_arm(joint1, theta1, theta2, joint2, joint3, arm1, arm2):
        joint2 = [joint1[0] + (L1 + R) * math.cos(math.radians(theta1)),
                  joint1[1] + (L1 + R) * math.sin(math.radians(theta1))]
        joint3 = [joint2[0] + (L2 + R) * math.cos(math.radians(theta1 + theta2)),
                  joint2[1] + (L2 + R) * math.sin(math.radians(theta1 + theta2))]

        arm1.x = 1+R
        arm1.y = 1+R

        arm2.x = 1+R+arm1.length
        arm2.y = 1+R


def create_arm_old(joint1, theta1, theta2):
    joint2 = [joint1[0] + (L1 + R) * math.cos(math.radians(theta1)),
              joint1[1] + (L1 + R) * math.sin(math.radians(theta1))]
    joint3 = [joint2[0] + (L2 + R) * math.cos(math.radians(theta1 + theta2)),
              joint2[1] + (L2 + R) * math.sin(math.radians(theta1 + theta2))]
    return joint2, joint3



def plot_arm(ax, j1, j2, j3, arm1, arm2):

    # polygons. This may be deleted after we import the .npy
    polygons = []
    for polygon in polygons:
        ax.fill(polygon[:, 0], polygon[:, 1], 'y', alpha=0.5)

    # drawing the glaph
    j1.draw(ax)
    j2.draw(ax)
    j3.draw(ax)
    arm1.draw(ax)
    arm2.draw(ax)


def plot_arm_old(ax, polygons, joint1, joint2, joint3, collided):
    ax.clear()  # Clear the current axes
    ax.set_aspect('equal', 'box')
    ax.set_xlim([0, 2])
    ax.set_ylim([0, 2])

    # polygons. This may be deleted after we import the .npy
    for polygon in polygons:
        ax.fill(polygon[:, 0], polygon[:, 1], 'y', alpha=0.5)

    # drawing the links (arm segments)
    ax.add_patch(Rectangle((joint1[0], joint1[1] - W / 2), L1, W, angle=theta1, color='r' if collided[0] else 'g'))
    ax.add_patch(
        Rectangle((joint2[0], joint2[1] - W / 2), L2, W, angle=theta1 + theta2, color='r' if collided[1] else 'g'))

    # drawing the joints
    ax.add_patch(Circle(joint1, R, color='r' if collided[2] else 'b'))
    ax.add_patch(Circle(joint2, R, color='r' if collided[3] else 'b'))
    ax.add_patch(Circle(joint3, R, color='r' if collided[4] else 'b'))


def find_roatated_position(origin, point, angle):

    ox = origin.x
    oy = origin.y
    px = point.x
    py = point.y
    #print(" origin x =" + str(origin.x) + "y =" + str(origin.y) + " old x =" + str(point.x) + " y =" + str(point.y))
    rx = ox + (px-ox) * math.cos(angle) - (py-oy) * math.sin(angle)
    ry = oy + (px-ox) * math.sin(angle) + (py-oy) * math.cos(angle)
    #print(" new x =" + str(rx) + " y =" + str(ry))
    return rx, ry


def on_key(event):
    ax.clear()  # Clear the current axes
    ax.set_aspect('equal', 'box')
    ax.set_xlim([0, 2])
    ax.set_ylim([0, 2])

    global theta1, theta2
    if event.key == 'right':
        theta1 += 5
        arm1.move(theta1)
        J2.move(theta1)
        arm2.resetOrigin(J2.curCenter)
        J3.resetOrigin(J2.curCenter)
        #print("reset J3 origin x= "+str(J2.curCenter.x)+" y= "+str(J2.curCenter.y))
        arm2.move(theta1+theta2)
        J3.move(theta1+theta2)
    elif event.key == 'left':
        theta1 -= 5
        arm1.move(theta1)
        J2.move(theta1)
        arm2.resetOrigin(J2.curCenter)
        J3.resetOrigin(J2.curCenter)
        arm2.move(theta1 + theta2)
        J3.move(theta1 + theta2)
    elif event.key == 'up':
        theta2 += 5
        arm2.move(theta1+theta2)
        J3.move(theta1+theta2)
    elif event.key == 'down':
        theta2 -= 5
        arm2.move(theta1+theta2)
        J3.move(theta1+theta2)
    elif event.key == 'c':
        collision_space()

    plot_arm(ax, J1, J2, J3, arm1, arm2)

    for polygon in polygons:
        polygon = np.concatenate((polygon, [polygon[0]]), axis=0)  # close the polygon
        xs, ys = zip(*polygon)
        plt.plot(xs, ys)


    plt.draw()


def main():
    global joint1, joint2, joint3, polygons, collided, ax, arm1, arm2
    global J1, J2, J3, arm1, arm2

    # create arm and joints
    arm1=Arm(1,1-R,L1)
    arm2=Arm(1+2*R+L1,1-R,L2)
    J1 = Joint(Point(1,1), arm1)
    J2 = Joint(Point(1+L1+R, 1), arm2)
    J2.origin=J1.center
    J3 = Joint(Point(1+L1+R+L2+R, 1), None)
    J3.origin=J2.center
    arm1.origin=J1.center
    arm2.origin=J2.center

    #load polygons

    #polygons_state = []

    polygons = load_polygons_from_file(r'E:\Alex\CS460Assignment1\arm_polygons.npy')
    for polygon in polygons:
        # print("Generated Polygon:", polygon)
        print(repr(polygon), end=' ')
        #polygons_state.append(False)

    #collision_checking(polygons)

    fig, ax = plt.subplots(dpi=100)
    ax.clear()  # Clear the current axes
    ax.set_aspect('equal', 'box')
    ax.set_xlim([0, 2])
    ax.set_ylim([0, 2])

    ax.set_aspect('equal')
    for polygon in polygons:
        polygon = np.concatenate((polygon, [polygon[0]]), axis=0)  # close the polygon
        xs, ys = zip(*polygon)
        plt.plot(xs, ys)

    collided = [False, False, False, False, False]


    fig.canvas.mpl_connect('key_press_event', on_key)

    plot_arm(ax, J1, J2, J3, arm1, arm2)
    plt.show()


if __name__ == "__main__":
    main()