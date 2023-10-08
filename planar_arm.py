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

    def toString(self):

        return ('['+str(self.topLeft.x)+','+str(self.topLeft.y)+']'+
                '[' + str(self.topRight.x) + ',' + str(self.topRight.y) + ']' +
                '[' + str(self.bottomLeft.x) + ',' + str(self.bottomLeft.y) + ']' +
                '[' + str(self.bottomRight.x) + ',' + str(self.bottomRight.y) + ']'
                )



class Joint:

    def __init__(self, center, arm):
        self.center = Point(center.x, center.y)
        self.curCenter = Point(center.x, center.y)
        self.prevCenter = Point(center.x, center.y)
        self.arm = arm
        self.theta = 0.0
        self.origin = Point(0, 0)
        self.moved = False

    def draw(self, ax, collied):
        color = 'b'
        if collied:
            color = 'r'

        ax.add_patch(Circle((self.curCenter.x, self.curCenter.y), R, color=color))

    def copyPoint(self, fromPoint, toPoint):
        toPoint.x = fromPoint.x
        toPoint.y = fromPoint.y

    def move(self, theta):
            angle = theta * math.pi / 180
            self.copyPoint(self.curCenter, self.prevCenter)
            rotatedPoints = find_roatated_position(self.origin, self.center, angle)
            self.curCenter.x = rotatedPoints[0]
            self.curCenter.y = rotatedPoints[1]
            self.moved=True

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

        self.prevRect = Rect(Point(x, y + 2 * R),
                            Point(x + R + length, y + 2 * R),
                            Point(x, y),
                            Point(x + R + length, y)
                            )

        self.theta = 0.0
        self.origin = None
        self.curOrigin = None
        self.moved = False


    def draw(self, ax, collied):

        color = 'g'
        if collied:
            color = 'r'

        nodes = []
        nodes.append((self.curRect.bottomLeft.x, self.curRect.bottomLeft.y))
        nodes.append((self.curRect.bottomRight.x, self.curRect.bottomRight.y))
        nodes.append((self.curRect.topRight.x, self.curRect.topRight.y))
        nodes.append((self.curRect.topLeft.x, self.curRect.topLeft.y))
        poly = np.array(nodes)

        polygon = Polygon(poly, closed=True, edgecolor=color)
        patches = [polygon]
        p = PatchCollection(patches, edgecolor=color, alpha=0.4)
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

    def toPoly(self):
        poly = []
        poly.append((self.curRect.bottomLeft.x, self.curRect.bottomLeft.y))
        poly.append((self.curRect.bottomRight.x, self.curRect.bottomRight.y))
        poly.append((self.curRect.topRight.x, self.curRect.topRight.y))
        poly.append((self.curRect.topLeft.x, self.curRect.topLeft.y))

        return poly

    def toPolyArray(self):
        poly = []
        poly.append([self.curRect.bottomLeft.x, self.curRect.bottomLeft.y])
        poly.append([self.curRect.bottomRight.x, self.curRect.bottomRight.y])
        poly.append([self.curRect.topRight.x, self.curRect.topRight.y])
        poly.append([self.curRect.topLeft.x, self.curRect.topLeft.y])

        return poly

    def copyRect(self, fromRect, toRect):
        toRect.topLeft.x=fromRect.topLeft.x
        toRect.topLeft.y=fromRect.topLeft.y
        toRect.topRight.x=fromRect.topRight.x
        toRect.topRight.y=fromRect.topRight.y
        toRect.bottomLeft.x=fromRect.bottomLeft.x
        toRect.bottomLeft.y=fromRect.bottomLeft.y
        toRect.bottomRight.x=fromRect.bottomRight.x
        toRect.bottomRight.y=fromRect.bottomRight.y

    def move(self, theta):
        angle = theta * math.pi / 180

        #print(" in arm_move joint x =" + str(joint.center.x) + " joint y =" + str(joint.center.y) + " theta =" + str(
         #   arm.theta))

        self.copyRect(self.curRect, self.prevRect)

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
        self.moved = True


    def resetOrigin(self, origin):
        self.origin = Point(origin.x, origin.y)
        self.rect = Rect(Point(self.origin.x, self.origin.y + R),
                         Point(self.origin.x + R + self.length, self.origin.y + R),
                         Point(self.origin.x, self.origin.y-R),
                         Point(self.origin.x + R + self.length, self.origin.y-R)
                         )



def collision_space():
    # compute and visualize the collision-free confuguration space
    print ("in collsion_space")

def check_all_collisions():
    collided = [False, False, False, False, False]

    # Check each part of the arm for collision
    #collided[0] = check_collision_with_polygons(arm1, polyGraph)
    #collided[1] = check_collision_with_polygons(arm2, polyGraph)
    #collided[2] = check_collision_with_polygons(J1, polyGraph)
    #collided[3] = check_collision_with_polygons(J2, polyGraph)
    #collided[4] = check_collision_with_polygons(J3, polyGraph)

    #return collided

    return False

def arm_collision_check(arm):

    for i in range(len(polyGraph)-1):
        poly1 = polyGraph[i]
        if collision_checking.collides_bounding_box(poly1, arm.toPolyArray())==True:
            print("collision check true!!!!!!!!!!!!!!!!!!!!")
            return True

    return False


def joint_collsion_check(joint):
    return False

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



def plot_graph(ax, collied):
    # drawing the glaph
    ax.clear()  # Clear the current axes
    ax.set_aspect('equal', 'box')
    ax.set_xlim([0, 2])
    ax.set_ylim([0, 2])

    J1.draw(ax, collied)
    J2.draw(ax, collied)
    J3.draw(ax, collied)
    arm1.draw(ax, collied)
    arm2.draw(ax, collied)

    # draw polygon objects
    for polygon in polyGraph:
        polygon = np.concatenate((polygon, [polygon[0]]), axis=0)  # close the polygon
        xs, ys = zip(*polygon)
        plt.plot(xs, ys)


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

def setToNotMoved():
    arm1.moved=False
    arm2.moved=False
    J1.moved=False
    J2.moved=False
    J3.moved=False

def restorePrevPosition():

    print("in restore")
    if arm1.moved == True:
        arm1.copyRect(arm1.prevRect, arm1.curRect)
        #print("restore arm1")
    if arm2.moved == True:
        #print("before restore arm2")
        #print("prev: " + arm2.prevRect.toString() + "\n")
        #print("cur: " + arm2.curRect.toString() + "\n\n")
        arm2.copyRect(arm2.prevRect, arm2.curRect)
        #print("after restore arm2")
        #print("prev: " + arm2.prevRect.toString() + "\n")
        #print("cur: " + arm2.curRect.toString() + "\n\n")
    if J2.moved == True:
        J2.copyPoint = (J2.prevCenter, J2.curCenter)
        #print("restore joint2")
    if J3.moved == True:
        J3.copyPoint(J3.prevCenter, J3.curCenter)
        #print("restore joint3")

    theta1 = prevTheta1
    theta2 = prevTheta2

def on_key(event):

    global theta1, theta2, prevTheta1, prevTheta2


    prevTheta1 = theta1
    prevTheta2 = theta2
    collied = False
    setToNotMoved()

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
        if arm_collision_check(arm2) == True:
            print("-------------------COLLISION-----------------")
            collied = True
        if joint_collsion_check(J3) == True:
            collied = True

    elif event.key == 'down':
        theta2 -= 5
        arm2.move(theta1+theta2)
        J3.move(theta1+theta2)
    elif event.key == 'c':
        collision_space()


    if collied == True:
        print("restore position     -----------------------")
        restorePrevPosition()
        plot_graph(ax, False)
    else:
        plot_graph(ax, False)
        #plt.draw()
        #plt.pause(.002)

    plt.draw()

    setToNotMoved()




def main():
    global joint1, joint2, joint3, polyGraph, collided, ax, arm1, arm2
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

    polyGraph = collision_checking.load_polygons_from_file(r'E:\Alex\CS460Assignment1\arm_polygons.npy')
    for polygon in polyGraph:
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
    #for polygon in polyGraph:
    #    polygon = np.concatenate((polygon, [polygon[0]]), axis=0)  # close the polygon
    #    xs, ys = zip(*polygon)
    #    plt.plot(xs, ys)

    collided = [False, False, False, False, False]


    fig.canvas.mpl_connect('key_press_event', on_key)

    plot_graph(ax, False)
    plt.show()


if __name__ == "__main__":
    main()