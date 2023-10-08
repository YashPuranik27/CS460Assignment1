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


class Arm:
    #(x, y) is the bottomLeft of the rectangle
    def __init__(self, x, y, length):
        #Arm.x = x
        #Arm.y = y
        self.lenght = length
        #Arm.height = 2*R
        self.center=Point(0,0)

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

def findRotatedPoints(cx, cy, vx, vy, rotatedAngle, newX, newY):
    rotatedAngle = rotatedAngle * math.PI / 180 # convert angle to radians

    # cx, cy is the center of rectangle
    # vx, vy is the vertex to be converted

    dx = vx - cx
    dy = vy - cy
    distance = cx + math.sqrt(dx*dx + dy*dy)
    preAngle = math.atan2(dy, dx)

    newX = cx + distance * math.cos(preAngle + rotatedAngle)
    newY = cy = distance * math.sin(preAngle + rotatedAngle)


def findRoatedRectangleVertex(rect):

    cx = rect.x + (rect.w / 2)
    cy = rect.y + (rect.h / 2)

    topleft  = findRotatedPoints(cx, cy, rect.x, rect.y, rect.rotation)
    topright = findRotatedPoints(cx, cy, rect.x+rect.w, rect.y, rectangle.rotation)
    bottomLeft = findRotatedPoints(cx, cy, rect.x, rect.y+rect.h, rect.rotation)
    topleft = findRotatedPoints(cx, cy, rect.x+rect.w, rect.y+rect.h, rect.rotation)

    #print("orig rect = "+rect.)



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



def plot_arm_new(ax, joint, arm):
    #ax.clear()  # Clear the current axes
    #ax.set_aspect('equal', 'box')
    #ax.set_xlim([0, 2])
    #ax.set_ylim([0, 2])

    # drawing the joints
    ax.add_patch(Circle((J1.center.x, J1.center.y), R, color='purple'))
    # draw the arm
    polygon = []
    polygon.append((arm.curRect.bottomLeft.x, arm.curRect.bottomLeft.y))
    polygon.append((arm.curRect.bottomRight.x, arm.curRect.bottomRight.y))
    polygon.append((arm.curRect.topRight.x, arm.curRect.topRight.y))
    polygon.append((arm.curRect.topLeft.x, arm.curRect.topLeft.y))

    polygon = np.concatenate((polygon, [polygon[0]]), axis=0)  # close the polygon
    xs, ys = zip(*polygon)
    plt.plot(xs, ys)


def plot_arm(ax, polygons, joint1, joint2, joint3, collided):
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
    print(" old x =" + str(point.x) + " old y =" + str(point.y))
    rx = ox + (px-ox) * math.cos(angle) - (py-oy) * math.sin(angle)
    ry = oy + (px-ox) * math.sin(angle) + (py-oy) * math.cos(angle)
    print(" new x =" + str(rx) + " new y =" + str(ry))
    return rx, ry


#move joint
def joint_move(origin, joint, theta):

    angle = theta * math.pi /180

    rotatedPoints = find_roatated_position(origin.center, joint.curCenter, angle)
    joint.curCenter.x = rotatedPoints[0]
    joint.curCenter.y = rotatedPoints[1]

    joint.theta = angle


#move arm
def arm_move(joint, arm, theta):

    angle = theta * math.pi /180

    print(" in arm_move joint x =" + str(joint.center.x) + " joint y =" + str(joint.center.y) + " theta =" + str(arm.theta))

    rotatedPoints = find_roatated_position(arm.origin, arm.rect.topLeft, angle)
    arm.curRect.topLeft.x = rotatedPoints[0]
    arm.curRect.topLeft.y = rotatedPoints[1]
    rotatedPoints = find_roatated_position(arm.origin, arm.rect.topRight, angle)
    arm.curRect.topRight.x = rotatedPoints[0]
    arm.curRect.topRight.y = rotatedPoints[1]
    rotatedPoints = find_roatated_position(arm.origin, arm.rect.bottomLeft, angle)
    arm.curRect.bottomLeft.x = rotatedPoints[0]
    arm.curRect.bottomLeft.y = rotatedPoints[1]
    rotatedPoints = find_roatated_position(arm.origin, arm.rect.bottomRight, angle)
    arm.curRect.bottomRight.x = rotatedPoints[0]
    arm.curRect.bottomRight.y = rotatedPoints[1]

    arm.theta = angle


def on_key(event):
    global theta1, theta2
    if event.key == 'right':
        theta1 += 5
        print("move arm1 ")
        arm_move(J1, arm1, theta1)
        joint_move(J1, J2, theta1)
        #arm_move(J2, arm2, theta1)
        #joint_move(J2, J3, theta1)
    elif event.key == 'left':
        theta1 -= 5
        arm_move(J1, arm1, theta1)
        joint_move(J1, J2, theta1)
        arm_move(J2, arm2, theta1)
        joint_move(J2, J3, theta1)
    elif event.key == 'up':
        theta2 += 5
        joint_move(J1, J2, theta2)
        arm_move(J2, arm2, theta2)
        joint_move(J2, J3, theta2)
    elif event.key == 'down':
        theta2 -= 5
        joint_move(J1, J2, theta2)
        arm_move(J2, arm2, theta2)
        joint_move(J2, J3, theta2)
    elif event.key == 'c':
        collision_space()

    plot_arm(ax, polygons, joint1, joint2, joint3, collided)
    plot_arm_new(ax, J1, arm1)
    plt.draw()


def on_key_old(event):
    global theta1, theta2
    if event.key == 'right':
        theta1 += 5
    elif event.key == 'left':
        theta1 -= 5
    elif event.key == 'up':
        theta2 += 5
    elif event.key == 'down':
        theta2 -= 5
    elif event.key == 'c':
        collision_space()


    joint2, joint3 = create_arm_old(joint1, theta1, theta2)

    plot_arm(ax, polygons, joint1, joint2, joint3, collided)
    plt.draw()


def main():
    global joint1, joint2, joint3, polygons, collided, ax, arm1, arm2
    global J1, J2, J3

    polygons = []  # Placeholder, replace with the .npy file

    arm1=Arm(1,1-R,L1)
    arm2=Arm(1+2*R+L1,1-R,L2)
    J1 = Joint(Point(1,1), arm1)
    J2 = Joint(Point(1+L1+R, 1), arm2)
    J3 = Joint(Point(1+L1+R+L2+R, 1), None)
    arm1.origin=J1.center
    #arm2.joint=J2

    #create_arm(joint1, theta1, theta2, joint2, joint3, arm1, arm2)

    joint1 = [1, 1]
    joint2, joint3 = create_arm_old(joint1, theta1, theta2)


    collided = [False, False, False, False, False]

    fig, ax = plt.subplots()
    fig.canvas.mpl_connect('key_press_event', on_key)

    plot_arm(ax, polygons, joint1, joint2, joint3, collided)
    plt.show()



def main_old():
    global joint1, polygons, collided, ax, arm1, arm2, joint2, joint3

    polygons = []  # Placeholder, replace with the .npy file

    joint1 = [1, 1]

    arm1=Arm(0,0,L1)
    arm2=Arm(0,0,L2)

    #create_arm(joint1, theta1, theta2, joint2, joint3, arm1, arm2)

    joint2, joint3 = create_arm_old(joint1, theta1, theta2)

    collided = [False, False, False, False, False]

    fig, ax = plt.subplots()
    fig.canvas.mpl_connect('key_press_event', on_key_old)

    plot_arm(ax, polygons, joint1, joint2, joint3, collided)
    plt.show()


if __name__ == "__main__":
    main()