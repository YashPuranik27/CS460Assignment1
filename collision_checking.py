# Collision checking between convex polygons

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches


# read polygon info from file
def load_polygons_from_file(filename):
    with np.load(filename, allow_pickle=True) as data:
        return [data[arr] for arr in data]


def collides(polygon1, polygon2):
    # Calculate bounding boxes
    min1, max1 = np.min(polygon1, axis=0), np.max(polygon1, axis=0)
    min2, max2 = np.min(polygon2, axis=0), np.max(polygon2, axis=0)

    # Check if bounding boxes collide
    collide_x = min1[0] < max2[0] and max1[0] > min2[0]
    collide_y = min1[1] < max2[1] and max1[1] > min2[1]
    result = collide_x and collide_y

    return result


def plot(polygons, polygons_state):
    # Visualization
    fig, ax = plt.subplots(dpi=100)
    ax.set_aspect('equal')

    # Draw polygons
    i=0
    for polygon in polygons:
        # print("Generated Polygon:", polygon)
        if (polygons_state[i] == True):
            ax.fill(polygon[:, 0], polygon[:, 1], 'r', alpha=0.5)
        else:
            ax.fill(polygon[:, 0], polygon[:, 1], 'b', alpha=0.5)
        i=i+1

    plt.show()



def plot_old(polygon1, polygon2, check_result):
    # Visualization
    fig, ax = plt.subplots(dpi=100)
    ax.set_aspect('equal')

    # Draw polygons
    ax.fill(polygon1[:, 0], polygon1[:, 1], 'b', label='Polygon 1', alpha=0.5)
    ax.fill(polygon2[:, 0], polygon2[:, 1], 'r', label='Polygon 2', alpha=0.5)

    # Calculate bounding boxes
    min1, max1 = np.min(polygon1, axis=0), np.max(polygon1, axis=0)
    min2, max2 = np.min(polygon2, axis=0), np.max(polygon2, axis=0)

    # Draw bounding boxes
    rect1 = patches.Rectangle(min1, max1[0] - min1[0], max1[1] - min1[1], linewidth=1, edgecolor='b', facecolor='none')
    rect2 = patches.Rectangle(min2, max2[0] - min2[0], max2[1] - min2[1], linewidth=1, edgecolor='r', facecolor='none')

    ax.add_patch(rect1)
    ax.add_patch(rect2)

    ax.legend()

    if check_result:
        ax.set_title("Bounding boxes collide")
    else:
        ax.set_title("Bounding boxes don't collide")
    plt.show()

def main():
    polygons_state = []

    polygons = load_polygons_from_file("collision_checking_polygons.npz")
    for polygon in polygons:
        # print("Generated Polygon:", polygon)
        print(repr(polygon), end=' ')
        polygons_state.append(False)

    #print("num of poly "+str(polygon.ndim))

    #poly1 = polygons[0]
    #poly2 = polygons[1]
    for i in range(len(polygons)-1):
        poly1 = polygons[i]
        print("poly1 -- " + str(i))
        for j in range(i + 1, len(polygons)-1):
            poly2 = polygons[j]
            print("poly2 -- " + str(j))
            check_result = collides(poly1, poly2)
            #plot(poly1, poly2, check_result)
            if polygons_state[i] == False:
                polygons_state[i] = check_result
            if polygons_state[j] == False:
                polygons_state[i] = check_result


    for i in range(len(polygons)):
        print("poly state  " + str(i) + str(polygons_state[i]))

    plot(polygons, polygons_state)


if __name__ == "__main__":
        main()


# Define two polygons
#polygon1 = np.array([[2, 3], [3, 8], [5, 6], [6, 4], [4, 2]])
#polygon2 = np.array([[5, 5], [7, 8], [9, 6], [8, 4]])




