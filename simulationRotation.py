import tkinter as tk
import math
import time

# Constants
CANVAS_SIZE = 800
WALLS = [[200, 215], [200, 415], [600, 415], [600, 215]]
ROTATION_ANGLE = 2  # Degrees per step
INTERVAL = 200  # ms
CYCLES = 180
LASER_ANGLE_OFFSET = 12.5  # Degrees

# Points for sensors
A = [400, 400]
B = [396, 400]
C = [404, 400]

# Data to store laser lengths
laserCentral = []
laserLeft = []
laserRight = []


def rotate_point(px, py, ox, oy, angle):
    """Rotate point (px, py) around (ox, oy) by angle in degrees."""
    radians = math.radians(angle)
    cos_val = math.cos(radians)
    sin_val = math.sin(radians)
    nx = cos_val * (px - ox) - sin_val * (py - oy) + ox
    ny = sin_val * (px - ox) + cos_val * (py - oy) + oy
    return [nx, ny]


def line_intersection(p1, p2, q1, q2):
    """Find intersection of two lines (p1-p2 and q1-q2)."""
    def det(a, b, c, d):
        return a * d - b * c

    x1, y1, x2, y2 = p1[0], p1[1], p2[0], p2[1]
    x3, y3, x4, y4 = q1[0], q1[1], q2[0], q2[1]

    det_val = det(x1 - x2, y1 - y2, x3 - x4, y3 - y4)
    if det_val == 0:
        return None  # Lines are parallel

    px = det(det(x1, y1, x2, y2), x1 - x2, det(x3, y3, x4, y4), x3 - x4) / det_val
    py = det(det(x1, y1, x2, y2), y1 - y2, det(x3, y3, x4, y4), y3 - y4) / det_val
    return [px, py]


def find_laser_end(sensor, angle, walls):
    """Find where a laser from sensor intersects the walls."""
    laser_dir = [sensor[0] + 1000 * math.cos(math.radians(angle)),
                 sensor[1] - 1000 * math.sin(math.radians(angle))]
    intersections = []
    for i in range(len(walls)):
        wall_start = walls[i]
        wall_end = walls[(i + 1) % len(walls)]
        intersection = line_intersection(sensor, laser_dir, wall_start, wall_end)
        if intersection and intersection[1] < sensor[1]:
            intersections.append(intersection)
    # Find the closest intersection point
    if intersections:
        
        return min(intersections, key=lambda p: math.dist(sensor, p))
    return laser_dir


def update_simulation():
    """Update the simulation by rotating walls and recalculating lasers."""
    global WALLS, cycle_count

    # Rotate walls around point A
    WALLS = [rotate_point(x, y, A[0], A[1], ROTATION_ANGLE) for x, y in WALLS]

    # Recalculate laser endpoints
    laser_central_end = find_laser_end(A, 90, WALLS)
    laser_left_end = find_laser_end(B, 90 + LASER_ANGLE_OFFSET, WALLS)
    laser_right_end = find_laser_end(C, 90 - LASER_ANGLE_OFFSET, WALLS)

    # Calculate and store laser lengths
    laserCentral.append(math.dist(A, laser_central_end))
    laserLeft.append(math.dist(B, laser_left_end))
    laserRight.append(math.dist(C, laser_right_end))

    # Update canvas
    canvas.delete("all")

    # Draw walls
    for i in range(len(WALLS)):
        canvas.create_line(*WALLS[i], *WALLS[(i + 1) % len(WALLS)], fill="black", width=2)

    # Draw lasers
    canvas.create_line(*A, *laser_central_end, fill="red")
    canvas.create_line(*B, *laser_left_end, fill="blue")
    canvas.create_line(*C, *laser_right_end, fill="green")

    # Draw sensors
    canvas.create_oval(A[0] - 7, A[1] - 7, A[0] + 7, A[1] + 7, fill="red")
    canvas.create_oval(B[0] - 7, B[1] - 7, B[0] + 7, B[1] + 7, fill="blue")
    canvas.create_oval(C[0] - 7, C[1] - 7, C[0] + 7, C[1] + 7, fill="green")

    # Check if simulation should continue
    cycle_count += 1
    if cycle_count < 100000:
        time.sleep(0.2)
        root.after(1, update_simulation)
    else:
        # Print results after simulation ends
        print("Laser Central:", laserCentral)
        print("Laser Left:", laserLeft)
        print("Laser Right:", laserRight)


# Initialize GUI
root = tk.Tk()
root.title("Laser Simulation")

canvas = tk.Canvas(root, width=CANVAS_SIZE, height=CANVAS_SIZE, bg="white")
canvas.pack()

cycle_count = 0
update_simulation()
root.mainloop()
