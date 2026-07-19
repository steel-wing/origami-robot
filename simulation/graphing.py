import math
import matplotlib.pyplot as plt
import numpy as np

def calculateTheta2(theta1):
  return 2 * math.atan(math.sqrt(2) * math.tan((theta1) / 2))

def getServoAngle(theta):
    x = -0.30
    y = 0.528
    d = 0.528
    l = 1.325

    if l < (math.sqrt(x * x + y * y) + d):
        return 0

    I1x = x - d * math.sin(theta)
    I1y = y - d * math.cos(theta)
    I2x = I1x + l * math.cos(theta)
    I2y = I1y - l * math.sin(theta)

    dx = I2x - I1x
    dy = I2y - I1y
    drs = dx * dx + dy * dy
    D = I1x * I2y - I2x * I1y

    q = -1 if theta <= math.pi else 1
    s = -1 if dy <= 0 else 1

    sqrt_term = math.sqrt(l * l * drs - D * D)

    xi = (D * dy - q * s * dx * sqrt_term) / drs
    yi = (-D * dx - q * abs(dy) * sqrt_term) / drs

    return math.atan2(yi, -xi)

def bezier_path(t, x1, y1, z1, x2, y2, z2, x3, y3, z3, x4, y4, z4):
    points = [
        np.array([x1, y1, z1]),
        np.array([x2, y2, z2]),
        np.array([x3, y3, z3]),
        np.array([x4, y4, z4]),
        # add more points here if you want, just be sure to add them wherever relevant
    ]
    n = len(points)

    # compute midpoints
    midpoints = [(points[i] + points[(i + 1) % n]) / 2 for i in range(n)]

    def quad_bezier(t, p0, p1, p2):
        return (1 - t)**2 * p0 + 2 * (1 - t) * t * p1 + t**2 * p2

    # build segments dynamically
    segments = [(midpoints[i], points[(i + 1) % n], midpoints[(i + 1) % n]) for i in range(n)]

    def f(t):
        t = t % n                               # wrap around loop
        i = int(t)                              # select which segment to be a part of
        local_t = t - i                         # normalize to [0, 1)
        p0, p1, p2 = segments[i]                # grab control points for curve
        return quad_bezier(local_t, p0, p1, p2)
    
    return f(t)

if __name__ == "__main__":
    start = 0
    end = math.pi
    num_points = 500

    x_vals = np.linspace(start, end, num_points)
    y_vals1 = [getServoAngle(x) for x in x_vals]

    plt.plot(x_vals, y_vals1)
    plt.xlabel("Hinge Angle (rad)")
    plt.ylabel("Servo Angle (rad)")
    plt.axis('equal')
    plt.grid(True)
    ax = plt.gca()
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    plt.show()

    y_vals2 = [calculateTheta2(x) for x in x_vals]
    plt.plot(x_vals, x_vals, color='red', label=r"$\phi_1$ (rad)")
    plt.plot(x_vals, y_vals2, color='blue',  label=r"$\phi_2$ (rad)")
    plt.xlabel(r"$\phi_1$ (rad)")
    plt.ylabel(r"$\phi_2$ (rad)")
    plt.legend()
    plt.grid(True)
    ax = plt.gca()
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.set_aspect('equal', adjustable='box')
    plt.show()

    y_vals3 = [getServoAngle(calculateTheta2(x)) for x in x_vals]
    plt.plot(x_vals, y_vals1, color='red',  label=r"$\phi_1$ (rad)")
    plt.plot(x_vals, y_vals3, color='blue',  label=r"$\phi_2$ (rad)")
    plt.xlabel(r"Hinge Angle, $\phi_{hinge}$ (rad)")
    plt.ylabel(r"Servo Angle, $\phi_{servo}$ (rad)")
    plt.legend()
    plt.grid(True)
    ax = plt.gca()
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.set_aspect('equal', adjustable='box')
    plt.show() 

    t_start = 0
    t_end = 4
    func = lambda t: bezier_path(t, 0.2, 2.0, 2.0, 0.2, 0.2, 0.2, 2.0, 0.2, 0.2, 1.7, 1.8, 0.2,)
    t_vals = np.linspace(t_start, t_end, num_points)
    pts = np.array([func(t) for t in t_vals])
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')
    # equal scaling
    max_range = np.array([x.max()-x.min(), y.max()-y.min(), z.max()-z.min()]).max() / 2.0
    ax.set_xlim(0, 2.1)
    ax.set_ylim(0, 2.1)
    ax.set_zlim(0, 2.1)
    ax.plot(x, y, z, color='tab:blue', label="Bezier Path")
    control_points = [np.array([0.2, 2.0, 2.0,]), np.array([0.2, 0.2, 0.2,]), np.array([2.0, 0.2, 0.2, ]), np.array([1.7, 1.8, 0.2,])]

    gait_points = [np.array([1.1, 0.2, 0.2]),  np.array([1.85, 1.0, 0.2]),  np.array([0.95, 1.9, 1.1]),  np.array([0.2, 1.1, 1.1])]

    cp = np.array(control_points)
    gp = np.array(gait_points)
    ax.scatter(cp[:,0], cp[:,1], cp[:,2], color='red', label="Control Points")
    ax.scatter(gp[:,0], gp[:,1], gp[:,2], color='blue', label="Gait Sample Points")
    ax.scatter(cp[:,0], cp[:,1], 0.1, color='black', label="Control Point Projections")
    cp_closed = np.vstack([cp, cp[0]])
    ax.plot(cp_closed[:,0], cp_closed[:,1], cp_closed[:,2], linestyle='--', color='gray')

    ax.set_xlabel("Left Vertex (rad)")
    ax.set_ylabel("Middle Hinge (rad)")
    ax.set_zlabel("Right Vertex (rad)")
    ax.legend()
    plt.show()