"""Collection of helper classes and functions in order to
solve the field of view objects coding challenge
"""
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
LENGTH_REAR2CAM = 1.72 # meter

class Point2D():
    def __init__(self,x,y):
        self._x = x
        self._y = y
    def x(self):
        return self._x
    def y(self):
        return self._y
    def __add__(self,other):
        x = self._x + other._x
        y = self._y + other._y
        return Point2D(x,y)
    def __sub__(self,other):
        x = self._x - other._x
        y = self._y - other._y
        return Point2D(x,y)

class Camera_FOV():
    # FOV_ANGLE [deg]
    # FOV_X_LAD [m]
    # t0,t1,t2  [m]
    def __init__(self,FOV_ANGLE,FOV_X_LAD):
        delta_y = FOV_X_LAD * np.tan(np.deg2rad((FOV_ANGLE/2)))
        self._t0 = Point2D(LENGTH_REAR2CAM,0)
        self._t1 = Point2D(LENGTH_REAR2CAM + FOV_X_LAD, delta_y)
        self._t2 = Point2D(LENGTH_REAR2CAM + FOV_X_LAD, -delta_y)

class Plot():
    def __init__(self, fov: Camera_FOV):
        self._ego_width = 1.879
        self._ego_length = 4.8
        self._dx_front_bumper = 3.3
        self._dx_rear_bumper = - (self._ego_length - self._dx_front_bumper)
        self._fov = fov
        self._t0 = self.trans_2_world(fov._t0)
        self._t1 = self.trans_2_world(fov._t1)
        self._t2 = self.trans_2_world(fov._t2)
        self.ego_plot()
        self.fov_plot()
    
    def trans_2_world(self, point_din70000: Point2D):
        # transform from DIN70000 to x,y world
        return Point2D(point_din70000.y(), point_din70000.x())

    def ego_plot(self):
        left = plt.plot([-self._ego_width/2, -self._ego_width/2],[self._dx_rear_bumper,self._dx_front_bumper],c="black")
        right = plt.plot([self._ego_width/2, self._ego_width/2],[self._dx_rear_bumper,self._dx_front_bumper],c="black")
        front = plt.plot([-self._ego_width/2, self._ego_width/2],[self._dx_front_bumper,self._dx_front_bumper],c="black")
        rear = plt.plot([-self._ego_width/2, self._ego_width/2],[self._dx_rear_bumper,self._dx_rear_bumper],c="black")
        center = plt.scatter(0,0,c="black")
    
    def fov_plot(self):
        t0_t1 = plt.plot([self._t0.x(), self._t1.x()],[self._t0.y(), self._t1.y()], c="red")
        t1_t2 = plt.plot([self._t1.x(), self._t2.x()],[self._t1.y(), self._t2.y()], c="red")
        t2_t0 = plt.plot([self._t2.x(), self._t0.x()],[self._t2.y(), self._t0.y()], c="red")

def unittest(fov: Camera_FOV, point_inside_fov):
    x = []
    y = []
    z = []
    x_max = int(fov._t1.x()) + 5
    y_max = int(fov._t1.y()) + 5
    for i in range(0, x_max):
        for j in range(-y_max, y_max):
            x.append(i * 1)
            y.append(j * 1)

    for i in range(0, len(x)):
        p = Point2D(x[i], y[i])
        z.append(point_inside_fov(fov, p))

    df = pd.DataFrame({"x": x, "y": y, "z": z,})


    groups = df.groupby("z")
    for name, group in groups:
        plt.plot(-group.y, group.x, marker="o", linestyle="", markersize=4, label=name) #(x,y)
    plt.legend()
    return plt.show()

def point_inside_fov_demo(fov: Camera_FOV, point: Point2D):
    t_ax = fov._t0.x() - point.x()
    t_ay = fov._t0.y() - point.y()
    t_bx = fov._t1.x() - point.x()
    t_by = fov._t1.y() - point.y()
    t_cx = fov._t2.x() - point.x()
    t_cy = fov._t2.y() - point.y()
    
    u = t_bx * t_cy - t_cx * t_by
    v = t_cx * t_ay - t_ax * t_cy
    w = t_ax * t_by - t_bx * t_ay

    if u<=0 and v<=0 and w<=0: 
        return True
    return False

""" other simple solution based on determinant
def area_triangle(a: Point2D, b: Point2D, c: Point2D):
    return 0.5 * np.abs( (a.x()*b.y() - a.y()*b.x()) + (b.x()*c.y() - b.y()*c.x()) + (c.x()*a.y() - c.y()*a.x()))
	
def point_inside_fov(fov: Camera_FOV, point: Point2D):
    A1 = area_triangle(fov._t0, fov._t1, point)
    A2 = area_triangle(fov._t0, fov._t2, point)
    A3 = area_triangle(fov._t1, fov._t2, point)
    A = area_triangle(fov._t0, fov._t1, fov._t2)
    
    return A + 0.001>= A1+A2+A3 
"""