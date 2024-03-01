#!/usr/bin/env python
# coding: utf-8


from vedo import *
#import pyautogui

def RotationMatrix(theta, axis_name):
    """ calculate single rotation of $theta$ matrix around x,y or z
        code from: https://programming-surgeon.com/en/euler-angle-python-en/
    input
        theta = rotation angle(degrees)
        axis_name = 'x', 'y' or 'z'
    output
        3x3 rotation matrix
    """

    c = np.cos(theta * np.pi / 180)
    s = np.sin(theta * np.pi / 180)
	
    if axis_name =='x':
        rotation_matrix = np.array([[1, 0,  0],
                                    [0, c, -s],
                                    [0, s,  c]])
    if axis_name =='y':
        rotation_matrix = np.array([[ c,  0, s],
                                    [ 0,  1, 0],
                                    [-s,  0, c]])
    elif axis_name =='z':
        rotation_matrix = np.array([[c, -s, 0],
                                    [s,  c, 0],
                                    [0,  0, 1]])
    return rotation_matrix


def createCoordinateFrameMesh():
    """Returns the mesh representing a coordinate frame
    Args:
      No input args
    Returns:
      F: vedo.mesh object (arrows for axis)
      
    """         
    _shaft_radius = 0.05
    _head_radius = 0.10
    _alpha = 1
    
    
    # x-axis as an arrow  
    x_axisArrow = Arrow(start_pt=(0, 0, 0),
                        end_pt=(1, 0, 0),
                        s=None,
                        shaft_radius=_shaft_radius,
                        head_radius=_head_radius,
                        head_length=None,
                        res=12,
                        c='red',
                        alpha=_alpha)

    # y-axis as an arrow  
    y_axisArrow = Arrow(start_pt=(0, 0, 0),
                        end_pt=(0, 1, 0),
                        s=None,
                        shaft_radius=_shaft_radius,
                        head_radius=_head_radius,
                        head_length=None,
                        res=12,
                        c='green',
                        alpha=_alpha)

    # z-axis as an arrow  
    z_axisArrow = Arrow(start_pt=(0, 0, 0),
                        end_pt=(0, 0, 1),
                        s=None,
                        shaft_radius=_shaft_radius,
                        head_radius=_head_radius,
                        head_length=None,
                        res=12,
                        c='blue',
                        alpha=_alpha)
    
    originDot = Sphere(pos=[0,0,0], 
                       c="black", 
                       r=0.10)


    # Combine the axes together to form a frame as a single mesh object 
    F = x_axisArrow + y_axisArrow + z_axisArrow + originDot
        
    return F


def getLocalFrameMatrix(R_ij, t_ij): 
    """Returns the matrix representing the local frame
    Args:
      R_ij: rotation of Frame j w.r.t. Frame i 
      t_ij: translation of Frame j w.r.t. Frame i 
    Returns:
      T_ij: Matrix of Frame j w.r.t. Frame i. 
      
    """             
    # Rigid-body transformation [ R t ]
    T_ij = np.block([[R_ij,                t_ij],
                     [np.zeros((1, 3)),       1]])
    
    return T_ij

def forward_kinematics(Phi, L1, L2, L3, L4):
    phi_extended = np.array(Phi)
    phi_extended = np.insert(phi_extended,len(phi_extended) - 1,0)
    local_frames = np.array([[[3],[2],[0]],[[L1+0.8],[0],[0]],[[L2+0.8],[0],[0]],[[L3+0.4],[0],[0]],[[L4],[0],[0]]])
    global_frames = [getLocalFrameMatrix(RotationMatrix(phi_extended[0], 'z'), local_frames[0])]
    for i in range(1,5):
        current = getLocalFrameMatrix(RotationMatrix(phi_extended[i], 'z'), local_frames[i])
        global_frames.append(global_frames[i-1] @ current)

    return global_frames[0], global_frames[1], global_frames[2], global_frames[3], [frame[3] for frame in global_frames[4][0:3]]

def forward_kinematics_practical(Phi, L1, L2, L3, L4, radius):
    phi_extended = np.array(Phi)
    phi_extended = np.insert(phi_extended,len(phi_extended) - 1,0)
    local_frames = np.array([[[3],[2],[0]],[[L1+radius*2],[0],[0]],[[L2+radius*2],[0],[0]],[[L3+radius*2],[0],[0]],[[L4],[0],[0]]])
    global_frames = [getLocalFrameMatrix(RotationMatrix(phi_extended[0], 'z'), local_frames[0])]
    for i in range(1,5):
        current = getLocalFrameMatrix(RotationMatrix(phi_extended[i], 'z'), local_frames[i])
        global_frames.append(global_frames[i-1] @ current)

    return global_frames


def main():
    # Set the limits of the graph x, y, and z ranges 
    axes = Axes(xrange=(0,20), yrange=(-2,10), zrange=(0,6))

    # Specifications for each arm & end effector
    lengths = [8,6,4,0.5]
    # Note that we will use spheres of radius "radius" as joints
    radius = 0.4
    # Animation Loop 
    # Base of arm is assumed to be fixed at 3,2,0
    fps = 30
    duration = 5
    angles = [0,0,0,0]
    angle_flags = [1,1,1,1]
    
    for k in range(0,fps*duration):
        for i in range(0,4):
            test = angles[i] + np.random.rand(1,1) * 5 * angle_flags[i]
            if test >= 90 or test <= -90:
                angle_flags[i] *= -1
                test = angles[i] + np.random.rand(1,1) * 5 * angle_flags[i]
            angles[i] = test;
        global_frames = forward_kinematics_practical(angles,lengths[0],lengths[1],lengths[2],lengths[3], radius)
        
        # create the components and place them
        frame1Arrows = createCoordinateFrameMesh()
        for i in range(0,4):
            p_s = [frame[3] for frame in global_frames[i][0:3]]
            c = Cylinder(r = radius, height = lengths[i], pos = (lengths[i]/2 + radius,0,0),c = "black", alpha = 1, axis = (1,0,0))
            c.apply_transform(global_frames[i])
            s = Sphere(r = radius).pos(p_s[0],p_s[1],p_s[2]).color("yellow").alpha(.7)
            frame1Arrows += c + s
    
        show(frame1Arrows, axes, viewup="y").close()
        #image = pyautogui.screenshot(region=(20,50,950,930))
        #image.save(r"animation\screenshot" + str(k) + ".png")
        
        


if __name__ == '__main__':
    main()



