import tkinter as tk
from tkinter import filedialog
import numpy as np
from numpy import deg2rad as rad
from numpy import rad2deg as deg
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
import quaternionic 

from quaternionic import converters
from dual_quaternions import DualQuaternion
from helper_functions import *

from pymycobot.mycobot import MyCobot
from pymycobot import PI_PORT, PI_BAUD
from ikpy.chain import Chain

dual_quaternions=[]
path_coords=[]
robot=Chain.from_urdf_file("mycobot_280_pi.urdf",base_elements=["g_base"])

def open_file():
    global selected_coords
    file = filedialog.askopenfilename(title="Select File", filetypes=(("Text files", "*.txt*"), ("all files", "*.*")))
    if file:
        selected_coords = parse_pose(file)
        update_motion()

def update_motion():
    update_listbox()

    global dual_quaternions
    dual_quaternions = []
    if selected_coords:
        for coord in selected_coords:
            temp_quaternion = quaternionic.array(to_quaternion(rad(coord[3]), rad(coord[4]), rad(coord[5])))
            print(temp_quaternion.tolist())
            temp_quaternion = temp_quaternion.tolist()
            temp_quaternion.extend(coord[0:3])
            temp_dq = DualQuaternion.from_quat_pose_array(temp_quaternion)
            dual_quaternions.append(temp_dq)
    print(dual_quaternions)
    createPath()

def createPath():
    global path_coords, dual_quaternions, joint_array
    path_coords=[] 
    joint_array=[]
    points=30
    for i in range(points+1):
        temp_dq=(i/points)*dual_quaternions[1]+(1-i/points)*dual_quaternions[0]
        temp_matrix=temp_dq.homogeneous_matrix()
        joint_array.append(robot.inverse_kinematics_frame(temp_matrix))
        temp_quat_pose=temp_dq.quat_pose_array()
        print(quaternionic.array(temp_quat_pose[0:4]))

        temp_angles=to_euler_angles(temp_quat_pose[0:4])
        temp_pose=temp_quat_pose[4:7]
        temp_angles = [deg(c) for c in temp_angles]
        # temp_angles=[deg(temp_angles[0]-180),-1*(deg(temp_angles[1])),deg(temp_angles[2]+180)]
        temp_pose.extend(temp_angles)
        path_coords.append(temp_pose)
        #robot.plot(joint_array[-1],ax)
        draw_axis(temp_pose,ax,np)
    print(path_coords)
    print(joint_array)
    #robot.plot(joint_array[0],ax)

def run_motion():
    for c in path_coords:
    #for c in joint_array:
        #mc.send_radians(radians=c[1:7], speed=20)
        mc.send_coords(path_coords,speed=20,mode=1)
        while mc.is_moving() == 1:
            pass
        if mc.is_moving() == -1:
            print(mc.get_error_information())
            return
    return

def update_listbox():
    listbox.delete(0, tk.END)
    if selected_coords:
        for i, coord in enumerate(selected_coords):
            listbox.insert(i, str(coord))

def release_servo():
    mc.release_all_servos()

root = tk.Tk()
root.wm_title("Embedding in Tk")
#mc = MyCobot(PI_PORT, PI_BAUD)
selected_coords = None

# Frames for layout
left_frame = tk.Frame(root)
right_frame = tk.Frame(root)

# Listbox in the left frame
listbox = tk.Listbox(left_frame, height=10, width=15, bg="grey", activestyle='dotbox', font="Helvetica")
listbox.pack(padx=10, pady=10)
left_frame.pack(side=tk.LEFT, fill=tk.Y)

# Plot in the right frame
fig = Figure(figsize=(5, 4), dpi=100)
ax = fig.add_subplot(111, projection="3d")
t = np.arange(0, 3, .01)
#ax.plot(t, 2 * np.sin(2 * np.pi * t))
ax.set_xbound(-.4,.4)
ax.set_ybound(-.4,.4)
ax.set_zlim(0, .6)

canvas = FigureCanvasTkAgg(fig, master=right_frame)
canvas.draw()
canvas.get_tk_widget().pack()

toolbar = NavigationToolbar2Tk(canvas, right_frame)
toolbar.update()

right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

# Control buttons
button_frame = tk.Frame(root)
tk.Button(button_frame, text="Browse", command=open_file).pack(side=tk.LEFT, padx=10)
tk.Button(button_frame, text="Release Servos",command=release_servo).pack(side=tk.BOTTOM, pady=10)
tk.Button(button_frame, text="Run Motion", command=run_motion).pack(side=tk.LEFT, padx=10)


button_frame.pack(fill=tk.X)

root.mainloop()
