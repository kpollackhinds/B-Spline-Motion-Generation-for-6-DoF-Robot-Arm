import tkinter as tk
from tkinter import ttk
from tkinter import filedialog
from tkinter import messagebox
import numpy as np
from numpy import deg2rad as rad
from numpy import rad2deg as deg
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
import quaternionic 
from quaternionic import converters
from dual_quaternions import DualQuaternion
import time

from helper_functions import *
from bspline import *

from pymycobot.mycobot import MyCobot
from pymycobot import PI_PORT, PI_BAUD
import roboticstoolbox as rtb
import os
import csv

dual_quaternions=[]
path_coords=[]
selected_coords=[]
Spline_degree=1
control_points=None
move_speed=30
arm=rtb.Robot.URDF(os.getcwd()+"/mycobot_280_pi.urdf")

def reset():
    global dual_quaternions, path_coords, selected_coords, Spline_degree, control_points
    dual_quaternions=[]
    path_coords=[]
    selected_coords=[]
    Spline_degree=1
    control_points=None
    degree_textbox.delete(0,tk.END)
    control_pts_textbox.delete(0,tk.END)
    update_motion()

def open_file():
    reset()
    global selected_coords
    file = filedialog.askopenfilename(title="Select File", filetypes=(("Text files", "*.txt*"), ("all files", "*.*")))
    if file:
        selected_coords = parse_pose(file)
        update_motion()

def save_to_file():
    file_path = filedialog.asksaveasfilename(defaultextension=".txt", filetypes=[("Text files", "*.txt"), ("All files", "*.*")])

    if file_path:  
        with open(file_path, mode='w', newline='') as file:
            writer = csv.writer(file, delimiter=',')
            for row in selected_coords:
                writer.writerow(row)
    pass


def set_coords():
    global selected_coords
    new_selected_coords = []
    win = tk.Toplevel()
    win.wm_title("Choose Coordinates")
    win.geometry("400x400")
    win.grab_set()
    win.focus_set()

    popup_listbox = tk.Listbox(win, height=10, width=50, bg="light grey", activestyle='dotbox', font=("Helvetica", 8))
    popup_listbox.grid(row=1, columnspan=2)

    # select_button = ttk.Button(win, text="Add Position", command= lambda: (new_selected_coords.append(mc.get_coords()),    
    #                                                                        listbox.delete(0, tk.END),
    #                                                                        listbox.insert((i,str(c)) for i,c in enumerate(new_selected_coords))))
    
    select_button = ttk.Button(win, text="Add Position", command= lambda: (new_selected_coords.append(mc.get_coords()),
                                                                           update_listbox(window=True,
                                                                             window_listbox=popup_listbox, 
                                                                             coords = new_selected_coords)))
                                                                           
    select_button.grid(row=0, column=0)

    save_selection = ttk.Button(win, text= "Save Positions", command = lambda: (selected_coords.clear(), 
                                                                        listbox.delete(0, tk.END),
                                                                        print(new_selected_coords),
                                                                        update_listbox(window=True, 
                                                                                       window_listbox=popup_listbox,
                                                
                                                                                        coords=new_selected_coords, 
                                                                                        save=True),
                                                                        update_motion(),
                                                                        win.destroy()))
    save_selection.grid(row=0, column=1)

    
    return

def update_motion():
    ax.clear()
    ax.set_xbound(-.4, .4)
    ax.set_ybound(-.4, .4)
    ax.set_zlim(0, .6)
    update_listbox()

    global dual_quaternions
    dual_quaternions = []
    if selected_coords:
        for coord in selected_coords:
            temp_quaternion = quaternionic.array(to_quaternion(rad(coord[3]), rad(coord[4]), rad(coord[5])))
            print(temp_quaternion.tolist())
            temp_quaternion = temp_quaternion.tolist()
            temp_quaternion.extend([c/1000 for c in coord[0:3]])
            temp_dq = DualQuaternion.from_quat_pose_array(temp_quaternion)
            dual_quaternions.append(temp_dq)
            draw_axis(coord,ax,np)
    print(dual_quaternions)
    createPath()

def createPath():
    global path_coords, dual_quaternions, joint_array, passed
    passed=True
    path_coords=[] 
    joint_array=[]
    points=30
    adjusted_control_pos_dq=dual_quaternions
    
    if (len(adjusted_control_pos_dq) <2):
        return
    if selected_type.get()=="Closed":
        adjusted_control_pos_dq.extend(dual_quaternions[0:Spline_degree])
    if not selected_curve:
        for i in range(points+1):
            temp_dq=(i/points)*dual_quaternions[1]+(1-i/points)*dual_quaternions[0]
            temp_matrix=temp_dq.homogeneous_matrix()
            joint_array.append(arm.ets().ik_NR(temp_matrix))
            temp_quat_pose=temp_dq.quat_pose_array()
            #print(quaternionic.array(temp_quat_pose[0:4]))
            temp_pose=[c*1000 for c in get_translation(temp_dq)]
            temp_angles=to_euler_angles(temp_quat_pose[0:4])
            #temp_pose=temp_quat_pose[4:7]
            #temp_angles = [deg(c) for c in temp_angles]
            # temp_angles=[deg(temp_angles[0]-180),-1*(deg(temp_angles[1])),deg(temp_angles[2]+180)]
            temp_pose.extend(temp_angles)
            path_coords.append(temp_pose)
            #robot.plot(joint_array[-1],ax)
            draw_axis(temp_pose,ax,np,full=False)
        print(path_coords)
        print(joint_array)
        #robot.plot(joint_array[0],ax)
    elif selected_curve.get() == "B-spline Motion":
        knot_vector = gen_knot_vector(degree=Spline_degree, n = len(adjusted_control_pos_dq)-1,style=selected_type.get())
        b_spline_dqs = b_spline_curve(knot_vector=knot_vector, 
                                      degree=Spline_degree, 
                                      control_positions=adjusted_control_pos_dq,
                                    resolution=20)
        for i,dq in enumerate(b_spline_dqs):
            temp_quat_pose=dq.quat_pose_array()
            print(quaternionic.array(temp_quat_pose[0:4]))
            temp_matrix=dq.homogeneous_matrix()
            joint_array.append(arm.ets().ik_GN(temp_matrix,ilimit=50,slimit=300))
            if joint_array[-1][1]==0:
                print("here",joint_array[-1])
                if passed:
                    messagebox.showerror(title="Robot Workspace Check", message="Manipulator unable to reach configuration")
                passed=False
            temp_angles=to_euler_angles(temp_quat_pose[0:4])
            temp_pose=[c*1000 for c in get_translation(dq)]
            #temp_angles = [deg(c) for c in temp_angles]
            temp_pose.extend(temp_angles)
            if i==0:
                path_coords.append((temp_pose,0))
            else:
                distance =find_distance(temp_pose,path_coords[-1][0])
                time_to_run=distance/move_speed
                path_coords.append((temp_pose,time_to_run))
            if i%5==0:
                draw_axis(temp_pose,ax,np)
            draw_axis(temp_pose,ax,np,joint_array[-1][1],full=False)
        print(path_coords)
        print(passed)
    
    elif selected_curve.get() == 'B-spline Interpolation':
        parameter=parameterize(dual_quaternions,selected_parameter.get(),selected_distance.get())
        control_points_arr=get_control_points(dual_quaternions,parameter,Spline_degree,control_points)
        for i in control_points_arr:
            temp_quat_pose=i.quat_pose_array()
            temp_angles=to_euler_angles(temp_quat_pose[0:4])
            temp_pose=[c*1000 for c in get_translation(i)]
            temp_pose.extend(temp_angles)
            #draw_axis(temp_pose,ax,np,False,full=False,)
            #draw_axis(temp_pose,ax,np)
        knot_vector = interpolation_knot_vector(len(dual_quaternions)-1,control_points,Spline_degree,parameter)
        print(knot_vector)
        b_spline_dqs = b_spline_curve(knot_vector=knot_vector, 
                                      degree=Spline_degree, 
                                      control_positions=control_points_arr,
                                    resolution=20)
        for i,dq in enumerate(b_spline_dqs):
            temp_quat_pose=dq.quat_pose_array()
            print(quaternionic.array(temp_quat_pose[0:4]))
            temp_matrix=dq.homogeneous_matrix()
            joint_array.append(arm.ets().ik_GN(temp_matrix,ilimit=50,slimit=300))
            if joint_array[-1][1]==0:
                print("here",joint_array[-1])
                if passed:
                    messagebox.showerror(title="Robot Workspace Check", message="Manipulator unable to reach configuration")
                passed=False
            temp_angles=to_euler_angles(temp_quat_pose[0:4])
            temp_pose=[c*1000 for c in get_translation(dq)]
            #temp_angles = [deg(c) for c in temp_angles]
            temp_pose.extend(temp_angles)
            if i==0:
                path_coords.append((temp_pose,0))
            else:
                distance =find_distance(temp_pose,path_coords[-1][0])
                time_to_run=distance/move_speed
                path_coords.append((temp_pose,time_to_run))
            if i%5==0:
                draw_axis(temp_pose,ax,np)
            draw_axis(temp_pose,ax,np,joint_array[-1][1],full=False)

def find_distance(point1,point2):
    return((point1[0]-point2[0])**2+(point1[1]-point2[1])**2+(point1[2]-point2[2])**2)**.5


def run_motion():
    global passed
    if not passed:
        messagebox.showerror(title="Robot Workspace Check", message="Manipulator unable to reach configuration.")
        return

        
    mc.send_coords(path_coords[0][0],speed=move_speed,mode=1)
    while mc.is_moving()==1:
        pass
    for c in path_coords[1:]:
        starttime=time.time()
    #for c in joint_array:
        #mc.send_radians(radians=c[1:7], speed=20)
        #while(time.time()-starttime<c[1]):
        time.sleep(0.01)
        mc.send_coords(c[0],speed=move_speed,mode=1)
        #while mc.is_moving() == 1:
        #    pass
        if mc.is_moving() == -1:
            print(mc.get_error_information())
            return
    return

def update_listbox(window = False, window_listbox = None, coords = None, save =False):
    global listbox
    global selected_coords
    if not window:
        listbox.delete(0, tk.END)
        if selected_coords:
            for i, coord in enumerate(selected_coords):
                listbox.insert(i, str(coord))
    elif window:
        selected_coords = coords.copy()
        window_listbox.delete(0, tk.END)
        if selected_coords:
            for i, coord in enumerate(selected_coords):
                window_listbox.insert(i, str(coord))
                if save:
                    listbox.insert(i,str(coord))


def release_servo():
    mc.release_all_servos()

def checkDegree(value):
    global Spline_degree
    if value.isnumeric() and int(value)>0 and int(value)<len(selected_coords):
        Spline_degree=int(value)
        update_motion()
        return True
    elif value=="":
        return True
    else:
        return False
    
def check_control_pts(value):
    global control_points
    if value.isnumeric() and int(value)>Spline_degree and int(value)<=len(selected_coords):
        control_points=int(value)-1
        update_motion()
        return True
    elif value=="":
        return True
    else:
        return False

def change_curve(var, index, mode):
    print(var, index, mode)
    #print(var.get())
    update_motion()

root = tk.Tk()
root.wm_title("Motion Selection/Visualization Interface")
#mc = MyCobot(PI_PORT, PI_BAUD)

left_frame = tk.Frame(root)
right_frame = tk.Frame(root)

listbox_frame = tk.Frame(left_frame)

label = tk.Label(left_frame, text="Control Positions")
label.pack(padx=10, pady=5)

listbox = tk.Listbox(listbox_frame, height=10, width=50, bg="light grey", activestyle='dotbox', font=("Helvetica", 8))
listbox.pack(side=tk.LEFT, padx=10, pady=10)

# Save Button in the listbox frame
saveButton = tk.Button(listbox_frame, text="Save Control Positions", command=save_to_file)
saveButton.pack(side=tk.LEFT, padx=10, pady=10) 
listbox_frame.pack(pady=5)

input_frame = tk.Frame(left_frame)
curve = [ 
    "B-spline Motion", "B-spline Interpolation"
] 
selected_curve= tk.StringVar() 
selected_curve.set( "B-spline Motion" ) 
selected_curve.trace_add("write",change_curve)

curve_label=tk.Label(input_frame,text="Select curve type")
curve_label.grid(row=0,column=0)
curve_dropdown = tk.OptionMenu(input_frame, selected_curve , *curve ) 
curve_dropdown.grid(row=0,column=1) 

deg_label=tk.Label(input_frame,text="Enter Degree of curve.\n Only numbers less than the number of points is allowed")
deg_label.grid(row=1,column=0)
deg_valid = input_frame.register(checkDegree)
degree_textbox=tk.Entry(input_frame,validate='key', validatecommand=(deg_valid,'%P'))
degree_textbox.grid(row=1,column=1)

types=["Closed","Clamped"]
type_label=tk.Label(input_frame,text="Select Curve Ending Condition")
type_label.grid(row=2,column=0)
selected_type=tk.StringVar()
selected_type.set('Clamped')
selected_type.trace_add("write",change_curve)
type_dropdown=tk.OptionMenu(input_frame,selected_type,*types)
type_dropdown.grid(row=2,column=1)

parameters=["Uniform","Chord","Centripetal"]
parameter_label=tk.Label(input_frame,text="Select Parameterization Type (Interpolation Only)")
parameter_label.grid(row=3,column=0)
selected_parameter=tk.StringVar()
selected_parameter.set("Uniform")
selected_parameter.trace_add("write",change_curve)
parameter_dropdown=tk.OptionMenu(input_frame,selected_parameter,*parameters)
parameter_dropdown.grid(row=3,column=1)

distance=["Quaternion","Cartesian"]
distance_label=tk.Label(input_frame,text="Select which way to find distance (Chord and Centripetal)")
distance_label.grid(row=4,column=0)
selected_distance=tk.StringVar()
selected_distance.set("Quaternion")
selected_distance.trace_add("write",change_curve)
distance_dropdown=tk.OptionMenu(input_frame,selected_distance,*distance)
distance_dropdown.grid(row=4,column=1)

control_pts_label=tk.Label(input_frame,text="Enter Number of Control Points (Interpolation Only).\n Only numbers less than or equal to the number of points is allowed")
control_pts_label.grid(row=5,column=0)
control_pts_valid = input_frame.register(check_control_pts)
control_pts_textbox=tk.Entry(input_frame,validate='key', validatecommand=(control_pts_valid,'%P'))
control_pts_textbox.grid(row=5,column=1)




left_frame.pack(side=tk.LEFT, fill=tk.Y)
input_frame.pack()
saveButton.pack()

#Plot in the right frame
fig = Figure(figsize=(5, 4), dpi=100)
ax = fig.add_subplot(111, projection="3d")
t = np.arange(0, 3, .01)
#ax.plot(t, 2 * np.sin(2 * np.pi * t))
ax.set_xbound(-.4, .4)
ax.set_ybound(-.4, .4)
ax.set_zlim(0, .6)

canvas = FigureCanvasTkAgg(fig, master=right_frame)
canvas.draw()
canvas.get_tk_widget().pack()

toolbar = NavigationToolbar2Tk(canvas, right_frame)
toolbar.update()

right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

#Control buttons
button_frame = tk.Frame(left_frame)
tk.Button(button_frame, text="Browse Files", command=open_file).pack(side=tk.LEFT, padx=10)
tk.Button(button_frame, text= "Collect Control Poses", command = set_coords).pack(side=tk.LEFT, padx=10)
tk.Button(button_frame, text="Run Motion", command=run_motion).pack(side=tk.LEFT, padx=10)
tk.Button(button_frame, text="Release Servos", command=release_servo).pack(side=tk.LEFT, padx=10)
tk.Button(button_frame, text="Reset",command=reset).pack(side=tk.LEFT, padx=10)

button_frame.pack()

root.mainloop()
