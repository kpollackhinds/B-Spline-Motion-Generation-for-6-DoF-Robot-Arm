import tkinter as tk
from tkinter import filedialog
import numpy as np
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk

# Assume `parse_pose` and `run_motion` are defined in helper_functions
from helper_functions import parse_pose, run_motion

def open_file():
    global selected_coords
    file = filedialog.askopenfilename(initialdir="/", title="Select File", filetypes=(("Text files", "*.txt*"), ("all files", "*.*")))
    if file:
        selected_coords = parse_pose(file)
        update_listbox()

def update_listbox():
    listbox.delete(0, tk.END)
    if selected_coords:
        for i, coord in enumerate(selected_coords):
            listbox.insert(i, str(coord))

root = tk.Tk()
root.wm_title("Embedding in Tk")

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
ax.plot(t, 2 * np.sin(2 * np.pi * t))

canvas = FigureCanvasTkAgg(fig, master=right_frame)
canvas.draw()
canvas.get_tk_widget().pack()

toolbar = NavigationToolbar2Tk(canvas, right_frame)
toolbar.update()

right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

# Control buttons
button_frame = tk.Frame(root)
tk.Button(button_frame, text="Browse", command=open_file).pack(side=tk.LEFT, padx=10)
tk.Button(button_frame, text="Run Motion", command=run_motion).pack(side=tk.LEFT, padx=10)
button_frame.pack(fill=tk.X)

root.mainloop()
