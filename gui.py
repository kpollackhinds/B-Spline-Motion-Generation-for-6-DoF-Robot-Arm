import tkinter as tk
from tkinter import ttk, filedialog
from tkinter.filedialog import askopenfile

import numpy as np
from mpl_toolkits.mplot3d import Axes3D
from matplotlib.backends.backend_tkagg import (
                                    FigureCanvasTkAgg, NavigationToolbar2Tk)
from matplotlib.figure import Figure
from helper_functions import *

def open_file():
   file = filedialog.askopenfile(mode='r')
   if file:
      content = file.read()
      file.close()
      print("%d characters in this file" % len(content))


root = tk.Tk()
root.wm_title("Embedding in Tk")

fig = Figure(figsize=(5, 4), dpi=100)

canvas = FigureCanvasTkAgg(fig, master=root)  # A tk.DrawingArea.
canvas.draw()

button = tk.Button(master= root, text="Run Motion", command=lambda: run_motion())
# Add a Label widget
label = tk.Label(root, text="Click the Button to browse the Files", font=('Georgia 13'))
label.pack(pady=5)
tk.Button(root, text="Browse", command=open_file).pack(pady=10)
ax = fig.add_subplot(111, projection="3d")
t = np.arange(0, 3, .01)
ax.plot(t, 2 * np.sin(2 * np.pi * t))

toolbar = NavigationToolbar2Tk(canvas, root)
toolbar.update()
canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=1)

button.pack()

tk.mainloop()