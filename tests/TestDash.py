import tkinter as tk
from tkinter import ttk
from tkinter.font import Font
from PIL import Image, ImageTk


window = tk.Tk()
window.title = "GeoScenario Simulation"
scenario_name = "LaneChange"
scenario_file = "lanechange1.osm"
n_vehicles = 4
time = 0 
frame = 0 
frametime = 0
drift = 0
#Images
img_wise = ImageTk.PhotoImage(Image.open("dash/img/wiselogo.png").resize((83, 40)))
img_uw = ImageTk.PhotoImage(Image.open("dash/img/uwlogo.png").resize((100, 40)))
img_gs = pimg = ImageTk.PhotoImage(Image.open("dash/img/icons/gs.png").resize((40, 40)))
img_veh = ImageTk.PhotoImage(Image.open("dash/img/icons/vehicle.png").resize((100, 47)))
img_ego = ImageTk.PhotoImage(Image.open("dash/img/icons/vehicle_ego.png").resize((80, 30)))
#Text
strgs = ' GeoScenario Vehicle Behavior Models '


#---Build layout
#-Header/Title
title_frame = tk.Frame(window, width = 1000, height = 40, bg = "orange")
title_frame.pack_propagate(False)
#title_frame.grid_propagate(False)
title_frame.pack()
tk.Label(title_frame, image = img_uw).pack(side = "right")
tk.Label(title_frame, image = img_wise).pack(side = "right")
#tk.Label(title_frame, image = img_gs).pack(side = "left")
lb = tk.Label(title_frame, text=strgs, bg = "orange")
lb.configure(font=Font(family="OpenSans", size=24))
#lb.grid(row=1, column=2)
lb.pack(side = 'left')

#-Global chart
gchart_frame = tk.Frame(window, width = 1000, height = 300, bg = "grey91")
gchart_frame.pack_propagate(False)
gchart_frame.pack()

#-Sim Stat
stat_frame = tk.Frame(window, width = 1000, height = 40, bg = "white")
stat_frame.pack()
strstat = "Scenario: {} ({})          | Time {}s Frame# {} FrameTime: {}s Drift: {}s".format(scenario_name, scenario_file, time, frame, frametime, drift)
lb = tk.Label(stat_frame, text=strstat, bg = "white")
lb.configure(font=Font(family="OpenSans", size=16))
#lb.grid(row=1, column=2)
lb.pack(side = 'left')

#Vehicles
vehicles_frame = tk.Frame(window, width = 1000, height = 800, bg = "white")
vehicles_frame.pack()
for v in range(n_vehicles):
    vid = 'vehicle1'
    type = 'SV'
    ego = False
    v_frame = tk.Frame(vehicles_frame, width = 1000, height = 200, bg = "white")
    v_frame.pack_propagate(False)
    v_frame.pack(fill='x')
    if ego:
        tk.Label(v_frame, image = img_ego).pack(side = "left")
    else:
        tk.Label(v_frame, image = img_veh).pack(side = "left")
    label = tk.Label(v_frame, text=vid, borderwidth=1)
    label.pack(side='left')
    


#tk.Label(sim_stat_frame, image = img_veh).pack(side = "left")
#tk.Label(sim_stat_frame, image = img_ego).pack(side = "left")
#imglb = tk.Label(sim_stat_frame, image = pimg)
#imglb.pack(side = "left", fill = "both", expand = "no")
#imglb.pack(side = "left")
#imglb.grid(row=1, column=1)


window.mainloop()

#bottom_frame = tkinter.Frame(window).pack(side = "bottom")



#btn1 = tkinter.Button(top_frame, text = "Button1", fg = "red").pack() #'fg or foreground' is for coloring the contents (buttons)
#btn2 = tkinter.Button(top_frame, text = "Button2", fg = "green").pack()
#btn3 = tkinter.Button(bottom_frame, text = "Button3", fg = "purple").pack(side = "left") #'side' is used to left or right align the widgets
#sbtn4 = tkinter.Button(bottom_frame, text = "Button4", fg = "orange").pack(side = "left")


height = 5
width = 5


#for i in range(height): #Rows
#   for j in range(width): #Columns
#        b = tkinter.Label(V1_frame, text=" test", borderwidth=1, width=10)
#        b.grid(row=i, column=j, padx=1, pady=1)
#tree = ttk.Treeview(V1_frame, columns=('Position', 'Name', 'Score'), show='headings')

window.mainloop()



""" 
class ExampleApp(tk.Tk):
    def __init__(self):
        tk.Tk.__init__(self)
        t = SimpleTable(self, 10,2)
        t.pack(side="top", fill="x")
        t.set(0,0,"Hello, world")

class SimpleTable(tk.Frame):
    def __init__(self, parent, rows=10, columns=2):
        # use black background so it "peeks through" to 
        # form grid lines
        tk.Frame.__init__(self, parent, background="black")
        self._widgets = []
        for row in range(rows):
            current_row = []
            for column in range(columns):
                label = tk.Label(self, text="%s/%s" % (row, column), 
                                 borderwidth=0, width=10)
                label.grid(row=row, column=column, sticky="nsew", padx=1, pady=1)
                current_row.append(label)
            self._widgets.append(current_row)

        for column in range(columns):
            self.grid_columnconfigure(column, weight=1)


    def set(self, row, column, value):
        widget = self._widgets[row][column]
        widget.configure(text=value)

if __name__ == "__main__":
    app = ExampleApp()
    app.mainloop()
 """
