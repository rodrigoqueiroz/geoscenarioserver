import tkinter

root = tkinter.Tk()

height = 5
width = 5
cells = {}
for i in range(height): #Rows
    for j in range(width): #Columns
        b = tkinter.Entry(root, text="")
        b.grid(row=i, column=j)
        cells[(i,j)] = b


root.mainloop()


