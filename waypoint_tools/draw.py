import tkinter as tk
from PIL import Image, ImageTk

import os, sys
dir_path = os.path.dirname(os.path.realpath(__file__))
parent_dir_path = os.path.abspath(os.path.join(dir_path, os.pardir))

sys.path.insert(0, parent_dir_path)
#This looks dumb, fix it someday
from routing_agent.routing_agent.WaypointGraph import WaypointGraph


def calculateDistance(from_location, to_location):
    return (to_location[0] - from_location[0]) ** 2 + (to_location[1] - from_location[1]) ** 2


class DrawingApp:
    threshold_of_nearby_dots_from_center = 20
    hoveringItem=None
    clickOnItem=None
    tempLine=None
    nodes={0:[0]}
    currentMapId=0
    modevalue=0
    
    w=WaypointGraph()
    print(w.convertToJSON())
    def __init__(self, root):
        self.root = root
        self.root.title("Drawing Lines, Dots, and Text with Button")

        # Load the background image
        self.bg_image = Image.open("lab.png")  # Replace with your image file
        self.bg_image = self.bg_image.resize((int(self.bg_image.size[0]/1.4), int(self.bg_image.size[1]/1.4)))  # Adjust to the window size
        self.bg_photo = ImageTk.PhotoImage(self.bg_image)

        # Set up the canvas for drawing
        print(self.bg_image.size)
        self.canvas = tk.Canvas(self.root, width=int(self.bg_image.size[0]/1.01), height=int(self.bg_image.size[1]/1.01), bg="white")
        self.canvas.pack()
        self.canvas.create_image(0, 0, anchor=tk.NW, image=self.bg_photo)

        # Variables to hold previous mouse coordinates for line drawing
        self.prev_x = None
        self.prev_y = None

        # List to keep track of the dots drawn and their associated IDs and texts
        self.dots = []
        self.dot_labels = []

        # Add a button to clear the canvas
        self.clear_button = tk.Button(self.root, text="Clear Canvas", command=self.clear_canvas)
        self.clear_button.pack()

        self.add_map_id=tk.Button(self.root, text="Add MapId", command=self.addMapId)
        self.add_map_id.pack()

        self.decrease_map_id = tk.Button(self.root, text="Decrease MapId", command=self.decreaseMapId)
        self.decrease_map_id.pack()



        # zoom
        self.zoom_factor = 1.0  # Initial zoom level

        # Bind mouse events
        self.canvas.bind("<Button-1>", self.on_click)  # Left click for dots
        self.canvas.bind("<B1-Motion>", self.on_drag)  # Hold left click to draw lines
        self.canvas.bind("<ButtonRelease-1>", self.stop_drag)
        self.canvas.bind("<Motion>", self.on_hover)  # Hover motion event
        self.root.bind("<MouseWheel>", self.zoom)

    def scaleImage(self,image):
        """Scale the image to the current zoom factor and return a PhotoImage."""
        new_width = int(self.img_width * self.zoom_factor)
        new_height = int(self.img_height * self.zoom_factor)
        scaled_image = self.image.resize((new_width, new_height), Image.ANTIALIAS)
        return ImageTk.PhotoImage(scaled_image)

    def createDots(self,x,y):
        too_close=False
        for i, dot in enumerate(self.dots):
            # Get the coordinates of each dot
            dot_x,dot_y=self.getCoordsOfDot(dot)

            # Check if the mouse is near the dot
            if calculateDistance([x, y], [dot_x,dot_y]) <= self.threshold_of_nearby_dots_from_center:
                too_close=True
        if(not too_close):
            dot = self.canvas.create_oval(x - 3, y - 3, x + 3, y + 3, fill="black")
            # Create text next to the dot
            print(self.currentMapId)
            mapId = str(self.currentMapId).zfill(3)
            index=str(self.getLabelName()).zfill(3)
            label = self.canvas.create_text(x, y-10, text=f"(ID:{mapId}_{index},{x}, {y})", anchor=tk.W)

            # Store dot and label ID to reference them later
            self.dots.append(dot)
            self.dot_labels.append(label)
            return dot
    def on_click(self, event):
        """Draw a dot and display text next to it."""
        x, y = event.x, event.y

        if(self.hoveringItem!=None):
            self.clickOnItem=self.hoveringItem

        if(self.clickOnItem!=None and self.hoveringItem==None):
            self.clickOnItem = None

        if(self.hoveringItem==None):
            self.createDots(x,y)
    def stop_drag(self,event):
        x, y = event.x, event.y
        self.createDots(x, y)
        prevX, prevY = self.getCoordsOfDot(self.clickOnItem)
        self.canvas.create_line(prevX, prevY, x, y, fill="black")
        self.hoveringItem == None
        self.clickOnItem = None

    def on_drag(self, event):
        """Draw a line while dragging the mouse."""
        if self.clickOnItem:
            x, y = event.x, event.y
            prevX,prevY=self.getCoordsOfDot(self.clickOnItem)
            if(self.tempLine==None):
                self.tempLine=self.canvas.create_line(prevX, prevY, x, y, fill="black")
            else:
                self.canvas.coords(self.tempLine, prevX, prevY, event.x, event.y)

    def getCoordsOfDot(self,dot):
        dot_coords = self.canvas.coords(dot)
        dot_x = (dot_coords[0] + dot_coords[2]) / 2  # X center of the dot
        dot_y = (dot_coords[1] + dot_coords[3]) / 2  # Y center of the dot
        return dot_x,dot_y

    def on_hover(self, event):
        """Highlight the dot if the mouse hovers over it."""
        x, y = event.x, event.y
        for i, dot in enumerate(self.dots):
            # Get the coordinates of each dot
            dot_x,dot_y=self.getCoordsOfDot(dot)

            # Check if the mouse is near the dot
            if calculateDistance([x, y], [dot_x,dot_y]) <= self.threshold_of_nearby_dots_from_center:  # 20 is the distance threshold for hovering
                self.canvas.itemconfig(dot, fill="red")  # Highlight dot with red
                self.canvas.itemconfig(self.dot_labels[i], fill="red")  # Change label color to red
                self.hoveringItem=dot
            else:
                self.canvas.itemconfig(dot, fill="black")  # Restore original color
                self.canvas.itemconfig(self.dot_labels[i], fill="black")  # Restore label color

                if (self.hoveringItem==dot):
                    self.hoveringItem=None

        return


    def getLabelName(self):
        if(self.currentMapId not in self.nodes.keys()):
            self.nodes[self.currentMapId]=[0]
        else:
            self.nodes[self.currentMapId].append(self.nodes[self.currentMapId][len(self.nodes[self.currentMapId])-1]+1)

        return self.nodes[self.currentMapId][len(self.nodes[self.currentMapId])-1]
    def clear_canvas(self):
        """Clear the canvas, removing all dots, lines, and text."""
        for dot in self.dots:
            self.canvas.delete(dot)
        for label in self.dot_labels:
            self.canvas.delete(label)
        self.dots = []
        self.dot_labels = []

    def reset_prev_coordinates(self):
        """Reset previous coordinates for line drawing."""
        self.prev_x = None
        self.prev_y = None



    def changemode(self,modevalue):
        self.modevalue=modevalue

    def export_as_CSL_format(self):
        return


    def addMapId(self):
        self.currentMapId+=1
    def decreaseMapId(self):
        self.currentMapId-=1


    def zoom(self, event):

        """Zoom in or out based on mouse wheel direction."""
        zoom_increment = 0.1  # Change in zoom per scroll step

        # Adjust the zoom factor
        if event.delta > 0:  # Scroll up to zoom in
            self.zoom_factor += zoom_increment
        elif event.delta < 0:  # Scroll down to zoom out
            self.zoom_factor = max(self.zoom_factor - zoom_increment, 0.1)  # Prevent zero/negative zoom

        # Update the scaled image
        self.display_image = self.get_scaled_image()
        self.canvas.itemconfig(self.canvas_image, image=self.display_image)

        # Center the image (optional)
        self.canvas.config(scrollregion=self.canvas.bbox(tk.ALL))



if __name__ == "__main__":
    # Create the main window
    root = tk.Tk()

    # Initialize the drawing application
    app = DrawingApp(root)

    # Start the GUI event loop
    root.mainloop()
