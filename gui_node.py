#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import tkinter as tk
import threading
import math

# --- CONSTANTS ---
CELL_SIZE = 24
GRID_W = 28
GRID_H = 18
W = GRID_W * CELL_SIZE
H = GRID_H * CELL_SIZE
MAP_RESOLUTION = 1.0

EMPTY = 0
WALL = 1

# --- SHARED STATE ---
lock = threading.Lock()
grid = [[EMPTY for _ in range(GRID_W)] for _ in range(GRID_H)]
start_pose = None
goal_pose = None
mouse_down = False
placing_wall = True
place_mode = None
drag_anchor_xy = None

def world_to_canvas(x, y):
    return x * CELL_SIZE, (GRID_H - y) * CELL_SIZE

def canvas_to_world(px, py):
    return px / CELL_SIZE, GRID_H - (py / CELL_SIZE)

# --- ROS 2 NODE ---
class GuiNode(Node):
    def __init__(self):
        super().__init__('tkinter_map_editor')
        self.grid_pub = self.create_publisher(OccupancyGrid, 'astar_grid', 10)
        self.start_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
        self.timer = self.create_timer(0.5, self.publish_grid) # Publish grid at 2Hz

    def publish_grid(self):
        with lock:
            local_grid = [row[:] for row in grid]
            
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.info.resolution = MAP_RESOLUTION
        msg.info.width = GRID_W
        msg.info.height = GRID_H
        
        data = []
        for r in reversed(range(GRID_H)):
            for c in range(GRID_W):
                data.append(100 if local_grid[r][c] == WALL else 0)
        msg.data = data
        self.grid_pub.publish(msg)

    def publish_start(self, x, y, yaw):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = float(x)
        msg.pose.pose.position.y = float(y)
        msg.pose.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.start_pub.publish(msg)

    def publish_goal(self, x, y, yaw):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        self.goal_pub.publish(msg)

# --- TKINTER GUI ---
root = tk.Tk()
root.title("Hybrid A* C++ Frontend")
canvas = tk.Canvas(root, width=W, height=H, bg="#1E1E1E")
canvas.pack()

ros_node = None

def draw():
    canvas.delete("all")
    with lock:
        for r in range(GRID_H):
            for c in range(GRID_W):
                x1, y1 = c * CELL_SIZE, r * CELL_SIZE
                color = "#FF9900" if grid[r][c] == WALL else "#2D2D2D"
                canvas.create_rectangle(x1, y1, x1+CELL_SIZE, y1+CELL_SIZE, fill=color, outline="#333")
        
        for pose, color, label in [(start_pose, "#00D1FF", "START"), (goal_pose, "#00FF66", "GOAL")]:
            if pose:
                px, py = world_to_canvas(pose[0], pose[1])
                canvas.create_oval(px-6, py-6, px+6, py+6, fill=color, outline="")
                canvas.create_text(px+10, py-10, text=label, fill=color, anchor="nw")
                canvas.create_line(px, py, px + 26*math.cos(-pose[2]), py + 26*math.sin(-pose[2]), fill=color, width=2, arrow=tk.LAST)
                
        if place_mode:
            canvas.create_text(8, 8, text=f"MODE: place {place_mode.upper()} (click+drag yaw)", fill="#FFD24D", anchor="nw")

def on_mouse_down(event):
    global mouse_down, placing_wall, drag_anchor_xy, place_mode, start_pose, goal_pose
    xw, yw = canvas_to_world(event.x, event.y)
    
    if place_mode in ("start", "goal"):
        drag_anchor_xy = (xw, yw)
        if place_mode == "start": start_pose = (xw, yw, 0.0)
        else: goal_pose = (xw, yw, 0.0)
        draw()
        return

    mouse_down = True
    r, c = event.y // CELL_SIZE, event.x // CELL_SIZE
    if 0 <= r < GRID_H and 0 <= c < GRID_W:
        with lock:
            placing_wall = (grid[r][c] == EMPTY)
            grid[r][c] = WALL if placing_wall else EMPTY
        draw()

def on_mouse_move(event):
    global start_pose, goal_pose
    if drag_anchor_xy and place_mode:
        xw, yw = canvas_to_world(event.x, event.y)
        yaw = math.atan2(yw - drag_anchor_xy[1], xw - drag_anchor_xy[0])
        if place_mode == "start": start_pose = (drag_anchor_xy[0], drag_anchor_xy[1], yaw)
        else: goal_pose = (drag_anchor_xy[0], drag_anchor_xy[1], yaw)
        draw()
        return

    if mouse_down:
        r, c = event.y // CELL_SIZE, event.x // CELL_SIZE
        if 0 <= r < GRID_H and 0 <= c < GRID_W:
            with lock: grid[r][c] = WALL if placing_wall else EMPTY
            draw()

def on_mouse_up(event):
    global mouse_down, drag_anchor_xy, place_mode
    if place_mode == "start" and start_pose and ros_node:
        ros_node.publish_start(*start_pose)
    elif place_mode == "goal" and goal_pose and ros_node:
        ros_node.publish_goal(*goal_pose)
    
    mouse_down = False
    drag_anchor_xy = None
    place_mode = None
    draw()

def on_key(event):
    global place_mode
    key = event.keysym.lower()
    if key == "s": place_mode = "start"
    elif key == "g": place_mode = "goal"
    elif key == "c": 
        with lock:
            for r in range(GRID_H):
                for c in range(GRID_W): grid[r][c] = EMPTY
    draw()

canvas.bind("<Button-1>", on_mouse_down)
canvas.bind("<B1-Motion>", on_mouse_move)
canvas.bind("<ButtonRelease-1>", on_mouse_up)
root.bind("<Key>", on_key)
draw()

def main():
    global ros_node
    rclpy.init()
    ros_node = GuiNode()
    ros_thread = threading.Thread(target=rclpy.spin, args=(ros_node,), daemon=True)
    ros_thread.start()
    root.mainloop()
    rclpy.shutdown()

if __name__ == "__main__":
    main()