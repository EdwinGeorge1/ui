# main.py (updated for camera feed)
from fastapi import FastAPI, WebSocket
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from datetime import datetime
import asyncio
import math
import uvicorn
import base64

# --- ROS 2 imports ---
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped, PointStamped
from nav_msgs.msg import Odometry, Path, OccupancyGrid
from sensor_msgs.msg import LaserScan, CompressedImage
from visualization_msgs.msg import MarkerArray
from tf_transformations import euler_from_quaternion

# -------------------------------
# FastAPI setup
# -------------------------------
app = FastAPI(title="Mobile Robot Backend")
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)

# -------------------------------
# Models
# -------------------------------
class CmdVel(BaseModel):
    linear: float
    angular: float

class Goal(BaseModel):
    x: float
    y: float
    theta: float = 0.0

# -------------------------------
# Robot state
# -------------------------------
robot_state = {
    "linear": 0.0,
    "angular": 0.0,
    "battery": 100.0,
    "mode": "IDLE",
    "pose": {"x": 0.0, "y": 0.0, "theta": 0.0},
    "goal": {"x": 0.0, "y": 0.0, "theta": 0.0},
    "scan_points": [],
    "path": [],
    "smoothed_path": [],
    "waypoints": [],
    "markers": [],
    "local_costmap": None,
    "camera_image": None  # NEW: store latest camera frame
}

logs = []

# -------------------------------
# ROS 2 Node
# -------------------------------
rclpy.init()
ros_node = Node("fastapi_ros_bridge")

# Publishers
cmd_vel_pub = ros_node.create_publisher(Twist, "/cmd_vel", 10)
goal_pub = ros_node.create_publisher(PoseStamped, "/goal_pose", 10)

# -------------------------------
# Subscribers
# -------------------------------
def odom_callback(msg: Odometry):
    robot_state["pose"]["x"] = msg.pose.pose.position.x
    robot_state["pose"]["y"] = msg.pose.pose.position.y
    q = msg.pose.pose.orientation
    _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
    robot_state["pose"]["theta"] = yaw

ros_node.create_subscription(Odometry, "/odom", odom_callback, 10)

def scan_callback(msg: LaserScan):
    points = []
    angle = msg.angle_min
    for r in msg.ranges:
        if not math.isinf(r) and not math.isnan(r):
            x = r * math.cos(angle)
            y = r * math.sin(angle)
            points.append({"x": x, "y": y})
        angle += msg.angle_increment
    robot_state["scan_points"] = points

ros_node.create_subscription(LaserScan, "/scan", scan_callback, 10)

ros_node.create_subscription(Path, "/plan", lambda msg: robot_state.update({
    "path": [{"x": p.pose.position.x, "y": p.pose.position.y} for p in msg.poses]
}), 10)

ros_node.create_subscription(Path, "/plan_smoothed", lambda msg: robot_state.update({
    "smoothed_path": [{"x": p.pose.position.x, "y": p.pose.position.y} for p in msg.poses]
}), 10)

def clicked_point_callback(msg: PointStamped):
    robot_state["waypoints"].append({"x": msg.point.x, "y": msg.point.y})
    if len(robot_state["waypoints"]) > 20:
        robot_state["waypoints"].pop(0)

ros_node.create_subscription(PointStamped, "/clicked_point", clicked_point_callback, 10)

def marker_callback(msg: MarkerArray):
    markers = []
    for m in msg.markers:
        markers.append({
            "position": {"x": m.pose.position.x, "y": m.pose.position.y, "z": m.pose.position.z},
            "color": {"r": m.color.r, "g": m.color.g, "b": m.color.b, "a": m.color.a}
        })
    robot_state["markers"] = markers

ros_node.create_subscription(MarkerArray, "/marker", marker_callback, 10)

def local_costmap_callback(msg: OccupancyGrid):
    robot_state["local_costmap"] = {
        "width": msg.info.width,
        "height": msg.info.height,
        "resolution": msg.info.resolution,
        "origin": {"x": msg.info.origin.position.x, "y": msg.info.origin.position.y},
        "data": list(msg.data)
    }

ros_node.create_subscription(OccupancyGrid, "/local_costmap/costmap", local_costmap_callback, 10)

# --- Camera Subscriber ---
def camera_callback(msg: CompressedImage):
    # Convert ROS CompressedImage to base64 string
    robot_state["camera_image"] = base64.b64encode(msg.data).decode("utf-8")

ros_node.create_subscription(CompressedImage, "/camera/image_raw/compressed", camera_callback, 10)

# -------------------------------
# Background ROS spin
# -------------------------------
async def ros_spin_loop():
    while True:
        rclpy.spin_once(ros_node, timeout_sec=0.1)
        await asyncio.sleep(0.01)

@app.on_event("startup")
async def start_ros_loop():
    asyncio.create_task(ros_spin_loop())
    print("✅ ROS node started and spinning")

# -------------------------------
# API Routes
# -------------------------------
@app.post("/cmd_vel")
def send_cmd_vel(cmd: CmdVel):
    msg = Twist()
    msg.linear.x = cmd.linear
    msg.angular.z = cmd.angular
    cmd_vel_pub.publish(msg)
    robot_state["linear"] = cmd.linear
    robot_state["angular"] = cmd.angular
    robot_state["mode"] = "MOVING" if (cmd.linear or cmd.angular) else "IDLE"

    logs.append({
        "time": datetime.now().strftime("%H:%M:%S"),
        "message": f"CMD_VEL: linear={cmd.linear:.2f}, angular={cmd.angular:.2f}"
    })
    if len(logs) > 100: logs.pop(0)
    return {"status": "ok", "mode": robot_state["mode"]}

@app.get("/status")
def get_status():
    return robot_state

@app.get("/logs")
def get_logs():
    return list(reversed(logs[-50:]))

@app.post("/goal")
def set_goal(goal: Goal):
    robot_state["goal"] = goal.dict()
    msg = PoseStamped()
    msg.header.frame_id = "map"
    msg.pose.position.x = goal.x
    msg.pose.position.y = goal.y
    msg.pose.position.z = 0.0
    msg.pose.orientation.w = 1.0
    goal_pub.publish(msg)
    logs.append({"time": datetime.now().strftime("%H:%M:%S"), "message": f"Goal set to ({goal.x:.2f},{goal.y:.2f})"})
    return {"status": "goal received", "goal": goal}

# -------------------------------
# WebSocket for live updates
# -------------------------------
@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    await ws.accept()
    try:
        while True:
            await ws.send_json({"type": "pose", "pose": robot_state["pose"]})
            await ws.send_json({"type": "scan", "points": robot_state["scan_points"]})
            await ws.send_json({"type": "path", "points": robot_state["path"]})
            await ws.send_json({"type": "smoothed_path", "points": robot_state["smoothed_path"]})
            await ws.send_json({"type": "waypoints", "points": robot_state["waypoints"]})
            await ws.send_json({"type": "markers", "markers": robot_state["markers"]})
            await ws.send_json({"type": "local_costmap", "data": robot_state["local_costmap"]})
            if robot_state["camera_image"]:
                await ws.send_json({"type": "camera", "image": robot_state["camera_image"]})
            await asyncio.sleep(0.1)
    except Exception as e:
        print("WebSocket disconnected:", e)

# -------------------------------
# Run FastAPI
# -------------------------------
if __name__ == "__main__":
    uvicorn.run("main:app", host="0.0.0.0", port=8000, reload=True)
