from fastapi import FastAPI, WebSocket
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from datetime import datetime
import uvicorn
import asyncio
import math

# --- ROS 2 Imports ---
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf_transformations import euler_from_quaternion

# -------------------------------
# FastAPI Setup
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
# Data Models
# -------------------------------
class CmdVel(BaseModel):
    linear: float
    angular: float

class Goal(BaseModel):
    x: float
    y: float
    theta: float = 0.0

# -------------------------------
# Robot State
# -------------------------------
robot_state = {
    "linear": 0.0,
    "angular": 0.0,
    "battery": 100.0,
    "mode": "IDLE",
    "pose": {"x": 0.0, "y": 0.0, "theta": 0.0},
    "goal": {"x": 0.0, "y": 0.0, "theta": 0.0},
    "scan_points": []
}

logs = []

# -------------------------------
# ROS 2 Node Setup
# -------------------------------
rclpy.init()
ros_node = Node("fastapi_ros_bridge")

# Publisher to /cmd_vel
cmd_vel_pub = ros_node.create_publisher(Twist, "/cmd_vel", 10)

# Publisher for goal (optional)
goal_pub = ros_node.create_publisher(PoseStamped, "/goal_pose", 10)

# Subscriber to /odom
def odom_callback(msg: Odometry):
    robot_state["pose"]["x"] = msg.pose.pose.position.x
    robot_state["pose"]["y"] = msg.pose.pose.position.y
    q = msg.pose.pose.orientation
    _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
    robot_state["pose"]["theta"] = yaw

odom_sub = ros_node.create_subscription(Odometry, "/odom", odom_callback, 10)

# Subscriber to /scan (LaserScan)
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

scan_sub = ros_node.create_subscription(LaserScan, "/scan", scan_callback, 10)

# Background ROS spin
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
    if len(logs) > 100:
        logs.pop(0)

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
    msg.pose.orientation.x = 0.0
    msg.pose.orientation.y = 0.0
    msg.pose.orientation.z = 0.0
    msg.pose.orientation.w = 1.0
    goal_pub.publish(msg)

    logs.append({
        "time": datetime.now().strftime("%H:%M:%S"),
        "message": f"Goal set to ({goal.x:.2f}, {goal.y:.2f})"
    })
    return {"status": "goal received", "goal": goal}

# -------------------------------
# WebSocket for live pose + LIDAR
# -------------------------------
@app.websocket("/ws")
async def websocket_endpoint(ws: WebSocket):
    await ws.accept()
    try:
        while True:
            # Send pose
            await ws.send_json({"type": "pose", "pose": robot_state["pose"]})

            # Send laser scan points
            if robot_state["scan_points"]:
                await ws.send_json({"type": "scan", "points": robot_state["scan_points"]})
            else:
                # fallback simulated LIDAR circle if /scan not yet received
                points = [
                    {"x": math.cos(a) * 1.5, "y": math.sin(a) * 1.5}
                    for a in [i * math.pi / 36 for i in range(72)]
                ]
                await ws.send_json({"type": "scan", "points": points})

            await asyncio.sleep(0.1)
    except Exception as e:
        print("WebSocket disconnected", e)

# -------------------------------
# Run FastAPI
# -------------------------------
if __name__ == "__main__":
    uvicorn.run("main:app", host="0.0.0.0", port=8000, reload=True)
