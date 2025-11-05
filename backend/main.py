from fastapi import FastAPI
from fastapi.middleware.cors import CORSMiddleware
from pydantic import BaseModel
from datetime import datetime
import uvicorn

app = FastAPI(title="Mobile Robot Backend")

# Allow all origins for development
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # You can restrict this to your frontend URL in production
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

# -------------------------------
# In-memory state
# -------------------------------
robot_state = {
    "linear": 0.0,
    "angular": 0.0,
    "battery": 92,
    "mode": "IDLE"
}

logs = []


# -------------------------------
# Routes
# -------------------------------
@app.get("/")
def root():
    return {"message": "Robot Backend is running 🚀"}


@app.post("/cmd_vel")
def send_cmd_vel(cmd: CmdVel):
    """
    Simulate receiving velocity commands.
    In real setup, send these to ROS2 / Micro-ROS / hardware controller.
    """
    robot_state["linear"] = cmd.linear
    robot_state["angular"] = cmd.angular
    robot_state["mode"] = "MOVING" if cmd.linear != 0 or cmd.angular != 0 else "IDLE"

    # Add to logs
    logs.append({
        "time": datetime.now().strftime("%H:%M:%S"),
        "message": f"CMD_VEL -> linear={cmd.linear:.2f}, angular={cmd.angular:.2f}"
    })

    # Keep log size reasonable
    if len(logs) > 100:
        logs.pop(0)

    return {"status": "ok", "linear": cmd.linear, "angular": cmd.angular}


@app.get("/status")
def get_status():
    """
    Returns current robot state.
    """
    # Example: battery drain simulation
    if robot_state["mode"] == "MOVING":
        robot_state["battery"] = max(0, robot_state["battery"] - 0.01)
    return robot_state


@app.get("/logs")
def get_logs():
    """
    Returns recent logs (latest first)
    """
    return list(reversed(logs[-50:]))


# -------------------------------
# Run the server
# -------------------------------
if __name__ == "__main__":
    uvicorn.run(app, host="0.0.0.0", port=8000)
