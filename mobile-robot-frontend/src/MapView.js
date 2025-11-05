// MapView.js
import React, { useRef, useEffect } from "react";
import ROSLIB from "roslib";

// ROS connection
const ros = new ROSLIB.Ros({ url: "ws://localhost:9090" });
const goalTopic = new ROSLIB.Topic({
  ros,
  name: "/goal_pose",
  messageType: "geometry_msgs/PoseStamped",
});

export default function MapView() {
  const canvasRef = useRef(null);
  const rafRef = useRef(null);
  const mapImageRef = useRef(new Image());
  const robotPoseRef = useRef({ x: 0, y: 0, theta: 0 });
  const pointsRef = useRef([]);

  const MAP_IMG = "/bot_cafe.png";
  const MAP_RESOLUTION = 0.05;
  const MAP_ORIGIN = [-9.89, -6.09];

  useEffect(() => {
    const mapImage = mapImageRef.current;
    mapImage.src = MAP_IMG;
    mapImage.onload = () => {
      const canvas = canvasRef.current;
      if (canvas) {
        canvas.width = mapImage.width;
        canvas.height = mapImage.height;
      }
      drawLoop();
    };
  }, []);

  useEffect(() => {
    const ws = new WebSocket("ws://127.0.0.1:8000/ws");
    ws.onmessage = (ev) => {
      try {
        const data = JSON.parse(ev.data);
        if (data.type === "pose") robotPoseRef.current = data.pose;
        if (data.type === "scan") pointsRef.current = data.points;
      } catch (e) {
        console.error("WS parse error", e);
      }
    };
    return () => ws.close();
  }, []);

  const worldToCanvas = (x, y) => {
    const canvas = canvasRef.current;
    if (!canvas) return [0, 0];
    const cx = (x - MAP_ORIGIN[0]) / MAP_RESOLUTION;
    const cy = canvas.height - (y - MAP_ORIGIN[1]) / MAP_RESOLUTION;
    return [cx, cy];
  };

  const drawLoop = () => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext("2d");
    const mapImage = mapImageRef.current;

    const draw = () => {
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      ctx.drawImage(mapImage, 0, 0);

      const { x: rx, y: ry, theta } = robotPoseRef.current;

      ctx.fillStyle = "lime";
      pointsRef.current.forEach((p) => {
        const mapX = rx + p.x * Math.cos(theta) - p.y * Math.sin(theta);
        const mapY = ry + p.x * Math.sin(theta) + p.y * Math.cos(theta);
        const [cx, cy] = worldToCanvas(mapX, mapY);
        ctx.beginPath();
        ctx.arc(cx, cy, 2, 0, 2 * Math.PI);
        ctx.fill();
      });

      const [robotCx, robotCy] = worldToCanvas(rx, ry);
      ctx.save();
      ctx.translate(robotCx, robotCy);
      ctx.rotate(-theta);
      ctx.fillStyle = "red";
      ctx.beginPath();
      ctx.moveTo(10, 0);
      ctx.lineTo(-6, 6);
      ctx.lineTo(-6, -6);
      ctx.closePath();
      ctx.fill();
      ctx.restore();

      rafRef.current = requestAnimationFrame(draw);
    };

    rafRef.current = requestAnimationFrame(draw);
  };

  const handleClick = (e) => {
    const canvas = canvasRef.current;
    const rect = canvas.getBoundingClientRect();
    const x = (e.clientX - rect.left) * MAP_RESOLUTION + MAP_ORIGIN[0];
    const y = ((rect.height - (e.clientY - rect.top)) * MAP_RESOLUTION) + MAP_ORIGIN[1];

    const goalMsg = new ROSLIB.Message({
      header: { frame_id: "map" },
      pose: { position: { x, y, z: 0 }, orientation: { x: 0, y: 0, z: 0, w: 1 } },
    });
    goalTopic.publish(goalMsg);
  };

  return (
    <div className="page">
      <h1 className="title">Map View</h1>
      <canvas
        ref={canvasRef}
        onClick={handleClick}
        style={{ border: "1px solid #555", display: "block" }}
      />
      <p className="small">
        Pose: ({robotPoseRef.current.x.toFixed(2)}, {robotPoseRef.current.y.toFixed(2)})
      </p>
    </div>
  );
}
