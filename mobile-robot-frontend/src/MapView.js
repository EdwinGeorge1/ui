// MapView.js
import React, { useRef, useEffect } from "react";
import ROSLIB from "roslib";

export default function MapView() {
  const canvasRef = useRef(null);
  const rafRef = useRef(null);

  // Data refs
  const robotPoseRef = useRef({ x: 0, y: 0, theta: 0 });
  const scanPointsRef = useRef([]);
  const pathRef = useRef([]);
  const smoothedPathRef = useRef([]);
  const waypointsRef = useRef([]);
  const markersRef = useRef([]);
  const localCostmapRef = useRef(null);

  const goalRef = useRef(null);
  const draggingRef = useRef(false);

  const MAP_IMG = "/bot_cafe.png";
  const MAP_RESOLUTION = 0.05;
  const MAP_ORIGIN = [-9.89, -6.09];
  const mapImageRef = useRef(new Image());

  // WebSocket connection
  useEffect(() => {
    const ws = new WebSocket("ws://127.0.0.1:8000/ws");
    ws.onmessage = (ev) => {
      try {
        const data = JSON.parse(ev.data);
        switch (data.type) {
          case "pose": robotPoseRef.current = data.pose; break;
          case "scan": scanPointsRef.current = data.points; break;
          case "path": pathRef.current = data.points; break;
          case "smoothed_path": smoothedPathRef.current = data.points; break;
          case "waypoints": waypointsRef.current = data.points; break;
          case "markers": markersRef.current = data.markers; break;
          case "local_costmap": localCostmapRef.current = data.data; break;
          default: break;
        }
      } catch (e) {
        console.error("WS parse error", e);
      }
    };
    return () => ws.close();
  }, []);

  // Map image
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

  // Convert world ↔ canvas
  const worldToCanvas = (x, y) => {
    const canvas = canvasRef.current;
    if (!canvas) return [0, 0];
    const cx = (x - MAP_ORIGIN[0]) / MAP_RESOLUTION;
    const cy = canvas.height - (y - MAP_ORIGIN[1]) / MAP_RESOLUTION;
    return [cx, cy];
  };
  const canvasToWorld = (cx, cy) => {
    const canvas = canvasRef.current;
    if (!canvas) return [0, 0];
    const x = cx * MAP_RESOLUTION + MAP_ORIGIN[0];
    const y = (canvas.height - cy) * MAP_RESOLUTION + MAP_ORIGIN[1];
    return [x, y];
  };

  // --- Mouse Handlers for Goal ---
  const handleMouseDown = (e) => {
    const canvas = canvasRef.current;
    const rect = canvas.getBoundingClientRect();
    const [x, y] = canvasToWorld(e.clientX - rect.left, e.clientY - rect.top);
    goalRef.current = { x, y, theta: 0 };
    draggingRef.current = true;
  };

  const handleMouseMove = (e) => {
    if (!draggingRef.current || !goalRef.current) return;
    const canvas = canvasRef.current;
    const rect = canvas.getBoundingClientRect();
    const [x, y] = canvasToWorld(e.clientX - rect.left, e.clientY - rect.top);
    const dx = x - goalRef.current.x;
    const dy = y - goalRef.current.y;
    goalRef.current.theta = Math.atan2(dy, dx);
  };

  const handleMouseUp = (e) => {
    if (!goalRef.current) return;
    draggingRef.current = false;

    // Send goal to ROS
    const ros = new ROSLIB.Ros({ url: "ws://localhost:9090" });
    const goalTopic = new ROSLIB.Topic({
      ros,
      name: "/goal_pose",
      messageType: "geometry_msgs/PoseStamped",
    });

    const { x, y, theta } = goalRef.current;
    const qz = Math.sin(theta / 2);
    const qw = Math.cos(theta / 2);

    const goalMsg = new ROSLIB.Message({
      header: { frame_id: "map" },
      pose: { position: { x, y, z: 0 }, orientation: { x: 0, y: 0, z: qz, w: qw } },
    });
    goalTopic.publish(goalMsg);
    console.log("Goal published:", goalRef.current);
  };

  // Draw everything
  const drawLoop = () => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const ctx = canvas.getContext("2d");
    const mapImage = mapImageRef.current;

    const draw = () => {
      ctx.clearRect(0, 0, canvas.width, canvas.height);
      ctx.drawImage(mapImage, 0, 0);

      // Draw costmap
      const costmap = localCostmapRef.current;
      if (costmap) {
        ctx.fillStyle = "rgba(100,100,100,0.5)";
        const { width, height, data, resolution, origin } = costmap;
        for (let y = 0; y < height; y++)
          for (let x = 0; x < width; x++)
            if (data[y*width + x] > 50) {
              const wx = origin.x + x * resolution;
              const wy = origin.y + y * resolution;
              const [cx, cy] = worldToCanvas(wx, wy);
              ctx.fillRect(cx, cy, 2, 2);
            }
      }

      // Draw paths
      const drawPath = (points, color) => {
        if (!points || points.length < 2) return;
        ctx.strokeStyle = color;
        ctx.lineWidth = 2;
        ctx.beginPath();
        points.forEach((p, i) => {
          const [cx, cy] = worldToCanvas(p.x, p.y);
          if (i===0) ctx.moveTo(cx, cy);
          else ctx.lineTo(cx, cy);
        });
        ctx.stroke();
      };
      drawPath(pathRef.current, "blue");
      drawPath(smoothedPathRef.current, "cyan");

      // Draw waypoints
      ctx.fillStyle = "orange";
      waypointsRef.current.forEach((wp) => {
        const [cx, cy] = worldToCanvas(wp.x, wp.y);
        ctx.beginPath();
        ctx.arc(cx, cy, 5, 0, 2*Math.PI);
        ctx.fill();
      });

      // Draw markers
      markersRef.current.forEach((m) => {
        const [cx, cy] = worldToCanvas(m.position.x, m.position.y);
        ctx.fillStyle = `rgba(${m.color.r*255},${m.color.g*255},${m.color.b*255},${m.color.a})`;
        ctx.beginPath();
        ctx.arc(cx, cy, 5, 0, 2*Math.PI);
        ctx.fill();
      });

      // Draw LIDAR
      ctx.fillStyle = "lime";
      scanPointsRef.current.forEach((p) => {
        const rx = robotPoseRef.current.x;
        const ry = robotPoseRef.current.y;
        const theta = robotPoseRef.current.theta;
        const mapX = rx + p.x*Math.cos(theta) - p.y*Math.sin(theta);
        const mapY = ry + p.x*Math.sin(theta) + p.y*Math.cos(theta);
        const [cx, cy] = worldToCanvas(mapX, mapY);
        ctx.beginPath();
        ctx.arc(cx, cy, 2, 0, 2*Math.PI);
        ctx.fill();
      });

      // Draw robot
      const [robotCx, robotCy] = worldToCanvas(robotPoseRef.current.x, robotPoseRef.current.y);
      ctx.save();
      ctx.translate(robotCx, robotCy);
      ctx.rotate(-robotPoseRef.current.theta);
      ctx.fillStyle = "red";
      ctx.beginPath();
      ctx.moveTo(10,0);
      ctx.lineTo(-6,6);
      ctx.lineTo(-6,-6);
      ctx.closePath();
      ctx.fill();
      ctx.restore();

      // Draw goal arrow
      if (goalRef.current) {
        const [gx, gy] = worldToCanvas(goalRef.current.x, goalRef.current.y);
        ctx.save();
        ctx.translate(gx, gy);
        ctx.rotate(-goalRef.current.theta);
        ctx.fillStyle = "green";
        ctx.beginPath();
        ctx.moveTo(10,0);
        ctx.lineTo(-6,6);
        ctx.lineTo(-6,-6);
        ctx.closePath();
        ctx.fill();
        ctx.restore();
      }

      rafRef.current = requestAnimationFrame(draw);
    };

    rafRef.current = requestAnimationFrame(draw);
  };

  return (
    <div className="page">
      <h1 className="title">Map View</h1>
      <canvas
        ref={canvasRef}
        onMouseDown={handleMouseDown}
        onMouseMove={handleMouseMove}
        onMouseUp={handleMouseUp}
        style={{ border: "1px solid #555", display: "block" }}
      />
      <p className="small">
        Pose: ({robotPoseRef.current.x.toFixed(2)}, {robotPoseRef.current.y.toFixed(2)})
      </p>
    </div>
  );
}
