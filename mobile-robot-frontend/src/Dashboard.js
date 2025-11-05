// Dashboard.js
import React, { useState, useEffect } from "react";
import { motion } from "framer-motion";
import axios from "axios";

const API_BASE = "http://127.0.0.1:8000";

export default function Dashboard() {
  const [linear, setLinear] = useState(0);
  const [angular, setAngular] = useState(0);
  const [status, setStatus] = useState(null);
  const [error, setError] = useState(null);

  const sendCmdVel = async (l, a) => {
    setLinear(l);
    setAngular(a);
    setError(null);
    try {
      await axios.post(`${API_BASE}/cmd_vel`, { linear: l, angular: a });
    } catch (err) {
      setError("Failed to send command");
      console.error("cmd_vel error:", err?.message);
    }
  };

  useEffect(() => {
    let mounted = true;
    const interval = setInterval(async () => {
      try {
        const res = await axios.get(`${API_BASE}/status`);
        if (mounted) setStatus(res.data);
      } catch (err) {
        if (mounted) setStatus(null);
        console.error("status error:", err?.message);
      }
    }, 1000);
    return () => { mounted = false; clearInterval(interval); };
  }, []);

  return (
    <div className="page">
      <h1 className="title">🤖 Mobile Robot Dashboard</h1>
      <div className="cards-container">
        <div className="card">
          <h2>Control Panel</h2>
          <div className="grid">
            <motion.button whileTap={{ scale: 0.9 }} className="btn blue" onClick={() => sendCmdVel(0.2, 0)}>▲</motion.button>
            <div></div>
            <motion.button whileTap={{ scale: 0.9 }} className="btn blue" onClick={() => sendCmdVel(-0.2, 0)}>▼</motion.button>
            <motion.button whileTap={{ scale: 0.9 }} className="btn green" onClick={() => sendCmdVel(0, 0.5)}>◀</motion.button>
            <motion.button whileTap={{ scale: 0.9 }} className="btn red" onClick={() => sendCmdVel(0, 0)}>⏹ STOP</motion.button>
            <motion.button whileTap={{ scale: 0.9 }} className="btn green" onClick={() => sendCmdVel(0, -0.5)}>▶</motion.button>
          </div>
          <p className="small">Linear: {linear.toFixed(2)} | Angular: {angular.toFixed(2)}</p>
          {error && <p className="error">{error}</p>}
        </div>

        <div className="card">
          <h2>Robot Status</h2>
          {status ? (
            <div className="status">
              <p><b>Linear Vel:</b> {Number(status.linear || 0).toFixed(2)} m/s</p>
              <p><b>Angular Vel:</b> {Number(status.angular || 0).toFixed(2)} rad/s</p>
              <p><b>Battery:</b> {Number(status.battery || 0).toFixed(2)}%</p>
              <p><b>Mode:</b> {status.mode || "UNKNOWN"}</p>
            </div>
          ) : <p>Waiting for backend...</p>}
        </div>
      </div>
    </div>
  );
}
