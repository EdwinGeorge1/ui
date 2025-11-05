// Logs.js
import React, { useState, useEffect } from "react";
import axios from "axios";

const API_BASE = "http://127.0.0.1:8000";

export default function Logs() {
  const [logs, setLogs] = useState([]);

  useEffect(() => {
    let mounted = true;
    const interval = setInterval(async () => {
      try {
        const res = await axios.get(`${API_BASE}/logs`);
        if (mounted) setLogs(res.data);
      } catch (err) {
        if (mounted) setLogs([]);
        console.error("logs error:", err?.message);
      }
    }, 2000);
    return () => { mounted = false; clearInterval(interval); };
  }, []);

  return (
    <div className="page">
      <h1 className="title">📜 Robot Logs</h1>
      <div className="logs">
        {logs.length > 0 ? logs.map((log, i) => (
          <div key={i} className="log-item">{log.time} → {log.message}</div>
        )) : <p>No logs yet...</p>}
      </div>
    </div>
  );
}
