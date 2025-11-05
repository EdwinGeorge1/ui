// App.js
import React from "react";
import { BrowserRouter as Router, Routes, Route, Link } from "react-router-dom";
import Dashboard from "./Dashboard";
import Logs from "./Logs";
import Settings from "./Settings";
import MapView from "./MapView";
import "./App.css";

export default function App() {
  return (
    <Router>
      <div className="layout">
        <nav className="sidebar">
          <h2>AMR</h2>
          <Link to="/" className="nav-link">Dashboard</Link>
          <Link to="/map" className="nav-link">Map View</Link>
          <Link to="/logs" className="nav-link">Logs</Link>
          <Link to="/settings" className="nav-link">Settings</Link>
        </nav>

        <main className="content">
          <Routes>
            <Route path="/" element={<Dashboard />} />
            <Route path="/map" element={<MapView />} />
            <Route path="/logs" element={<Logs />} />
            <Route path="/settings" element={<Settings />} />
          </Routes>
        </main>
      </div>
    </Router>
  );
}
