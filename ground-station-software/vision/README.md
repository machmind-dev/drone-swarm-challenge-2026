# GCS Vision — ArUco Offboard Detection (Abandoned)

This was the initial vision solution developed for **Drone Version 1.0**. ArUco marker detection ran on the Ground Control Station using OpenCV (`aruco_node.py`), with the camera stream streamed from the drone over WiFi.

The approach was abandoned due to connectivity issues over the **2.4 GHz WiFi band** — the video stream introduced latency and packet loss that made reliable real-time pose estimation impractical in a competition environment with multiple drones and interference.

For Version 2.0, ArUco detection was moved fully **onboard** the drone (ESP32-P4), eliminating the dependency on the wireless video link.

<img src="aruco-offboard-1.jpeg" width="868">

<img src="aruco-offboard-2.jpeg" width="868">
