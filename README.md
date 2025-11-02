# ROS 2 Orbbec Validation Protocol

A ROS 2 Jazzy-based validation protocol for the Orbbec Gemini 336L camera with automatic MCAP rosbag recording, deployed via Docker Compose.

## Overview

This project provides a complete Docker-based setup for:
- Running the Orbbec Gemini 336L camera driver on ROS 2 Jazzy
- Automatically recording all camera topics to rosbag with MCAP storage format
- Persistent storage of recordings to `/tmp/recordingns` on the host

