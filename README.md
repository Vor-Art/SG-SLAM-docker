# SG‑SLAM (Docker Edition)

**Simultaneous Globally‑consistent SLAM (SG‑SLAM)** – now fully containerised for repeatable builds on any modern Linux host (tested on Ubuntu 24.04, ROS 2 workstations, NVIDIA & Intel GPUs).
This fork adds a one‑command Docker/Compose workflow, so you can demo or hack on the algorithm without polluting your system.

---

## ✨ Highlights

* **Hermetic build** – Pangolin 0.5, OpenCV 3.4.15, Eigen 3.1.0, ncnn (RTTI ON) and ROS Melodic pre‑compiled in the image.
* **GPU‑ready** – automatic Vulkan/NVIDIA acceleration if `nvidia‑docker` is present.
* **Hot‑reload dev loop** – host `SG‑SLAM/` folder is bind‑mounted, so `catkin_make` inside the container recompiles instantly.
* **Cross‑desktop GUI** – RViz & Pangolin forwarded over X11; Wayland supported via XWayland (`QT_QPA_PLATFORM=xcb`).

---

## 🚀 Quick start

### 1  Clone & enter the repo

```bash
git https://github.com/Vor-Art/SG-SLAM-docker.git
cd SG-SLAM-docker
```

### 2  Prerequisites

| Tool                          | Minimum version   | Install                                                                            |
| ----------------------------- | ----------------- | ---------------------------------------------------------------------------------- |
| **Docker Engine**             | 20.10 +           | [https://docs.docker.com/engine/install/](https://docs.docker.com/engine/install/) |
| **Docker Compose v2**         |  2.5 +            | `sudo apt install docker‑compose‑plugin` or follow Docker docs                     |
| **(optional) NVIDIA drivers** |  535 +            | required for GPU acceleration                                                      |

### 3  Build the image (once)

```bash
xhost +local:root          # allow GUI apps from the container
docker compose build       # ~10 min first time (OpenCV build)
```

### 4  Run the demo

Place your TUM dataset files under `./datasets/TUM` on your host machine.
And update the `TUN_DATASET` in the `.env` file.

Create the association file:
```bash
source .env && python3 scripts/TUN_rgbd_tools/associate_3.py $TUN_DATASET/rgb.txt $TUN_DATASET/depth.t
xt > $TUN_DATASET/associations.txt
```

Run service:

```bash
docker compose up sg_slam_tum
```

---


## ⚖️ License

SG‑SLAM remains under the original MIT license.
Docker‑specific files in this fork are also MIT.

---

## 🙏 Acknowledgements

Credits to the original SG‑SLAM authors and the ROS / OpenCV / Pangolin communities.

---

> **Happy mapping!**

⚠️ *This README was partially drafted with the assistance of an AI language model (ChatGPT, June 2025). Please review and test the commands in your own environment before using them in production.*
