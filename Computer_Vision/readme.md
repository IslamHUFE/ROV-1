# Underwater Trash Detection - YOLO Model for ROV

[![Python 3.8+](https://img.shields.io/badge/Python-3.8+-blue.svg)](https://www.python.org/downloads/)
[![YOLOv8](https://img.shields.io/badge/YOLOv8-Object%20Detection-red.svg)](https://github.com/ultralytics/ultralytics)

A fine-tuned YOLO-based object detection model specifically designed for autonomous underwater vehicles (ROVs) to detect and classify underwater trash and debris in real-time.

## 🌊 Overview
This repository contains a fine-tuned YOLO (You Only Look Once) object detection model for underwater trash detection.


## ✨ Features
- **Real-time detection**
- **Underwater optimized**: Handles challenging underwater conditions (low visibility, color distortion, marine life)
- **Lightweight**: Model quantization and pruning options for resource-constrained ROVs
- **Confidence scoring**: Probability scores for each detection
- **Bounding box coordinates**: Precise localization for ROV navigation

## 🏗️ Model Architecture
- **Base Model**: YOLOv8n (Ultralytics)
- **Fine-tuning**: Transfer learning on custom underwater dataset
- **Input Size**: 640×640 pixels (configurable)

### Dataset Split
- Training: (3626 images)
- Validation: (1,000 images)
- Testing: (501 images)

### Data Augmentation
- Blur 
- Random flips and rotations
- HSV adjustment (underwater color correction)
- Gaussian noise (simulating turbidity)


