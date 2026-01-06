\# System Architecture



This project is structured as a modular, real-time control system

that connects computer vision to physical hardware.



The system is divided into three main layers.



---



\## 1. Perception Layer



\- Camera captures live video

\- MediaPipe detects hand landmarks in real time

\- OpenCV handles image processing and visualization



Output:

\- Normalized 2D hand landmark coordinates

\- Hand classification (Left / Right)



---



\## 2. Control Layer



Implemented in Python.



Responsibilities:

\- Gesture recognition (pinch, fist, rotation)

\- Gesture hysteresis and debouncing

\- Multi-mode state machine

\- Mapping gestures to control signals

\- Smoothing and rate limiting



This layer converts noisy vision input into stable, deterministic commands.



---



\## 3. Actuation Layer



Implemented on Arduino.



Responsibilities:

\- Serial command parsing

\- Stepper motor control

\- PWM LED control

\- Speed and brightness updates



The Arduino layer is intentionally simple and deterministic.



---



\## Data Flow



Camera  

→ Python (vision + logic + state machine)  

→ Serial protocol  

→ Arduino  

→ Physical hardware (motor, LED)



---



\## Design Goals



\- Touchless control

\- Robust gesture detection

\- Minimal gesture vocabulary

\- Hardware-safe behavior

\- Easy extensibility



This architecture is intended for rapid prototyping,

interactive installations, and robotics UI experiments.



