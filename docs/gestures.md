\# Gesture Vocabulary



The system uses a small, intentional set of hand gestures.

Each gesture is designed to be robust and easy to perform.



---



\## Pinch



\*\*Description\*\*

\- Thumb tip and index tip move close together



\*\*Purpose\*\*

\- Acts as a clutch

\- Enables or disables continuous control



\*\*Used In\*\*

\- Stepper motor jog (Mode 1 and Mode 3)



---



\## Hand Rotation



\*\*Description\*\*

\- Rotation of the hand around the wrist axis

\- Measured using index MCP to pinky MCP vector



\*\*Purpose\*\*

\- Continuous control signal



\*\*Used In\*\*

\- Stepper motor position (Mode 1)

\- LED brightness (Mode 2)

\- Stepper motor speed (Mode 3)



---



\## Short Fist



\*\*Description\*\*

\- All fingers closed briefly



\*\*Purpose\*\*

\- Toggle action



\*\*Used In\*\*

\- LED on/off toggle (Mode 2)



---



\## Long Fist



\*\*Description\*\*

\- All fingers closed and held for ~2 seconds



\*\*Purpose\*\*

\- Mode switching



\*\*Used In\*\*

\- Cycle through control modes



---



\## Gesture Robustness



To improve reliability:

\- Distance thresholds use hysteresis

\- Some gestures use normalized landmark ratios

\- Time-based filtering prevents false triggers

\- Control signals are rate-limited



This allows stable interaction even with camera noise

or small hand movements.



