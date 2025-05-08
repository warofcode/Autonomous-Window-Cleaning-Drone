 
Project Name: AeroClean: Autonomous Window Cleaning Drone

AeroClean is an intelligent autonomous drone system designed for the efficient and safe cleaning of high-rise building windows. Using onboard sensors, computer vision, and path-planning algorithms, the drone scans building facades, identifies window surfaces, plans optimal cleaning routes, and executes precise cleaning operations—minimizing human risk and maximizing cleaning efficiency.

Key Features:
🛫 Autonomous Takeoff, Navigation, and Landing
Fully automated flight from launch to landing with GPS return-to-home.

🧠 Multi-State Finite State Machine
Manages drone behavior through states like IDLE, SCANNING, CLEANING, RETURNING, and EMERGENCY.

📷 Window Detection via Simulated Computer Vision
Scans the building facade and detects window locations using mock visual detection.

🗺️ Dynamic Path Planning
Creates an optimized cleaning path based on window positions and cleaning strategy (e.g., zigzag).

🧼 Cleaning Simulation with Consumable Management
Simulates spraying and wiping mechanisms while tracking battery and cleaning fluid usage.

🛑 Safety Protocols & Emergency Handling
Low battery triggers auto-return, and critical failures initiate emergency landing.

🔄 Maintenance Routines
Includes methods for recharging the battery and refilling cleaning fluid.