# Gesture Controlled Robot Project

Hey! This is my project where I built a robot that you can control with hand gestures. Pretty cool right? You just point your finger and the robot moves around in Gazebo.

## What it does

I made this 4-wheel robot that watches your hand through a webcam and moves based on what you do:
- Point up with your index finger → robot goes forward
- Point up with your middle finger → robot goes backward
- Make a fist or do anything else → robot stops

The whole thing runs on ROS2 and I can see everything happening in RViz while testing it in Gazebo.

## What you need

You'll need Ubuntu (I used 22.04) with ROS2 installed. Also need a webcam obviously/webcam.

**Software stuff:**
- ROS2 Rolling 
- Gazebo (for simulation)
- RViz (to see what's happening)
- Python 3.12
- A webcam

## How to set it up

1. First clone this somewhere:
```bash
git clone <https://github.com/BishalDas1010/QuadBot.git>
cd Ros2_4wheel_Control_bot
```

2. I had to use a virtual environment because of some package conflicts:
```bash
cd src
python3 -m venv .venv
source .venv/bin/activate
pip install mediapipe opencv-python (most important)
```

3. Install the ROS2 dependencies:
```bash
sudo 
```

4. Build everything (make sure you're in the main folder, not src):
```bash
cd ..
colcon build
source install/setup.bash
```

## Running it

This was tricky to figure out, but here's what works:

```bash
# Go to your workspace
cd /path/to/your/Ros2_4wheel_Control_bot

# Activate the virtual environment
source src/.venv/bin/activate

# Set up the Python path (this was the key!)
export PYTHONPATH=$PWD/src/.venv/lib/python3.12/site-packages:$PYTHONPATH

# Build and source
colcon build --packages-select my_robo_controller
source install/setup.bash

# Finally run it
ros2 run my_robo_controller robo_control_node
```

If you see a camera window pop up, you're good to go! Wave your hand in front of it.

## The gestures

Keep it simple:
- **Point with index finger** = forward
- **Point with middle finger** = backward  
- **Everything else** = stop

The robot publishes to `/cmd_vel` so you can use it with any robot that listens to Twist messages.

## What I learned

- MediaPipe is pretty awesome for hand tracking
- ROS2 + virtual environments can be a pain to set up
- Always build from the workspace root, not the src folder
- The PYTHONPATH thing was crucial - took me forever to figure that out

## Common issues I ran into

**"No module named 'mediapipe'"** - This means the Python path isn't set right. Make sure you export PYTHONPATH like shown above.

**Camera window doesn't show** - Your system might not have a GUI or the camera permissions are wrong. Try `ls /dev/video*` to see if your camera is detected.

**Robot doesn't move** - Check that something is subscribing to `/cmd_vel`. You can test with `ros2 topic echo /cmd_vel`.

## Files in this project

- `robo_control_node.py` - Main gesture control node
- `package.xml` - ROS2 package dependencies  
- `setup.py` - Python package setup
- `logger.py` - Simple logging node for testing

## Future improvements

- Add more gestures (left/right turns, speed control)
- Better hand tracking in different lighting
- Maybe add voice commands too
- Clean up the installation process

## Notes

This was my first time combining ML with ROS2 and it was pretty fun! The hardest part was definitely getting all the Python packages to work together. If you're trying this, don't give up on the environment setup - it's worth it when you see the robot actually responding to your gestures.

Feel free to message me if you get stuck on anything!
