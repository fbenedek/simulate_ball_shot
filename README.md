# Simulate Ball Shots for Anymal
A quick ros package for shooting simulated balls at Anymal

## Clone and build
~~~
cd ~/git
git clone git@github.com:fbenedek/simulate_ball_shot.git
cd simulate_ball_shot/src
chmod a+x convert_to_markers.py npy_publisher.py simulate_shots.py
cd ~/catkin_ws/src
ln -s ~/git/simulate_ball_shot
cd ..
catkin build simulate_ball_shot
~~~

## Shooting 
To shoot balls with displacement <d> meters and speed <s>, publish a std_msgs/Float32MultiArray message to the /sim_capture/fire_ball_at_robot topic, similarly to below: 
~~~
rostopic pub /sim_capture/fire_ball_at_robot std_msgs/Float32MultiArray "{data: [<d>,<s>]}"
~~~
