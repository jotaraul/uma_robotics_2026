This pkg implements two simple keyboard-based controllers for mobile robots:

# Keyboard_control:
Sends Twist commands according to the keys pressed. Only one key can be pressed simultaneously, and if the key is released, no Twist msg is sent anymore. Allows a simple but very reactive control.

### Parameters
    <param name="linear_v" value="5.0" /> 
    <param name="angular_v" value="5.0" /> 
    <param name="publish_topic" value="/cmd_vel" />

# Keyboard_control_plus: 
Improved version that continuously sends Twist msgs to the robot. Keys allow to imcrease/decrease both linear and angular current velocities, allowing for soft trayectories.

### Parameters
    <param name="linear_v_inc" value="0.1" />
    <param name="angular_v_inc" value="0.1" />
    <param name="publish_topic" value="/cmd_vel" />