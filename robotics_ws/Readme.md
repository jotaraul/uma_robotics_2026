sudo apt install xterm

sudo rosdep init
rosdep update
rosdep install --from-paths src


colcon build --symlink-install

source install/setup.bash