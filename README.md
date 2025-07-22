# nun_bot_ros
to start the keyboard controlling, start two terminals :
*in the first terminal*

```
source install/setup.bash
ros2 run nun_bot arduino_listener
```

*in the second termminal*

```
source isntall/setup.bash
ros2 run nun_bot keyboard talker
```
now when you press direction key of your keyboard in this terminal you will see the robot moving
