# roboteam\_input

## Getting the controllers right
Just plug them all in. Start the `joysticks.launch` file. Then assign the joysticks to the right players. Keep in mind that the xbox receiver creates 4 controllers, even though you might only connect 1 xbox controller.

## Testing which controller is which & if they are functioning properly.

`ls /dev/input/` should tell you which joysticks are available. They are typically numbered from js0 to js8 or some other N. With `jstest /dev/input/jsX` (where X = some integer between 0 and infinity) you can check if the joystick is working correctly, and also check which joystick is connected to that port.

## Installing SDL2 for keyboard controller

`sudo apt-get install libsdl2-dev libsdl2-ttf-dev` 
`sudo apt-get install ros-melodic-joystick-drivers`  should take care of it. If you don't have it installed it should not give an error but just not compile the keyboard controller.
