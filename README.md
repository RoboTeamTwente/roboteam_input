# roboteam\_input

## Getting the controllers right
Make sure in roboteam\_input.launch that every joy\_node points to a "functioning"
joystick. So if a joy\_node has /dev/input/js0 as input, it has the first connected joystick
as input (unless your laptop has a gyroscope!). 

Then, fill in each joy\_node name in the roboteam\_input node with the correct type and
robot to be controlled. For example, let's say you connect the playstation controller.
In the joy\_node with name js0, we give the dev param the value /dev/input/js0 to indicate the first connected playstation controller. Then we look at roboteam\_input, change input0/input to js0, and input0/joyType to playsation.

Keep in mind that Bob's wireless xbox connector allocates 4 (!) joysticks. So if you first connect the wireless connector (without even connecting an xbox controller!), 4 joysticks will be created (/dev/input/js0, js1, js2, and js3. Then, as xbox controllers connect, they will be named js0, js1, js2, js3, as the slots fill up). If you then connect a playstation controller via cable, it will appear as /dev/input/js4. This makes it tricky to configure the joy\_nodes and roboteam\_input node correctly, but it'll have to make do for now.

