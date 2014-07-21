The main file with all the code is scripting_interfaces.py. To run it, 
you must first alter it's unix permissions to allow executing it. Next,
place it and the corresponding images folder somewhere in your RSDK
directory, probably alongside some other scripts that Rething has
already created, or, if you're feeling brave/want more organization,
you can create your own script directory. Just follow the examples
already set up by Rethink. Also, open up the script and alter the
constant IMAGE_DIRECTORY at the top to be the absolute path to the
images folder that came with this program (you can make it relative
if you want, if you can figure out how).

Run scripting_interfaces.py just like any other of Rethink's scripts,
e.g., "rospy wherever-you-put-it/scripting_interfaces.py -r".
Alternatively, you can change "-r" to "-k" if you'd rather use the
keyboard & mouse interface instead of the on-robot interface.

Please note that this script does not initialize the grippers. If
the error console yells at you because the grippers aren't initialized,
then you can use one of Rethink's scripts to do so. Also, this script
doesn't turn off the robot when it exits, so you'll have to do that
using enable.py with the -d command when you want to shut Baxter off.