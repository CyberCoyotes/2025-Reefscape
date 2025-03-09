# Tutorial Settings

These settings are based on a [tutorial](https://www.chiefdelphi.com/t/from-zero-to-auto-align-to-reef-tutorial-how-to-simply-auto-align-to-the-reef/494478) and other research online. I've also downloaded the pipeline and put in the `documentation` folder.

- **Enable 3d tracking** In the limelight web interface you need to set your pipeline to april tags than go the “advanced” tab and set the “Full 3D Targeting” option to “Yes”
- **Set your Visualizer setup** “Robot pose in Target Space” this means the data in the visualizer bellow will be from the pov of the april tag as of it is sable at a fixed point and the robot it moving relative to it.
- in the standard tab set the sort mode to closest
- Tolerance: this is how much error we are allowing. the smaller the value the more accurate we want to be but the harder it is to tune. I recommend putting this value a bit smaller than how much the robot needs to be aligned.
don’t make this value too large as than the aligning will stop to early and the robot won’t be aligned and don’t make it too small as than it will be really hard to tune, and also d you really need to be accurate to the mm or maybe 5 or 10 mm would be close enough
- Setpoints: **put your robot on the field where you want it to be at the end of the aligning (where it is when it scores) then open the limelight web interface and go to the advanced tab and get the robot position from there**
- Set the view to “Robot pose in Target Space”

set each setpoint to its relative value in the web interface

x stepoint = TX
y setpoint = TZ*
rot setpoint = RY

*if your camera isn’t centered with your scoring system you would need to have 2 different values for the right and left reef pipes. if it is centered you can just use the same value but but negative (-Constants.Y_SETPOINT_REEF_ALIGNMENT)

DONT_SEE_TAG_WAIT_TIME - how long that we don’t see the tag before we stop the command
POSE_VALIDATION_TIME - how long we need to be aligned before we finish the command.
too small of a value might make the robot miss if the pid isn’t perfect and you ahve overshoot, too long just waste time.

recommend putting this at a high value when tuning to make sure the pid is good and there isnt any overshoot and lower it later.

now the most important thing
the pid values this need to be changed by testing and tuning (there a lot of places and threads to learn how to tune) run the command see how it goes and change the values accordingly

## How the Code Works
xController - a pid controller that will do the aligning in the x direction

yController - a pid controller that will do the aligning in the y direction

rotController - a pid controller that will do the aligning of the robots rotation

isRightScore - each side of the reef has two scoring pipes this boolean is used to choose if you want to go to the right pipe if it is true or the left pipe if its false

dontSeeTagTimer - this timer counts the time since we last saw the tag we align to and if we haven’t seen for for some time it automatically stops the command

stopTimer - if the pid is not perfect we might overshot and miss the target this timer is used so that the robot waits a bit when its aligned to make sure it stopped and is truly aligned

tagID - the ID of the tag we align to. *its the first tag saw when we started aligning

### builder
setting the pid constants and saving the isRightScore

### initialize
creating the timers and starting them

setting the setpoints and tolerance for the pid controllers

setpoint - the target we want to align to

tolerance - what is our acceptable error

getting the id of the tag we align to

### execute
the if is checking if we see a target and if the target has the id we are aligning to

we reset the dontSeeTagTimer because we saw a tag

we get the pose of the robot relative to the tag as an array and put in the variable positions

we use the pid to calculate the speed we want the robot to drive at giving it the distance from the tag and it giving us what speeds to drive the robot

drive - setting the speeds for the robot to drive at

!!!NOTICE the drive has to be robot relative and not field relative

“yController.getError() < Constants.Y_TOLERANCE_REEF_ALIGNMENT ? xSpeed : 0” - this line in the drive code is used to make sure we are aligned in the y direction before we start alignin in the x direction

if you want both alignment to happen at the same time replace this with xSpeed

### isFinished

check if we either didn’t see a tag in some time or we are aligned and if so stops the command