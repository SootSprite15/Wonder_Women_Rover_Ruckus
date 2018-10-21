## TeamCode Module

Welcome!

Explanation of functions in WonderWomen class as of 10-21-18

public void initDriveMotors() -
	FUNCTION: reads hardware map and initializes DC motors for our robot


public void initIMUGyro() -
	FUNCTION: initializes gyro sensor built in on REV hub


public void initRobot(HardwareMap hwMap, LinearOpMode opmode) -
	INPUT: HardwareMap hwMap is hardwareMap ( you don’t need to change anything between opcodes)
		LinearOpMode opmode is the type of opcode (usually this if extends Linear OpMode)
	FUNCTION: to be used at the start of opcodes
		  gets robot ready to run (sets motors, sensors, etc.)


public double getIMUBearing()
	FUNCTION:reads IMU gyro sensor and returns the angle reading


public static double Angledistance(double alpha, double beta)
    INPUT: alpha is 1st angle
           beta is 2nd angle
    OUTPUT : angle distance between 2 angles (if 0 is start of circle, left side goes from 0 through 180 and the right side goes from 0 through -180)


public void gyroTurn(double turnAngle)
	INPUT: the angle degree you want to turn (- for right turns, + for left)
	FUNCTION:turns robot that number of degrees


 public void setHardwareMap(HardwareMap hwMap)
	FUNCTION:sets the Hardware Map
	INPUT: hardwareMap


public void setOpMode (LinearOpMode opcode)
	FUNCTION:sets the OpMode
	INPUT: current opcode (this)


public void setMecanumPower(double drive, double strafe, double turn)
	INPUTS: drive is the stick value in the TeleOp program for driving
		strafe is whether you strafed or not
		turn is the stick value in the TeleOp program for turning
	FUNCTION: the reason we have this function and the one below it is because if no maxspeed is being inputed into this function, it automatically gets set to 1


public void setMecanumPower(double drive, double strafe, double turn, double maxspeed)
	same as above except it has a max speed parameter so
	if bumpers are being pressed, the maxspeed value will be able to change


public void setTankPower(double right, double left, double strafe, double maxspeed)
	INPUTS: right is right stick value in TeleOp
		left is left stick value in TeleOp
		strafe is whether you strafe
		max speed is the max speed of the robot
	FUNCTION: sets the motors to tank drive


 public void setTankPower(double right, double left, double strafe)
	INPUTS are same as above and
	FUNCTION: the reason for this function is the same reason as having the setMecanumPower function without a maxspeed parameter


public void setDrivePower(double frontrightPower, double frontleftPower, double 			backleftPower, double backrightPower, double maxspeed)
	INPUTS: what they say (frontrightPower is the power for the front right wheel, etc.)
		maxspeed is max speed
	FUNCTION: this function sets the power of each motor to the parameter corresponding with the motor


public void setDrivePower(double frontrightPower, double frontleftPower, double 			backleftPower, double backrightPower)
	INPUTS: same as other setDrivePower but no max speed
	FUNCTION: purpose is to set max speed to 1 if not specified


public void setHexMotorPower()
	FUNCTION: sets the HexMotor to run (used for a test run)


public void driveByTicks(int ticks, double speed)
	INPUT: ticks is # of ticks you want the motor to go
		speed is the speed of the motors

	FUNCTION: runs the motors until reached the # of ticks specified





This module, TeamCode, is the place where you will write/paste the code for your team's
robot controller App. This module is currently empty (a clean slate) but the
process for adding OpModes is straightforward.

## Creating your own OpModes

The easiest way to create your own OpMode is to copy a Sample OpMode and make it your own.

Sample opmodes exist in the FtcRobotController module.
To locate these samples, find the FtcRobotController module in the "Project/Android" tab.

Expand the following tree elements:
 FtcRobotController / java / org.firstinspires.ftc.robotcontroller / external / samples

A range of different samples classes can be seen in this folder.
The class names follow a naming convention which indicates the purpose of each class.
The full description of this convention is found in the samples/sample_convention.md file.

A brief synopsis of the naming convention is given here:
The prefix of the name will be one of the following:

* Basic:    This is a minimally functional OpMode used to illustrate the skeleton/structure
            of a particular style of OpMode.  These are bare bones examples.
* Sensor:   This is a Sample OpMode that shows how to use a specific sensor.
            It is not intended as a functioning robot, it is simply showing the minimal code
            required to read and display the sensor values.
* Hardware: This is not an actual OpMode, but a helper class that is used to describe
            one particular robot's hardware devices: eg: for a Pushbot.  Look at any
            Pushbot sample to see how this can be used in an OpMode.
            Teams can copy one of these to create their own robot definition.
* Pushbot:  This is a Sample OpMode that uses the Pushbot robot structure as a base.
* Concept:	This is a sample OpMode that illustrates performing a specific function or concept.
            These may be complex, but their operation should be explained clearly in the comments,
            or the header should reference an external doc, guide or tutorial.
* Library:  This is a class, or set of classes used to implement some strategy.
            These will typically NOT implement a full OpMode.  Instead they will be included
            by an OpMode to provide some stand-alone capability.

Once you are familiar with the range of samples available, you can choose one to be the
basis for your own robot.  In all cases, the desired sample(s) needs to be copied into
your TeamCode module to be used.

This is done inside Android Studio directly, using the following steps:

 1) Locate the desired sample class in the Project/Android tree.

 2) Right click on the sample class and select "Copy"

 3) Expand the  TeamCode / java folder

 4) Right click on the org.firstinspires.ftc.teamcode folder and select "Paste"

 5) You will be prompted for a class name for the copy.
    Choose something meaningful based on the purpose of this class.
    Start with a capital letter, and remember that there may be more similar classes later.

Once your copy has been created, you should prepare it for use on your robot.
This is done by adjusting the OpMode's name, and enabling it to be displayed on the
Driver Station's OpMode list.

Each OpMode sample class begins with several lines of code like the ones shown below:

```
 @TeleOp(name="Template: Linear OpMode", group="Linear Opmode")
 @Disabled
```

The name that will appear on the driver station's "opmode list" is defined by the code:
 ``name="Template: Linear OpMode"``
You can change what appears between the quotes to better describe your opmode.
The "group=" portion of the code can be used to help organize your list of OpModes.

As shown, the current OpMode will NOT appear on the driver station's OpMode list because of the
  ``@Disabled`` annotation which has been included.
This line can simply be deleted , or commented out, to make the OpMode visible.



## ADVANCED Multi-Team App management:  Cloning the TeamCode Module

In some situations, you have multiple teams in your club and you want them to all share
a common code organization, with each being able to *see* the others code but each having
their own team module with their own code that they maintain themselves.

In this situation, you might wish to clone the TeamCode module, once for each of these teams.
Each of the clones would then appear along side each other in the Android Studio module list,
together with the FtcRobotController module (and the original TeamCode module).

Selective Team phones can then be programmed by selecting the desired Module from the pulldown list
prior to clicking to the green Run arrow.

Warning:  This is not for the inexperienced Software developer.
You will need to be comfortable with File manipulations and managing Android Studio Modules.
These changes are performed OUTSIDE of Android Studios, so close Android Studios before you do this.
 
Also.. Make a full project backup before you start this :)

To clone TeamCode, do the following:

Note: Some names start with "Team" and others start with "team".  This is intentional.

1)  Using your operating system file management tools, copy the whole "TeamCode"
    folder to a sibling folder with a corresponding new name, eg: "Team0417".

2)  In the new Team0417 folder, delete the TeamCode.iml file.

3)  the new Team0417 folder, rename the "src/main/java/org/firstinspires/ftc/teamcode" folder
    to a matching name with a lowercase 'team' eg:  "team0417".

4)  In the new Team0417/src/main folder, edit the "AndroidManifest.xml" file, change the line that contains
         package="org.firstinspires.ftc.teamcode"
    to be
         package="org.firstinspires.ftc.team0417"

5)  Add:    include ':Team0417' to the "/settings.gradle" file.
    
6)  Open up Android Studios and clean out any old files by using the menu to "Build/Clean Project""