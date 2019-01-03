## TeamCode Module

Welcome!

## Explanation of functions in WonderWomen class as of 1-3-19

Our WonderWomen class has many functions that we make to use in both Autonomous and TeleOp programs. This makes reusing code easy.
If you see a reference to a MyGoldDetector detector, this is the MyGoldDetector program. This program is one that came
with the OpenCV file we found online. We have modified this file to fit our needs and outline the gold mineral with a rectangle.

##### public void initArmMotors()
	FUNCTION: 
        - reads hardware map and initializes DC arm motors for our robot, initializes the limit switches and touch sensor for the arm
	NOTES: 
	    - Used in initRobot function


##### public void initDriveMotors() 
 	FUNCTION: 
        - reads hardware map and initializes DC drive motors for our robot
	NOTES: 
        -Used in initRobot function


##### public void initIMUGyro() 
 	FUNCTION: 
        -initializes gyro sensor built in on REV hub
	NOTES: 
        -Used in initRobot function



##### public void initRobot(HardwareMap hwMap, LinearOpMode opmode) 
 	INPUT: 
        - HardwareMap hwMap is hardwareMap ( you don’t need to change anything between opmodes)
        - LinearOpMode opmode is the type of opmode (usually this if extends Linear OpMode)
 	FUNCTION: 
        - to be used at the start of opmodes
		- gets robot ready to run (sets motors, sensors, etc.)

##### public void initRobot(HardwareMap hwMap, OpMode opmode) -
 	INPUT: 
        - HardwareMap hwMap is hardwareMap ( you don’t need to change anything between opcodes)
        - OpMode opmode is the type of opcode 
 	FUNCTION: 
        - same as LinearOpMode initRobot, but initializes for an iterative OpMode		
        - gets robot ready to run (sets motors, sensors, etc.)


##### public double getIMUBearing()
	FUNCTION:
        - reads IMU gyro sensor and returns the angle reading


##### public static double Angledistance(double alpha, double beta)
	INPUT:  
        - alpha is 1st angle
      	- beta is 2nd angle
	OUTPUT : 
        - angle distance between 2 angles (if 0 is start of circle, left side goes from 0 through 180 and the right side goes from 0 through -180)


##### public void gyroTurn(double turnAngle)
 	INPUT: 
        - the angle degree you want to turn (- for right turns, + for left)
 	FUNCTION: 
        - turns robot that number of degrees


##### public void setHardwareMap(HardwareMap hwMap)
 	FUNCTION:
        - sets the Hardware Map
 	INPUT: 
        - hardwareMap


##### public void setOpMode (LinearOpMode opcode)
 	FUNCTION:
        - sets the OpMode to use LinearOpMode
 	INPUT: 
        - current opcode (this)

##### public void setOpMode (OpMode opcode)
 	FUNCTION:
        - sets the OpMode to use an iterative OpMode
 	INPUT: 
        - current opcode (this)


##### public void setMecanumPower(double drive, double strafe, double turn)
	INPUTS: 
        - Drive is the stick value in the TeleOp program for driving.
		- strafe is whether you strafed or not
		- turn is the stick value in the TeleOp program for turning
		
 	FUNCTION: 
        - the reason we have this function and the one below it is because if no maxspeed is being inputed into this function, it automatically gets set to 1


##### public void setMecanumPower(double drive, double strafe, double turn, double maxspeed)
 	same as above except it has a max speed parameter so
 	if bumpers are being pressed, the maxspeed value will be able to change


##### public void setTankPower(double right, double left, double strafe, double maxspeed)
 	INPUTS: 
        - right is right stick value in TeleOp
        - left is left stick value in TeleOp
		- strafe is whether you strafe
        - max speed is the max speed of the robot
 	FUNCTION: 
        - sets the motors to tank drive


 ##### public void setTankPower(double right, double left, double strafe)
 	INPUTS:
        - same as above
 	FUNCTION: 
        - the reason for this function is the same reason as having the setMecanumPower function without a maxspeed parameter


##### public void setDrivePower(double frontrightPower, double frontleftPower, double backleftPower, double backrightPower, double maxspeed)
 	INPUTS: 
        - what they say (frontrightPower is the power for the front right wheel, etc.)
		- maxspeed is max speed
 	FUNCTION: 
        - this function sets the power of each motor to the parameter corresponding with the motor


#####public void setDrivePower(double frontrightPower, double frontleftPower, double backleftPower, double backrightPower)
 	INPUTS: 
        - same as other setDrivePower but no max speed
 	FUNCTION:
        - purpose is to set max speed to 1 if not specified


##### public void driveByTicks(int ticks, double speed)
	INPUT: 
        - ticks is # of ticks you want the motor to go
        - speed is the speed of the motors

 	FUNCTION: 
        - runs the motors until reached the # of ticks specified


##### public void resetEncoder()
	FUNCTION: 
        - resets the drive motors’ encoders
	NOTES: 
        - Do not use before TeleOp because TeleOp does not work in the STOP_AND_RESET_ENCODER RunMode


##### public void RetractAndResetArmEncoder()	FUNCTION: 
        - retracts extender until touching the touch sensor and resets the encoder
	NOTES: 
        - This was created for testing purposes, however we do not use this in our programs or to initialze. Our extender TeleOp cannot be reset at this position. (see extenderController below)


##### public void driveForInches(int inches, double speed)
	INPUT:  
        - int inches: the number of inches you need to go forward
		- double speed: speed of robot
	FUNCTION:
        - drives the robot forward a certain number of inches using the encoders
	NOTES:
        - to go backwards, use negative inches, not negative speed


##### public void strafeForInches(int inches, double speed)
	INPUT:  
        - int inches: the number of inches you need to strafe 
		- double speed: speed of robot
	FUNCTION: 
        - strafes the robot a certain number of inches 
	NOTES: 
        - positive inches is right, negative is left


##### public void RaiseRotationArm()
	FUNCTION: 
        - rotates the arm up until the limit switch is activated


##### public void LowerRotationArm()
	FUNCTION: 
        - rotates the arm down until the limit switch is activated


##### public void RetractingArm()
	FUNCTION:
	        - retracts extender until touch sensor is pressed


##### public void ExtendingArm()
	FUNCTION:
        - extends arm up to lower from lander; uses encoder and a pre-set number of ticks


##### public void setRotationArmPower(double power)
	INPUT: 
        - double power: power of motor
	FUNCTION:
        - sets the rotation arm motor to set power


##### public void setExtenderArmPower(double power)
	INPUT: 
        - double power: power of motor
	FUNCTION: 
        - sets the extendermotor to set power


##### public void setIntakePower(double power)
	INPUT: 
        - double power: power of motor
	FUNCTION: 
        - sets the intake motor to set power
	NOTES:
        - for auto, we use seconds to run the intake motor, so we this function should be used in the actual OpMode/LinearOpMode program, not inside the WonderWomen class


##### public void SUPERUNSAFEEXTENDERCONTROLLER(double extender)
	INPUT:
        - double extender: joystick value (for use in TeleOp)
	FUNCTION:
        - sets extender motor to power of joystick
	NOTES:
        - We do not use this in our TeleOp because it is unsafe for our motors. It will not stop at the touch sensor, so we could break our motors. This was used for testing purposes only


##### public void resetExtender()
	FUNCTION: 
        - resets the extender encoder and resets the mode to run using encoder

##### public void resetRotator()
	FUNCTION: 
        - resets the orator encoder and resets the mode to run using encoder


##### public void extenderController(double extender)
	INPUTS:
        - double extender: joystick value for extender motor
	FUNCTION: 
        - same as extenderController(double extender, boolean tele) but does not have telemetry for the extender motor

##### public void extenderController(double extender, boolean tele)
	INPUTS:  
        - double extender: joystick value for extender motor
        - boolean tele: whether you want telemetry for the extender motor
	FUNCTION: 
        - sets the extender motor value to joystick value if it’s possible (checks direction of extender and compares it to the touch sensor) It stops the motor if touch sensor is pressed, but allows you to go the other direction.. It shows the telemetry reading for the extender motor’s encoder, the prevention state (the direction it cannot go in) and direction you want to go.


##### public void rotatorController(double rotator, boolean tele)
	INPUT:  
        - double rotator: joystick value for rotator motor
        - boolean tele: whether you want telemetry for the rotator motor
	FUNCTION: 
        - sets the rotator motor value to joystick value if it’s possible (checks direction of rotator and compares it to the limit switches) It stops the motor if limit switches are activated, but allows you to go the other direction. It shows the telemetry reading for the rotator motor’s encoder and the prevention state (the direction it cannot go in)


##### public void rotatorController(double rotator)
	INPUT: 
        - double rotator: joystick value for rotator motor
	FUNCTION: 
        - same as rotatorController above but does not show telemetry reading


##### public void extenderForTicks(int ticks, double speed)
	INPUT: 
        - int ticks: number of ticks you want to go
        - double speed: the speed of the extender motor (should be 1; it’s REALLY slow)
	FUNCTION: 
        - runs extender to go for a set number of ticks
	NOTES:
        - to go down, use negative ticks, not negative speed


##### public void IntakeForTicks(int ticks, double speed)
	INPUT:  
         - int ticks: number of ticks you want to go
         - double speed: the speed of the intake motor
	FUNCTION: 
        - runs intake for a set number of ticks
	NOTES: 
        - to go the opposite direction, use negative ticks, not negative speed


##### public void RotatorForTicks(int ticks, double speed)
	INPUT:  
	    - int ticks: number of ticks you want to go
	    - double speed: the speed of the rotator motor
	FUNCTION: 
        - runs rotator for a set number of ticks
	NOTES: 
        - to go down, use negative ticks, not negative speed


##### public void alignGold(MyGoldDetector detector)
    INPUT: 
        - MyGoldDetector detector: the gold detector needs to be declared in the actual OpMode, but we need functions from the detector, so this input allows us to access the detector
	FUNCTION:
        - goes forward until the gold block on the camera is in the “ideal spot”, which we set above
    NOTES: 
        - We do not use this function in our auto. It did work as reliably as we intended.


##### public void findGold(MyGoldDetector detector)
	INPUT: 
        - MyGoldDetector detector: the gold detector needs to be declared in the actual OpMode, but we need functions from the detector, so this input allows us to access the detector
	FUNCTION: 
        - this function finds the gold mineral by checking the middle first, then the right, then the left. Once the mineral is found, it sets the goldSide enum to the side the gold is on. This enum is used in later functions.


##### public void depotClaimFromDepot()
	FUNCTION:
        - used in auto that starts at the depot; uses the goldSide enum to get to the depot depending on which side the gold mineral was on


##### public void goToCraterFromDepot()
	FUNCTION:
        - depending on the goldSide enum, it moves the robot to end in the crater. Used in depotAuto





## Creating your own OpModes

This module, TeamCode, is the place where you will write/paste the code for your team's
rosbot controller App. This module is currently empty (a clean slate) but the
process for adding OpModes is straightforward.

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