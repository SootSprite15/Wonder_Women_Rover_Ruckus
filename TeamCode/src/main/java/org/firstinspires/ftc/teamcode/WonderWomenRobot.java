package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.hardware.rev.RevTouchSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Rotation;
import org.opencv.core.Point;

import java.util.Timer;

//not an opmode
//to be used by every opmode
//In this file we define the robot and its motions. Opmodes then use these motions.


public class WonderWomenRobot {

    //OpMode members
    private DcMotor FrontRight = null;
    private DcMotor FrontLeft = null;
    private DcMotor BackRight = null;
    private DcMotor BackLeft = null;
    private DcMotor RotationArm = null;
    private DcMotor Extender = null;
    private DcMotor Intake = null;

    private HardwareMap hardwareMap = null;
    private OpMode opmode = null;
    //   private LinearOpMode opmode1 = null;

    private DistanceSensor sensorRange;//generic distance sensor
    private BNO055IMU imu;
    private Orientation angles;
    Rev2mDistanceSensor sensorTimeOfFlight;//extra fancy distance sensor extends DistanceSensor

    DigitalChannel LoweringLimitSwitch;
    DigitalChannel RaisingLimitSwitch;
    DigitalChannel RetractionTouchSensor;

    private int rotatorStopDirection = 0;  //1 means can't go positive, -1 = can't go negative, 0 = can go both directions
    private int extenderStopDirection = 0;

    static double TICKS = -2240;
    static double EXTENDER_TICKS = -2240;
    static double TICKSFORNEVEREST40MOTOR = 1120;
    static double PI = 3.1415926535897932384626433;
    static double wheelDiameter = 4;
    static double circumferenceOfWheel = PI * wheelDiameter;
    static double motorRotationTeeth = 20;
    static double wheelRotationTeeth = 10;
    static double gearRatio = 0.25;
    static double extenderGearRatio = 10;
    static double gearDiameter = 2;
    static double circumferenceOfSecondGear = PI * gearDiameter;
    static double extenderTicksPerInch = EXTENDER_TICKS * (extenderGearRatio) * (1 / circumferenceOfSecondGear);
    static double ticksPerInch = TICKS * (gearRatio) * (1 / circumferenceOfWheel);
    static int raisingArmTicks = 40000;

    static double idealX = 450;
    static double idealY = 250;
    static double idealThreshold = 10;

    enum rotatorDirect {UP, STOP, DOWN}

    ;

    enum rotatorPrevent {UP, NONE, DOWN}

    ;
    rotatorPrevent rotatorState = rotatorPrevent.NONE;

    enum extenderDirect {NEG, STOP, POS}

    ;

    enum extenderPrevent {NEG, NONE, POS}

    ;
    extenderPrevent extenderstate = extenderPrevent.NONE;

    enum mineral {LEFT, MIDDLE, RIGHT, UNKNOWN}

    ;
    mineral goldSide = mineral.UNKNOWN;


    //Initialize drive motors for the arm (Extender, Intake, and RotationArm)
    public void initArmMotors() {
        //naming the motors
        RotationArm = hardwareMap.get(DcMotor.class, "RotationArm");
        Extender = hardwareMap.get(DcMotor.class, "Extender");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        RetractionTouchSensor = hardwareMap.get(DigitalChannel.class, "RetractionTouchSensor");
        RotationArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Extender.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // The right motors needed to be reversed to run forward.
        Extender.setDirection(DcMotor.Direction.REVERSE);
        RotationArm.setDirection(DcMotor.Direction.REVERSE);
        Intake.setDirection(DcMotor.Direction.FORWARD);


        // set power of motors to 0
        Extender.setPower(0);
        RotationArm.setPower(0);
        Intake.setPower(0);

        Extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        LoweringLimitSwitch = hardwareMap.get(DigitalChannel.class, "LoweringLimitSwitch");
        RaisingLimitSwitch = hardwareMap.get(DigitalChannel.class, "RaisingLimitSwitch");
        RetractionTouchSensor = hardwareMap.get(DigitalChannel.class, "RetractionTouchSensor");


        LoweringLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
        RaisingLimitSwitch.setMode(DigitalChannel.Mode.INPUT);
        RetractionTouchSensor.setMode(DigitalChannel.Mode.INPUT);

    }

    public void initDriveMotors() {
        //naming the motors
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");

//
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // The right motors needed to be reversed to run forward.
        FrontLeft.setDirection(DcMotor.Direction.FORWARD);
        BackRight.setDirection(DcMotor.Direction.REVERSE);
        FrontRight.setDirection(DcMotor.Direction.REVERSE);
        BackLeft.setDirection(DcMotor.Direction.FORWARD);

        // resetEncoder();
        // set power of motors to 0
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);
    }
    public void initDriveMotorsAuto() {
        //naming the motors
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");

//
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        // The right motors needed to be reversed to run forward.
        FrontLeft.setDirection(DcMotor.Direction.FORWARD);
        BackRight.setDirection(DcMotor.Direction.REVERSE);
        FrontRight.setDirection(DcMotor.Direction.REVERSE);
        BackLeft.setDirection(DcMotor.Direction.FORWARD);

        // resetEncoder();
        // set power of motors to 0
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);
   }

    public void initIMUGyro() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = false;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        opmode.telemetry.addData("X", "Calibrating gyro...");
        opmode.telemetry.update();

        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


    }

//    public void initDistanceSensor() {
//        sensorRange = hardwareMap.get(DistanceSensor.class, "DistanceSensor");
//        sensorTimeOfFlight = (Rev2mDistanceSensor) sensorRange;
//    }
//
//    public double getDistance(DistanceUnit unit) {
//        //Available DistanceUnits are DistanceUnit.MM, DistanceUnit.CM, DistanceUnit.METER, DistanceUnit.INCH
//        return sensorRange.getDistance(unit);
//
//    }

    //initRobot initalizes motors and brings in the opmode and its hardware map
    public void initRobot(HardwareMap hwMap, LinearOpMode opmode) {
        setHardwareMap(hwMap);
        setOpMode(opmode);
        initDriveMotors();
        initArmMotors();
        initIMUGyro();

    }
    public void initRobotAuto(HardwareMap hwMap, LinearOpMode opmode) {
        setHardwareMap(hwMap);
        setOpMode(opmode);
        initDriveMotorsAuto();
        initArmMotors();
        initIMUGyro();

    }
    public void initRobotAuto(HardwareMap hwMap, OpMode opmode) {
        setHardwareMap(hwMap);
        setOpMode(opmode);
        initDriveMotorsAuto();
        initArmMotors();
        initIMUGyro();

    }
    public void initRobot(HardwareMap hwMap, OpMode opmode) {
        setHardwareMap(hwMap);
        setOpMode(opmode);
        initDriveMotors();
        initArmMotors();
        initIMUGyro();
    }

    public double getIMUBearing() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    //stackoverflow to the rescue. No math!!! https://stackoverflow.com/questions/7570808/how-do-i-calculate-the-difference-of-two-angle-measures/30887154
    public static double Angledistance(double alpha, double beta) {
        double phi = Math.abs(beta - alpha) % 360;       // This is either the distance or 360 - distance
        double distance = phi > 180 ? 360 - phi : phi;
        return distance;
    }

    public void gyroTurn(double turnAngle) {

        if (turnAngle < 0) {
            double startAngle = getIMUBearing();
            double targetAngle;
            double angle;

            targetAngle = startAngle + turnAngle;

            double error = Angledistance(startAngle, targetAngle);

            while (Math.abs(error) > 5) {
                angle = getIMUBearing();
                error = Angledistance(angle, targetAngle);

                opmode.telemetry.addData("imu gyro angle", angle);
                opmode.telemetry.addData("Target angle", targetAngle);
                opmode.telemetry.addData("Error = ", error);
                opmode.telemetry.update();

                if (error > 5) { //clockwise
                    setDrivePower(-1, 1, 1, -1, 0.2);
                } else if (error < -5) {//counterclockwise
                    setDrivePower(1, -1, -1, 1, 0.2);
                } else // stop
                    setDrivePower(0, 0, 0, 0);


            }
            setDrivePower(0, 0, 0, 0, 0);
        } else if (turnAngle > 0) {
            double startAngle = getIMUBearing();
            double targetAngle;
            double angle;

            targetAngle = startAngle + turnAngle;

            double error = Angledistance(startAngle, targetAngle);


            while (Math.abs(error) > 5) {
                angle = getIMUBearing();
                error = -Angledistance(angle, targetAngle);

                opmode.telemetry.addData("imu gyro angle", angle);
                opmode.telemetry.addData("Target angle", targetAngle);
                opmode.telemetry.addData("Error = ", error);
                opmode.telemetry.update();

                if (error > 5) { //clockwise
                    setDrivePower(-1, 1, 1, -1, 0.2);
                } else if (error < -5) {//counterclockwise
                    setDrivePower(1, -1, -1, 1, 0.2);
                } else // stop
                    setDrivePower(0, 0, 0, 0);


            }
            setDrivePower(0, 0, 0, 0, 0);
        }
    }

    //brings in hardware map
    public void setHardwareMap(HardwareMap hwMap) {
        this.hardwareMap = hwMap;
    }

    // brings in opmode
    public void setOpMode(LinearOpMode opmode) {
        this.opmode = opmode;
    }

    public void setOpMode(OpMode opmode) {
        this.opmode = opmode;
    }

    public void setMecanumPower(double drive, double strafe, double turn) {
        setMecanumPower(drive, strafe, turn, 1.0);
    }

    //sets mecanum power using drive, strafe, and turn
    public void setMecanumPower(double drive, double strafe, double turn, double maxspeed) {
        double frontleftPower;
        double frontrightPower;
        double backleftPower;
        double backrightPower;

//        double normalize = Math.max(Math.max(Math.abs(drive), Math.abs(turn)), Math.abs(strafe) );

//        drive = Math.pow(drive, 3);
//        turn = Math.pow(turn, 3);
//        strafe = Math.pow(strafe, 3);

//       if (normalize >= 1) {
//           drive = drive / normalize;
//           strafe = strafe / normalize;
//           turn = turn / normalize;
//       }

        frontleftPower = drive + strafe + turn;
        frontrightPower = drive - strafe - turn;
        backleftPower = drive - strafe + turn;
        backrightPower = drive + strafe - turn;
        double normalize = Math.max(Math.max(Math.max(Math.abs(frontleftPower), Math.abs(frontrightPower)), Math.abs(backleftPower)), Math.abs(backrightPower));

        if (normalize > 1) {
            frontleftPower /= normalize;
            backrightPower /= normalize;
            backleftPower /= normalize;
            frontrightPower /= normalize;
        }
        setDrivePower(frontrightPower, frontleftPower, backleftPower, backrightPower, maxspeed);


    }

    public void setTankPower(double right, double left, double strafe, double maxspeed) {

        double drive = (right + left) / 2;
        double turn = (left - right) / 2;
        setMecanumPower(drive, strafe, turn, maxspeed);
        ;

    }

    public void setTankPower(double right, double left, double strafe) {
        setTankPower(right, left, strafe, 1.0);
    }

    public void setDrivePower(double frontrightPower, double frontleftPower, double backleftPower, double backrightPower, double maxspeed) {
        FrontRight.setPower(frontrightPower * maxspeed);
        FrontLeft.setPower(frontleftPower * maxspeed);
        BackLeft.setPower(backleftPower * maxspeed);
        BackRight.setPower(backrightPower * maxspeed);
    }

    public void setDrivePower(double frontrightPower, double frontleftPower, double backleftPower, double backrightPower) {
        setDrivePower(frontrightPower, frontleftPower, backleftPower, backrightPower, 1.0);
    }

    public void resetEncoder() {
        FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void RetractAndResetArmEncoder() {
        Extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while (RetractionTouchSensor.getState() == true) {
            Extender.setPower(-0.5);
        }
        Extender.setPower(0);
        Extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    //to go backwards in driveForInches() you need to have negative inches NOT negative speed
    public void driveForInches(int inches, double speed) {

        //declares tick target
        //this is the amount of ticks each wheel respectively will move forward
        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int newFrontLeftTarget, newBackLeftTarget, newBackRightTarget, newFrontRightTarget;

        //converts the tick value to inches

        int newTicks = (int) Math.round(ticksPerInch * inches);

        //sets the target to the current encoder value plus the number of ticks you want to go forward
        newFrontLeftTarget = FrontLeft.getCurrentPosition() + newTicks;
        newBackLeftTarget = BackLeft.getCurrentPosition() + newTicks;
        newBackRightTarget = BackRight.getCurrentPosition() + newTicks;
        newFrontRightTarget = FrontRight.getCurrentPosition() + newTicks;

        //sets the encoder position to stop at the encoder value you want
        FrontLeft.setTargetPosition(newFrontLeftTarget);
        BackLeft.setTargetPosition(newBackLeftTarget);
        BackRight.setTargetPosition(newBackRightTarget);
        FrontRight.setTargetPosition(newFrontRightTarget);

        //sets the power to the speed declared above
        FrontLeft.setPower(Math.abs(speed));
        BackLeft.setPower(Math.abs(speed));
        BackRight.setPower(Math.abs(speed));
        FrontRight.setPower(Math.abs(speed));

        while (FrontLeft.isBusy() && FrontRight.isBusy() && BackLeft.isBusy() && BackRight.isBusy()) {
            //  opmode.telemetry.addData("")

        }

        //stops the motors
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);

        // Turn off RUN_TO_POSITION
        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    public void strafeForInches(int inches, double speed) {
         ElapsedTime RunTime = new ElapsedTime();
         RunTime.reset();
        //declares tick target
        //this is the amount of ticks each wheel respectively will move forward
        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        int newFrontLeftTarget, newBackLeftTarget, newBackRightTarget, newFrontRightTarget;

        //converts the tick value to inches

        int newTicks = (int) Math.round(ticksPerInch * inches);

        //sets the target to the current encoder value plus the number of ticks you want to go forward


        newFrontLeftTarget = FrontLeft.getCurrentPosition() - newTicks;
        newBackLeftTarget = BackLeft.getCurrentPosition() + newTicks;
        newBackRightTarget = BackRight.getCurrentPosition() - newTicks;
        newFrontRightTarget = FrontRight.getCurrentPosition() + newTicks;

        //sets the encoder position to stop at the encoder value you want
        FrontLeft.setTargetPosition(newFrontLeftTarget);
        BackLeft.setTargetPosition(newBackLeftTarget);
        BackRight.setTargetPosition(newBackRightTarget);
        FrontRight.setTargetPosition(newFrontRightTarget);

        //sets the power to the speed declared above
        FrontLeft.setPower(Math.abs(speed));
        BackLeft.setPower(Math.abs(speed));
        BackRight.setPower(Math.abs(speed));
        FrontRight.setPower(Math.abs(speed));

        //while ((FrontLeft.isBusy() || FrontRight.isBusy() || BackLeft.isBusy() || BackRight.isBusy())&& RunTime.time() < 5 ) {
        while (FrontLeft.isBusy() && FrontRight.isBusy() && BackLeft.isBusy() && BackRight.isBusy() ) {
            //  opmode.telemetry.addData("")

        }

        //stops the motors
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);

        // Turn off RUN_TO_POSITION
        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    public void RaiseRotationArm() {

        // set the digital channel to input.
        RaisingLimitSwitch.setMode(DigitalChannel.Mode.INPUT);

        while (RaisingLimitSwitch.getState() == true) {

            RotationArm.setPower(1);
            //telemetry.addData("Limit Switch", "Is Not close");
        }
        RotationArm.setPower(0);
        //telemetry.addData("Limit Switch", "Is close");
    }

    public void LowerRotationArm() {

        // set the digital channel to input.
        LoweringLimitSwitch.setMode(DigitalChannel.Mode.INPUT);

        while (LoweringLimitSwitch.getState() == true) {

            RotationArm.setPower(-1);
            //telemetry.addData("Limit Switch", "Is Not close");
        }
        RotationArm.setPower(0);
        //telemetry.addData("Limit Switch", "Is close");
    }

    public void RetractingArm() {

        // set the digital channel to input.
        RetractionTouchSensor.setMode(DigitalChannel.Mode.INPUT);

        while (RetractionTouchSensor.getState() == true) {

            Extender.setPower(-0.5);
            //telemetry.addData("Limit Switch", "Is Not close");
        }
        Extender.setPower(0);
        //telemetry.addData("Limit Switch", "Is close");
    }

    public void ExtendingArm() {
        extenderForTicks(raisingArmTicks, 1);
    }

    public void setRotationArmPower(double power) {
        RotationArm.setPower(power);
    }

    public void setExtenderArmPower(double power) {
        Extender.setPower(power);
    }

    public void setIntakePower(double power) {
        Intake.setPower(power);
    }

    public void SUPERUNSAFEEXTENDERCONTROLLER(double extender) {
        setExtenderArmPower(extender);
    }

    public void resetExtender() {
        Extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void resetRotator() {
        RotationArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RotationArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void extenderController(double extender) {

        extenderController(extender, false);
    }

    public void extenderController(double extender, boolean tele) {
        double deadZone = 0.05;
        extenderDirect direction; //this is motor direction NOT arm direction

        if(extender > deadZone){
            direction = extenderDirect.POS;
        }else if(extender < -deadZone){
            direction = extenderDirect.NEG;
        }else{
            direction = extenderDirect.STOP;
        }

        if(RetractionTouchSensor.getState() == false){//this might change based on motor direction
            if(Extender.getCurrentPosition() < 0){ //don't go down anymore
                extenderstate = extenderPrevent.POS;
            }else{
                extenderstate = extenderPrevent.NEG; //don't go up anymore
            }
        }

        if(extenderstate == extenderPrevent.NEG){
            if(direction == extenderDirect.NEG){
                setExtenderArmPower(0);
            }else{
                setExtenderArmPower(extender);
                extenderstate = extenderPrevent.NONE;
            }
        }else if(extenderstate == extenderPrevent.POS) {
            if (direction == extenderDirect.POS) {
                setExtenderArmPower(0);
            } else {
                setExtenderArmPower(extender);
                extenderstate = extenderPrevent.NONE;
            }
        }else{
            setExtenderArmPower(extender);
        }

//        if( direction == extenderDirect.POS){ //pushing up on stick
//            if(extenderstate != extenderPrevent.POS){ //you can go up
//                setExtenderArmPower(extender);
//                extenderstate = extenderPrevent.NONE;
//            }else{
//                setExtenderArmPower(0);
//            }
//        }else if (direction == extenderDirect.NEG){
//            if(extenderstate != extenderPrevent.NEG){
//                setExtenderArmPower(extender);
//                extenderstate = extenderPrevent.NONE;
//            }
//        }else{
//            setExtenderArmPower(0);
//        }
        if (tele == true) {
            //add telemetry
            opmode.telemetry.addData("Extender encoder", Extender.getCurrentPosition());
            opmode.telemetry.addData("Prevent Extender State", extenderstate);
            opmode.telemetry.addData("Direction Extender", direction);
            opmode.telemetry.update();
        }

    }

    public void rotatorController(double rotator, boolean tele) {
        double deadZone = 0.05;
        rotatorDirect direction;
        if (rotator > deadZone) {
            direction = rotatorDirect.UP;
        } else if (rotator < -deadZone) {
            direction = rotatorDirect.DOWN;
        } else {
            direction = rotatorDirect.STOP;
        }

        if (RaisingLimitSwitch.getState() == false) {
            rotatorState = rotatorPrevent.UP;
        }else if (LoweringLimitSwitch.getState() == false) {
            rotatorState = rotatorPrevent.DOWN;
        }
        else{
            rotatorState = rotatorPrevent.NONE;
        }

        if (direction == rotatorDirect.UP) { // if you want to go up
            if (rotatorState != rotatorPrevent.UP) { // if you can go up
                setRotationArmPower(rotator); //go up
                rotatorState = rotatorPrevent.NONE;
            } else { //can't go up but want to
                setRotationArmPower(0); //don't go up
            }
        } else if (direction == rotatorDirect.DOWN) { //if you want to go down
            if (rotatorState != rotatorPrevent.DOWN) { //and you can go down
                setRotationArmPower(rotator);// go down
                rotatorState = rotatorPrevent.NONE;
            } else {
                setRotationArmPower(0);
            }
        } else {
            setRotationArmPower(0);//must be in stop

        }
        if (tele == true) {
//          add telemetry
            opmode.telemetry.addData("Rotator encoder", RotationArm.getCurrentPosition());
            opmode.telemetry.addData("Prevent Rotator State", rotatorState);
            opmode.telemetry.update();
        }
    }

    public void rotatorController(double rotator) {
        rotatorController(rotator, false);
    }

    public void extenderForTicks(int ticks, double speed) {

        resetExtender();
        Extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int newExtenderTarget;
        int startPosition = Extender.getCurrentPosition();
        newExtenderTarget = Extender.getCurrentPosition() + ticks;
        Extender.setTargetPosition(newExtenderTarget);
        Extender.setPower(Math.abs(speed));
//isbusy did not work. Maybe this motor encoder does not return isbusy
        while (Math.abs(Extender.getCurrentPosition() - startPosition) < Math.abs(ticks)) {
//            opmode.telemetry.addData("Extender encoder", Extender.getCurrentPosition());
//            opmode.telemetry.addData("ticks so far", Extender.getCurrentPosition()-startPosition);
//            opmode.telemetry.update();
        }
        Extender.setPower(0);
        // Turn off RUN_TO_POSITION
        Extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    public void IntakeForTicks(int ticks, double speed) {

        //resetExtender();
        Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Intake.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int newIntakeTarget;
        int startPosition = Intake.getCurrentPosition();
        newIntakeTarget = Intake.getCurrentPosition() + ticks;
        Intake.setTargetPosition(newIntakeTarget);
        Intake.setPower(Math.abs(speed));
//isbusy did not work. Maybe this motor encoder does not return isbusy
        while (Math.abs(Intake.getCurrentPosition() - startPosition) < Math.abs(ticks)) {
//            opmode.telemetry.addData("Extender encoder", Extender.getCurrentPosition());
//            opmode.telemetry.addData("ticks so far", Extender.getCurrentPosition()-startPosition);
//            opmode.telemetry.update();
        }
        Intake.setPower(0);
        // Turn off RUN_TO_POSITION
        Intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    public void RotatorForTicks(int ticks, double speed) {

        resetRotator();
        RotationArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        int newExtenderTarget;
        int startPosition = RotationArm.getCurrentPosition();
        newExtenderTarget = RotationArm.getCurrentPosition() + ticks;
        RotationArm.setTargetPosition(newExtenderTarget);
        RotationArm.setPower(Math.abs(speed));
//isbusy did not work. Maybe this motor encoder does not return isbusy
        while (Math.abs(RotationArm.getCurrentPosition() - startPosition) < Math.abs(ticks)) {
            opmode.telemetry.addData("Rotator encoder", RotationArm.getCurrentPosition());
            opmode.telemetry.addData("ticks so far", RotationArm.getCurrentPosition() - startPosition);
            opmode.telemetry.update();
        }
        RotationArm.setPower(0);
        // Turn off RUN_TO_POSITION
        RotationArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }

    public void alignGold(MyGoldDetector detector) {
        //detector.enable();
        Point RectPoint = detector.getScreenPosition();
        if (RectPoint == null) {
            detector.disable();
            return;

        }
        double pointX = RectPoint.x;
        double pointY = RectPoint.y;

        double diffX = pointX - idealX; //if diffX = a + then go right
        double diffY = pointY - idealY; // if diff Y = a + then go forward
        double distance = Math.sqrt((diffX * diffX) + (diffY * diffY));
        double gainX = -1;
        double gainY = -1;
        while (diffX > idealX) {
            setMecanumPower(0, diffX * gainX, 0, 0.4);
            RectPoint = detector.getScreenPosition();// ideally we should check to make sure it's still on screen
            pointX = RectPoint.x;
            pointY = RectPoint.y;
            diffX = idealX - pointX; //if diffX = a + then go right
            diffY = idealY - pointY; // if diff Y = a + then go forward
            distance = Math.sqrt((diffX * diffX) + (diffY * diffY));
        }
        setMecanumPower(0, 0, 0, 0);
        //detector.disable();
    }

    public void findGold(MyGoldDetector detector) {
        //detector.enable();

        Point screenPos = detector.getScreenPosition();
        if (screenPos.x > 50 && screenPos.x < 430) {
            strafeForInches(9, 0.2);
            //  RotatorForTicks(-50,1);
            // setRotationArmPower(0);
            driveForInches(25, 0.4);
            goldSide = mineral.MIDDLE;
        } else {
            strafeForInches(21, 0.2);
            screenPos = detector.getScreenPosition();
            setMecanumPower(0, 0, 0, 0);
            if (screenPos.x > 50 && screenPos.x < 430) {
                // screenPos = detector.getScreenPosition();
                strafeForInches(8, 0.2);
                setMecanumPower(0, 0, 0, 0);
                driveForInches(36, 0.4);
                driveForInches(-6, 0.4);
                goldSide = mineral.RIGHT;
            } else {
                strafeForInches(-31, 0.2);
                driveForInches(30, 0.4);
                goldSide = mineral.LEFT;
            }
            opmode.telemetry.addData("gold detected", goldSide);
            opmode.telemetry.update();

        }

    }

    public void findGoldCrater(MyGoldDetector detector) {
        //detector.enable();

        Point screenPos = detector.getScreenPosition();
        if (screenPos.x > 50 && screenPos.x < 430) {
            strafeForInches(9, 0.2);
            //  RotatorForTicks(-50,1);
            // setRotationArmPower(0);
            driveForInches(12, 0.4);
            goldSide = mineral.MIDDLE;
        } else {
            strafeForInches(21, 0.2);
            screenPos = detector.getScreenPosition();
            setMecanumPower(0, 0, 0, 0);
            if (screenPos.x > 50 && screenPos.x < 430) {
                // screenPos = detector.getScreenPosition();
                strafeForInches(8, 0.2);
                setMecanumPower(0, 0, 0, 0);
                driveForInches(16, 0.4);
                //driveForInches(-6,0.4);
                goldSide = mineral.RIGHT;
            } else {
                strafeForInches(-31, 0.2);
                driveForInches(16, 0.4);
                goldSide = mineral.LEFT;
            }
            opmode.telemetry.addData("gold detected", goldSide);
            opmode.telemetry.update();


        }

    }

    public void depotClaimFromDepot() {
        if (goldSide == mineral.LEFT) {
            gyroTurn(-20);
            driveForInches(5, 0.4);
            LowerRotationArm();
            // RotatorForTicks(-1300,1);
        }
        if (goldSide == mineral.RIGHT) {
            gyroTurn(20);
            LowerRotationArm();
            //RotatorForTicks(-1300,1);
        }
        if (goldSide == mineral.MIDDLE) {
            LowerRotationArm();
            //RotatorForTicks(-1300,1);

        }
    }

    public void goToCraterFromDepot() {

        // initIMUGyro();
        //goldSide = mineral.LEFT;
        if (goldSide == mineral.LEFT) {
            double target1 = getIMUBearing();
            gyroTurn(150);

//                driveForInches(12,0.4);
//                gyroTurn(90);
            driveForInches(50, 0.6); //needs to be 48ish
            LowerRotationArm();
            // RotatorForTicks(-1300,1);
        }
        if (goldSide == mineral.RIGHT) {

//                gyroTurn(45);m kn
//                // driveForInches(82,0.4);
            gyroTurn(15);
            driveForInches(20, 0.4);
            double target1 = getIMUBearing();
            gyroTurn(80);
            driveForInches(76, 0.6);//needs to be 48ish
            LowerRotationArm();
            //RotatorForTicks(-1300,1);


        }
        if (goldSide == mineral.MIDDLE) {

            gyroTurn(45);
            // driveForInches(72,0.4);
            driveForInches(15, 0.4);
             double target1 = getIMUBearing();
            gyroTurn(80);
            driveForInches(50, 0.6);//needs to be 48ish

            LowerRotationArm();
            //RotatorForTicks(-1300,1);
        }
        opmode.telemetry.addData("goldSide ", goldSide);
        opmode.telemetry.update();

    }

    public void goToSameCraterFromDepot(){
        driveForInches(-20,0.4);
        gyroTurn(-182);
        driveForInches(24,0.4);
        LowerRotationArm();
    }

    public void goToDepotFromCrater() {

        // initIMUGyro();
        //goldSide = mineral.LEFT;
        driveForInches(-12, 0.4);

        if (goldSide == mineral.LEFT) {
            gyroTurn(45);
            driveForInches(20, 0.6);

        }
        if (goldSide == mineral.RIGHT) {
            gyroTurn(90);
            driveForInches(30, 0.4);
            gyroTurn(-45);
            driveForInches(20, 0.6);
        }
        if (goldSide == mineral.MIDDLE) {
            gyroTurn(90);
            driveForInches(17, 1);
            gyroTurn(-45);
            driveForInches(25, 0.6);

        }

        gyroTurn(80);
        driveForInches(45, 0.4);
        LowerRotationArm();

        opmode.telemetry.addData("goldSide ", goldSide);
        opmode.telemetry.update();
    }

    public void goToSecondSampling() {

        // initIMUGyro();
        //goldSide = mineral.LEFT;
        driveForInches(-8, 0.4);

        if (goldSide == mineral.LEFT) {
            gyroTurn(45);
            driveForInches(20, 0.4);

        }
        if (goldSide == mineral.RIGHT) {
            gyroTurn(90);
            driveForInches(25, 0.4);
            gyroTurn(-45);
            driveForInches(25, 0.4);
        }
        if (goldSide == mineral.MIDDLE) {
            gyroTurn(90);
            driveForInches(10, 1);
            gyroTurn(-45);
            driveForInches(25, 0.4);

        }

        gyroTurn(105);
        driveForInches(20,0.4);
        gyroTurn(-90);


//        public void lineToGold(MyGoldDetector detector) {
//                //detector.enable();
//               // double Xnull = 0;
//            FrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//            FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//                Point screenPos = detector.getScreenPosition();
//                //resetEncoder();
//
//                    while (screenPos.x < 250 || screenPos.x > 430) {
//                        //setMecanumPower(0,0,1,-0.3);
//                        setMecanumPower(0, 1, 0, -0.2);
//                        screenPos = detector.getScreenPosition();
//                        opmode.telemetry.addData("Strafe encoder", FrontLeft.getCurrentPosition());
//                        opmode.telemetry.update();
//                        if(screenPos.x > 250 && screenPos.x < 430){
//                            opmode.telemetry.addData("Strafe encoder", FrontLeft.getCurrentPosition());
//                            opmode.telemetry.update();
//                            break;
//
//
//                        }
//
//                    }
//                    setMecanumPower(0, 0, 0, 0);
//
//
//            }


        //  driveForInches(4,0.5);


        //setMecanumPower(0,0,0,0);


//    public void extenderForInches(int inches, double speed) {
//        //declares tick target
//        //this is the amount of ticks each wheel respectively will move forward
//        Extender.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//
//
//        int newExtenderTarget;
//
//        //converts the tick value to inches
//
//        int newTicks = (int) Math.round(extenderTicksPerInch * inches);
//
//        //sets the target to the current encoder value plus the number of ticks you want to go forward
//        newExtenderTarget = FrontLeft.getCurrentPosition() + newTicks;
//
//
//        //sets the encoder position to stop at the encoder value you want
//        Extender.setTargetPosition(newExtenderTarget);
//
//
//        //sets the power to the speed declared above
//        Extender.setPower(Math.abs(speed));
//
//
//        while (Extender.isBusy()) {
//            //  opmode.telemetry.addData("")
//
//        }
//
//        //stops the motors
//        Extender.setPower(0);
//
//
//        // Turn off RUN_TO_POSITION
//        Extender.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//
//    }

    }
    public void gyroPID(double inches, double target){

       // double current = getIMUBearing();

        //declares tick target
        //this is the amount of ticks each wheel respectively will move forward
        FrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        int newFrontLeftTarget, newBackLeftTarget, newBackRightTarget, newFrontRightTarget;

        //converts the tick value to inches

        int newTicks = (int) Math.round(ticksPerInch * inches);

        //sets the target to the current encoder value plus the number of ticks you want to go forward
        newFrontLeftTarget = FrontLeft.getCurrentPosition() + newTicks;
        newBackLeftTarget = BackLeft.getCurrentPosition() + newTicks;
        newBackRightTarget = BackRight.getCurrentPosition() + newTicks;
        newFrontRightTarget = FrontRight.getCurrentPosition() + newTicks;

        //sets the encoder position to stop at the encoder value you want
        FrontLeft.setTargetPosition(newFrontLeftTarget);
        BackLeft.setTargetPosition(newBackLeftTarget);
        BackRight.setTargetPosition(newBackRightTarget);
        FrontRight.setTargetPosition(newFrontRightTarget);

//
//        //sets the power to the speed declared above
//        FrontLeft.setPower(Math.abs(speed));
//        BackLeft.setPower(Math.abs(speed));
//        BackRight.setPower(Math.abs(speed));
//        FrontRight.setPower(Math.abs(speed))
// ;
       double max = 5;
        while (FrontLeft.isBusy() && FrontRight.isBusy() && BackLeft.isBusy() && BackRight.isBusy()) {
            //  opmode.telemetry.addData("")
            double bearing = getIMUBearing();
            double diff = bearing - target;
            double power = diff/max;
            setMecanumPower(Math.abs(1-power),0, power,0.5);
        }

        //stops the motors
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);

        // Turn off RUN_TO_POSITION
        FrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




    }
}