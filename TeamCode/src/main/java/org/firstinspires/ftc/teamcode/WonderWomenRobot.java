package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

//not an opmode
//to be used by every opmode
//In this file we define the robot and its motions. Opmodes then use these motions.
public class WonderWomenRobot {

    //OpMode members
    private DcMotor FrontRight = null;
    private DcMotor FrontLeft = null;
    private DcMotor BackRight = null;
    private DcMotor BackLeft = null;
    private DcMotor HexMotor = null;
    private HardwareMap hardwareMap = null;
    private LinearOpMode opmode = null;
    private BNO055IMU imu;
    private Orientation angles;

    //Initialize drive motors
    public void initDriveMotors(){
    //naming the motors
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");
        HexMotor = hardwareMap.get(DcMotor.class, "HexMotor");
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

        // set power of motors to 0
        FrontLeft.setPower(0);
        FrontRight.setPower(0);
        BackLeft.setPower(0);
        BackRight.setPower(0);
    }
    public void initIMUGyro(){
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








//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
//        parameters.loggingEnabled = true;
//        parameters.loggingTag = "IMU";
//        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//        imu.initialize(parameters);


    }
//    public void calibIMUGyro(){
//        while(!LinearOpMode.isStopRequested() && !imu.isGyroCalibrated()){
//            LinearOpMode.sleep(50);
//            LinearOpMode.idle();
//        }
    //}

    //initRobot initalizes motors and brings in the opmode and its hardware map
    public void initRobot(HardwareMap hwMap, LinearOpMode opmode){
        setHardwareMap(hwMap);
        setOpMode(opmode);
        initDriveMotors();
        initIMUGyro();


    }
//    public void initRobot(HardwareMap hwMap){
//        setHardwareMap(hwMap);
//        setOpMode(OpMode);
//
//    }
    public double getIMUBearing(){
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
        }else if(turnAngle > 0){
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
    public void setHardwareMap(HardwareMap hwMap){
        this.hardwareMap = hwMap;
    }
    //brings in opmode
    public void setOpMode(LinearOpMode opmode){
        this.opmode = opmode;
    }
    public void setMecanumPower(double drive, double strafe, double turn){
        setMecanumPower(drive, strafe, turn, 1.0);
    }
//sets mecanum power using drive, strafe, and turn
    public void setMecanumPower(double drive, double strafe, double turn, double maxspeed){
        double frontleftPower;
        double frontrightPower;
        double backleftPower;
        double backrightPower;

        double normalize = Math.max(Math.max(Math.abs(drive), Math.abs(turn)), Math.abs(strafe) );

//        drive = Math.pow(drive, 3);
//        turn = Math.pow(turn, 3);
//        strafe = Math.pow(strafe, 3);

       if (normalize >= 1) {
           drive = drive / normalize;
           strafe = strafe / normalize;
           turn = turn / normalize;
       }

        frontleftPower  = drive + strafe + turn;
        frontrightPower = drive - strafe - turn;
        backleftPower   = drive - strafe + turn;
        backrightPower  = drive + strafe - turn;



        setDrivePower(frontrightPower, frontleftPower, backleftPower, backrightPower, maxspeed);

    }

    public void setTankPower(double right, double left, double strafe, double maxspeed){

        double drive = (right + left) / 2;
        double turn = (left - right) /2 ;
        setMecanumPower(drive, strafe, turn, maxspeed);;

    }
    public void setTankPower(double right, double left, double strafe){
        setTankPower(right, left, strafe,1.0);
    }

    public void setDrivePower(double frontrightPower, double frontleftPower, double backleftPower, double backrightPower, double maxspeed){
        FrontRight.setPower(frontrightPower * maxspeed);
        FrontLeft.setPower(frontleftPower * maxspeed);
        BackLeft.setPower(backleftPower * maxspeed);
        BackRight.setPower(backrightPower * maxspeed);
    }
    public void setDrivePower(double frontrightPower, double frontleftPower, double backleftPower, double backrightPower){
        setDrivePower(frontrightPower, frontleftPower, backleftPower, backrightPower, 1.0);
    }
    public void setHexMotorPower(){
        HexMotor.setPower(1);
    }
}





