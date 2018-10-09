package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
//not an opmode
//to be used by every opmode
//In this file we define the robot and its motions. Opmodes then use these motions.
public class WonderWomenRobot{

    //OpMode members
    private DcMotor FrontRight = null;
    private DcMotor FrontLeft = null;
    private DcMotor BackRight = null;
    private DcMotor BackLeft = null;
    private HardwareMap hardwareMap = null;
    private LinearOpMode opmode = null;

    //Initialize drive motors
    public void initDriveMotors(){
    //naming the motors
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");

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

    //initRobot initalizes motors and brings in the opmode and its hardware map
    public void initRobot(HardwareMap hwMap, LinearOpMode opmode){
        setHardwareMap(hwMap);
        setOpMode(opmode);
        initDriveMotors();


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

        drive = Math.pow(drive, 3);
        turn = Math.pow(turn, 3);
        strafe = Math.pow(strafe, 3);

       if (normalize >= 1) {
           drive = drive / normalize;
           strafe = strafe / normalize;
           turn = turn / normalize;
       }

        frontleftPower  = drive + strafe + turn;
        frontrightPower = drive - strafe - turn;
        backleftPower   = drive - strafe + turn;
        backrightPower  = drive + strafe - turn;



        FrontRight.setPower(frontrightPower * maxspeed);
        FrontLeft.setPower(frontleftPower * maxspeed);
        BackLeft.setPower(backleftPower * maxspeed);
        BackRight.setPower(backrightPower * maxspeed);

    }
    public void setTankPower(double right, double left, double strafe, double maxspeed){

        double drive = (right + left) / 2;
        double turn = (left - right) /2 ;
        setMecanumPower(drive, strafe, turn, maxspeed);;

    }
    public void setTankPower(double right, double left, double strafe){
        setTankPower(right, left, strafe,1.0);
    }


    }





