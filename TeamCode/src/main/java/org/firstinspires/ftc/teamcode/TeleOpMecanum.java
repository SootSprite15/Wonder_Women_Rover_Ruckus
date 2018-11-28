package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp
public class TeleOpMecanum extends LinearOpMode{
    WonderWomenRobot robot = new WonderWomenRobot();

    @Override
    public void runOpMode() {
        //initialize robot using hardware map and opmode
        robot.initRobot(hardwareMap, this);
        gamepad1.setJoystickDeadzone((float) 0.05);
        gamepad2.setJoystickDeadzone((float) 0.05);
        waitForStart();

        while(opModeIsActive()){
            //DRIVE CONTROLS
            //set gamepad controls
            double turn = gamepad1.right_stick_x;
            double drive = gamepad1.left_stick_y;
            double rotator = -gamepad2.left_stick_y;
            double extender = gamepad2.right_stick_y;
            double strafe = 0;
            double maxspeed = 1;
            //strafes with triggers
            if(gamepad1.right_trigger>0) {
                strafe = gamepad1.right_trigger;
            }else if(gamepad1.left_trigger>0){
                strafe = -gamepad1.left_trigger;
            }else{
                strafe =0;
            }
            //enter low speed mode when bumpers are pressed
            if(gamepad1.right_bumper || gamepad1.left_bumper){
                maxspeed = 0.5;
            }
            //set power to drive
            robot.setMecanumPower(drive, strafe, turn, maxspeed);



            //MANIPULATOR CONTROLS


            robot.rotatorController(rotator);
            robot.extenderController(extender);

            if(gamepad2.right_bumper){
                robot.setIntakePower(1);
            }
            else if(gamepad2.left_bumper){
                robot.setIntakePower(-1);
            }else{
                robot.setIntakePower(0);
            }


            sleep(1);
        }
    }
}
