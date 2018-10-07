package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp
public class TeleOp_WonderWomenRobotTest extends LinearOpMode{
    WonderWomenRobot robot = new WonderWomenRobot();

    @Override
    public void runOpMode() {
        //initialize robot using hardware map and opmode
        robot.initRobot(hardwareMap, this);
        waitForStart();
        while(opModeIsActive()){
            //set gamepad controls
            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            double strafe = gamepad1.left_stick_x;
            //set power to drive
            robot.setMecanumPower(drive, strafe, turn);
        }
    }
}
