package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous
public class AutoTest extends LinearOpMode {
    WonderWomenRobot robot = new WonderWomenRobot();

    @Override
    public void runOpMode() throws InterruptedException{
       robot.initRobot(hardwareMap, this);

       robot.driveForInches(12,0.5);
       //positive angle = left turn
       robot.gyroTurn(90);
       robot.driveForInches(12,0.5);
       robot.gyroTurn(45);
       robot.driveForInches(12,0.5);

    }
}


