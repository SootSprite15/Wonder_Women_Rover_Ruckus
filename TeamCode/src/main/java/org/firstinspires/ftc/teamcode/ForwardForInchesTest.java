package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Autonomous
public class ForwardForInchesTest extends LinearOpMode {
    WonderWomenRobot robot = new WonderWomenRobot();

    @Override
    public void runOpMode() throws InterruptedException{
       robot.initRobot(hardwareMap, this);

       robot.driveForInches(5,0.5);
       robot.driveForInches(-5,0.5);
    }
}


