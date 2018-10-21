package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous
public class RunEncoderTest extends LinearOpMode{
    WonderWomenRobot robot = new WonderWomenRobot();

    @Override
    public void runOpMode() throws InterruptedException{
        robot.initRobot(hardwareMap, this);
        while (!isStarted()) {
            telemetry.addData("0", "robot ready");

        }
        robot.driveByTicks(100000, 0.5 );

    }
}
