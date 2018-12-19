package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


@Disabled
@Autonomous
public class GyroTurnWithClass extends LinearOpMode {
    WonderWomenRobot robot = new WonderWomenRobot();

    @Override
    public void runOpMode() throws InterruptedException{
        robot.initRobot(hardwareMap, this);
        while (!isStarted()) {
            telemetry.addData("0", "robot ready");

        }
        robot.gyroTurn(90);
        robot.gyroTurn(-90);

    }
}


//robot.initRobot(hardwareMap, this);
