package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;



@Autonomous
public class GyroTurnWithClass extends LinearOpMode {
    WonderWomenRobot robot = new WonderWomenRobot();

    @Override
    public void runOpMode() throws InterruptedException{
        robot.initRobot(hardwareMap, this);
        while (!isStarted()) {
            telemetry.addData("0", "robot ready");

        }
//        imuAngle turnAngle = new imuAngle();
        robot.gyroTurn(280);


    }
}

