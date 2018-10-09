package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class Turning extends LinearOpMode {
    WonderWomenRobot robot = new WonderWomenRobot();

    @Override
    public void runOpMode() {

        robot.initRobot(hardwareMap, this);
        waitForStart();
        while (opModeIsActive()) {
            robot.setDrivePower(1, -1, -1, 1, 0.25);

        }
    }
}
