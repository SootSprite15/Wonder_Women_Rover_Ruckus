package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


@Autonomous
public class AutoTest extends LinearOpMode {
    WonderWomenRobot robot = new WonderWomenRobot();




    @Override
    public void runOpMode() {
       robot.initRobot(hardwareMap, this);

       waitForStart();

       while (opModeIsActive()){
           robot.driveForInches(-2,0.1);
//           //positive angle = left turn
//           robot.gyroTurn(-45);
//           robot.driveForInches(2,0.1);
//           robot.gyroTurn(45);
//           robot.driveForInches(2,0.1);
      }


    }
}


