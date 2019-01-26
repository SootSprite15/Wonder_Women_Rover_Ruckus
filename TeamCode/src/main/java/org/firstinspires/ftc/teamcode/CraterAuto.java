/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.opencv.core.Size;

//import com.qualcomm.robotcore.eventloop.opmode.Disabled;

/*
 * This is an example LinearOpMode that shows how to use
 * a REV Robotics Touch Sensor.
 *
 * It assumes that the touch sensor is configured with a name of "digitalTouch".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 */
@Autonomous
public class CraterAuto extends LinearOpMode {
    WonderWomenRobot robot = new WonderWomenRobot();
    private MyGoldDetector detector;
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "DogeCV 2018.0 - Gold SilverDetector Example");

        // Setup detector
        detector = new MyGoldDetector(); // Create detector
        detector.setAdjustedSize(new Size(480, 270)); // Set detector size
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance()); // Initialize detector with app context and camera
        detector.useDefaults(); // Set default detector settings
        // Optional tuning

        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.PERFECT_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005;

        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;

        robot.initRobot(hardwareMap, this);
        detector.enable();
       // robot.initRobot(hardwareMap, this);

        // wait for the start button to be pressed.
        waitForStart();

        // while the op mode is active, loop and read the light levels.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.

//
       // robot.extenderForTicks(15000,1); //needs to raise 23 inches

        robot.setExtenderArmPower(1);
        sleep(6000);
        robot.setExtenderArmPower(0);

        robot.strafeForInches(-7, 1); //strafe off lander
        detector.enable();
        robot.setMecanumPower(0,0,0,0);
       // robot.driveForInches(20,0.2);//forward to avoid hitting the lander was 16
        robot.driveForInches(23,0.2);
        robot.strafeForInches(-7,1);//strafe to line up to middle mineral was -6
        robot.findGoldCrater(detector);//find the gold mineral and pushes out

        telemetry.addData("Status", "found gold");
        telemetry.update();

       robot.goToDepotFromCraterGyro(); //goes to depot
        telemetry.addData("Status", "went to depot");
        telemetry.update();
        robot.setIntakePower(-1); //claims depot
        sleep(1600);
        robot.setIntakePower(0);
//         robot.RaiseRotationArm();
        robot.RotatorForTicks(1200,1); //raises arm
        telemetry.addData("Status", "arm raised");
        telemetry.update();
        robot.goToSameCraterFromDepotGyro(); //goes to crater
        telemetry.addData("Status", "at crater");
       // robot.extenderForTicks(15000,1); //extends arm to prepare for TeleOp

        robot.setExtenderArmPower(1);
        sleep(3000);
        robot.setExtenderArmPower(0);

        telemetry.update();
        sleep(1000);
        detector.disable();



        while(opModeIsActive()){
            telemetry.addData("Screen Position", detector.getScreenPosition());
            telemetry.update();
        }
    }
}
