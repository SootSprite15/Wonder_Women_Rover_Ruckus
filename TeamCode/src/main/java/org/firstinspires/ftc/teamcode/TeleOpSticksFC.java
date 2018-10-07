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

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

//strafing with sticks: fine controls


@TeleOp(name="TeleOp Sticks FC", group="Teleop")

public class TeleOpSticksFC extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    public DcMotor FrontRight = null;
    public DcMotor FrontLeft = null;
    public DcMotor BackRight = null;
    public DcMotor BackLeft = null;

    double drive = 0;
    double turn = 0;

    @Override
    public void runOpMode() {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        FrontLeft = hardwareMap.get(DcMotor.class, "FrontLeft");
        FrontRight = hardwareMap.get(DcMotor.class, "FrontRight");
        BackLeft = hardwareMap.get(DcMotor.class, "BackLeft");
        BackRight = hardwareMap.get(DcMotor.class, "BackRight");

        // Most robots need the motor on one side to be reversed to drive forward
      //   Reverse the motor that runs backwards when connected directly to the battery
        FrontLeft.setDirection(DcMotor.Direction.REVERSE);
        BackLeft.setDirection(DcMotor.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            FrontLeft.setPower(0);
            FrontRight.setPower(0);
            BackLeft.setPower(0);
            BackRight.setPower(0);
            //double strafe = 0;
            double turn = -gamepad1.left_stick_y;
            double drive = gamepad1.right_stick_x;
            double strafe = gamepad1.left_stick_x;
           drive = Math.pow(drive, 3);
           turn = Math.pow(turn, 3);
           strafe = Math.pow(strafe, 3);

////get rid of if statement if using sticks to control strafing
//            if (gamepad1.right_bumper) {
//                FrontRight.setPower(1);
//                FrontLeft.setPower(1);
//                BackLeft.setPower(-1);
//                BackRight.setPower(-1);
//                strafe = -1;
//            } else if (gamepad1.left_bumper) {
//                FrontRight.setPower(-1);
//                FrontLeft.setPower(-1);
//                BackLeft.setPower(1);
//                BackRight.setPower(1);
//                 strafe = -1;
//            }


                // Setup a variable for each drive wheel to save power level for telemetry
                double frontleftPower;
                double frontrightPower;
                double backleftPower;
                double backrightPower;
                double r = turn;
                double y = strafe;
                double x = drive;

                double normalize = Math.max(Math.max(Math.abs(x), Math.abs(r)), Math.abs(y) );

            if (normalize >= 1) {
                x = x / normalize;
                y = y / normalize;
                r = r / normalize;
            }

                frontleftPower = x + y +r;
                frontrightPower = x - y -r;
                backleftPower = x -y +r;
                backrightPower = x + y -r;



                // Send calculated power to wheels
                FrontRight.setPower(frontrightPower);
                FrontLeft.setPower(frontleftPower);
                BackLeft.setPower(backleftPower);
                BackRight.setPower(backrightPower);



                // Show the elapsed game time and wheel power.
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.addData("Motors", "left (%.2f), right (%.2f)", frontleftPower, frontrightPower);
                telemetry.update();

        }
    }
}

