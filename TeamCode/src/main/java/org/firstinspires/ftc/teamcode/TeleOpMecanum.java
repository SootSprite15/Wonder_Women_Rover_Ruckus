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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

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
@TeleOp
@Disabled
public class TeleOpMecanum extends LinearOpMode {
    WonderWomenRobot robot = new WonderWomenRobot();
    @Override
    public void runOpMode() {

        robot.initRobot(hardwareMap, this);

        // wait for the start button to be pressed.
        waitForStart();

        // while the op mode is active, loop and read the light levels.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {
            double turn = gamepad1.right_stick_x;
            double drive = gamepad1.left_stick_y;
            double rotator = gamepad2.left_stick_y;
            double extender = -gamepad2.right_stick_y;
            double strafe = 0;
            double maxspeed = 1;
            //strafes with triggers
            if(gamepad1.right_trigger>0) {
                strafe = gamepad1.right_trigger;
            }else if(gamepad1.left_trigger>0){
                strafe = -gamepad1.left_trigger;
            }else{
                strafe =0;
            }

            //enter low speed mode when bumpers are pressed
            if(gamepad1.right_bumper || gamepad1.left_bumper){
                maxspeed = 0.5;
            }

            //set power to drive
            robot.setMecanumPower(drive, strafe, turn, maxspeed);



            //MANIPULATOR CONTROLS
            if(gamepad2.a && gamepad2.b){
                robot.resetExtender();
            }

            robot.rotatorController(rotator, false);
            robot.extenderController(extender, true);




            if(gamepad2.right_bumper){
                robot.setIntakePower(1);
            }
            else if(gamepad2.left_bumper){
                robot.setIntakePower(-1);
            }else{
                robot.setIntakePower(0);
            }

        }
    }
}
