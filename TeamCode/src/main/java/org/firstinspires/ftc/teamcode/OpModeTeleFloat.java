package org.firstinspires.ftc.teamcode;//package org.firstinspires.ftc.teamcode;


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



import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp

public class OpModeTeleFloat extends OpMode {
    WonderWomenRobot robot = new WonderWomenRobot();


    @Override
    public void init() {
        robot.initRobotTeleOp(hardwareMap, this);

    }
    /*
     * Code to run REPEATEDLY when the driver hits INIT
     */
    @Override
    public void init_loop() {

    }
    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

    }
        /*
         * Code to run REPEATEDLY when the driver hits PLAY
         */
        @Override
        public void loop () {
            double turn = gamepad1.right_stick_x;
            double drive = gamepad1.left_stick_y;
            double rotator = -gamepad2.left_stick_y;
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

        telemetry.addData("Diana believes in you!", "Go Jessica and Amelia!");

            telemetry.update();
//sleep(1);
        }
        /*
         * Code to run ONCE after the driver hits STOP
         */
        @Override
        public void stop () { }

    }
