
        package org.firstinspires.ftc.teamcode;

        import com.qualcomm.robotcore.eventloop.opmode.Disabled;
        import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
        import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
        import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.WonderWomenRobot;
@Disabled
@TeleOp
public class TrialTeleOp extends LinearOpMode {
    WonderWomenRobot robot = new WonderWomenRobot();

    @Override
    public void runOpMode() {
        //initialize robot using hardware map and opmode
        robot.initRobot(hardwareMap, this);
        gamepad1.setJoystickDeadzone((float) 0.05);
        //gamepad2.setJoystickDeadzone((float) 0.05);
        waitForStart();

        while(opModeIsActive()){
            //set gamepad controls
            double turn = gamepad1.right_stick_x;
            double drive = gamepad1.left_stick_y;
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
            sleep(1);
        }
    }
}
