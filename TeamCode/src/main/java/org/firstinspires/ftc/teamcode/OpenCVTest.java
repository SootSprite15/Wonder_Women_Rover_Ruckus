package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.disnodeteam.dogecv.OpenCVPipeline;

import org.opencv.core.Mat;

@Autonomous
public class OpenCVTest extends LinearOpMode {
    WonderWomenRobot robot = new WonderWomenRobot();

    OpenCVPipeline pipeline = new OpenCVPipeline() {
        @Override
        public Mat processFrame(Mat rgba, Mat gray) {
            return null;
        }
    };

    @Override
    public void runOpMode() throws InterruptedException {
        // robot.initRobot(hardwareMap, this);
        pipeline.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
    pipeline.enable();
        //shows camera on phone
        while (opModeIsActive()) {

        }
    }
}


