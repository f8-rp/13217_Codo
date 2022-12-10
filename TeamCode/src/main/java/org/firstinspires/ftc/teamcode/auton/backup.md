/*
* Copyright (c) 2021 OpenFTC Team
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
  */

package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.ArrayList;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;



@Autonomous
public class AutonTemplate extends LinearOpMode
{
//INTRODUCE VARIABLES HERE
DcMotor leftFrontDrive = null;
DcMotor leftBackDrive = null;
DcMotor rightFrontDrive = null;
DcMotor rightBackDrive = null;
DcMotor slide = null;
OpenCvCamera camera;
AprilTagDetectionPipeline aprilTagDetectionPipeline;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;



    int LEFT = 17;
    int MIDDLE = 18;
    int RIGHT = 19;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);


        //HARDWARE MAPPING HERE etc.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "front_left");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "back_left");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "front_right");
        rightBackDrive = hardwareMap.get(DcMotor.class, "back_right");
        slide = hardwareMap.get(DcMotor.class, "slide");
        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            telemetry.update();
            sleep(20);
        }





        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available.");
            telemetry.update();
        }

        //PUT AUTON CODE HERE (DRIVER PRESSED THE PLAY BUTTON!)

        // 2 squares, 0.5 speed 1650 time
        // move into middle of square

        slide.setPower(-1);
        sleep(1000);
        forward(0.5,70, true);
        sleep(500);

        //go left 1 square and forward 1 square
        left(0.5,1050, true);
        reset();
        sleep(700);
        //line up with pole and bring slide up
        forward(0.5,880, true);
        back(1,1, true);
        right(1,1, true);
        slideUp();
        forward(0.5, 50, true);
        sleep(750);
        //drop cone
        back(0.5,50, true);
        reset();
        slideDown();


        //drive into parking zone based on signal sleeve ID
        if(tagOfInterest.id==RIGHT){
            left(0.5,600, false);
            back(0.5,1850, false);
            reset();
        }
        if(tagOfInterest.id==MIDDLE){
            left(0.5,600, false);
            back(0.5,1850, false);
            right(0.5,1000, false);
            reset();
        }
        if(tagOfInterest.id==LEFT){
            right(0.5,1650, false);
            back(0.5,1650, false);
            reset();
        }
        forward(0.5,500, false);
        sleep(8000);
        left(0.5,1000, false);


    }

    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    void forward(double speed, long time, boolean e){
        leftFrontDrive.setPower(speed);
        rightFrontDrive.setPower(speed);
        leftBackDrive.setPower(speed);
        rightBackDrive.setPower(speed);
        if(e) {
            slide.setPower(-0.1);
        }
        sleep(time);
        reset();
        slide.setPower(-0.1);
    }

    void reset(){
        leftFrontDrive.setPower(0);
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
    }

    void right(double speed, long time, boolean e){
        leftFrontDrive.setPower(speed);
        rightFrontDrive.setPower(-speed);
        leftBackDrive.setPower(-speed);
        rightBackDrive.setPower(speed);
        if(e) {
            slide.setPower(-0.1);
        }
        sleep(time);
        reset();
        slide.setPower(-0.1);
    }
    void left(double speed, long time, boolean e){
        leftFrontDrive.setPower(-speed);
        rightFrontDrive.setPower(speed);
        leftBackDrive.setPower(speed);
        rightBackDrive.setPower(-speed);
        if(e) {
            slide.setPower(-0.1);
        }
        sleep(time);
        reset();
        slide.setPower(-0.1);
    }
    void back(double speed, long time, boolean e){
        leftFrontDrive.setPower(-speed);
        rightFrontDrive.setPower(-speed);
        leftBackDrive.setPower(-speed);
        rightBackDrive.setPower(-speed);
        if(e) {
            slide.setPower(-0.1);
        }
        sleep(time);
        reset();
        slide.setPower(-0.1);
    }
    void turnRightNinety(double speed){
        leftFrontDrive.setPower(speed);
        rightFrontDrive.setPower(-speed);
        leftBackDrive.setPower(speed);
        rightBackDrive.setPower(-speed);
        sleep(780);
        reset();
    }
    void turnLeftNinety(double speed){
        leftFrontDrive.setPower(-speed);
        rightFrontDrive.setPower(speed);
        leftBackDrive.setPower(-speed);
        rightBackDrive.setPower(speed);
        sleep(780);
        reset();
    }
    void slideUp(){
        slide.setPower(-1);
        sleep(2000);
        slide.setPower(-0.1);
        /*//release cone*/

    }
    void slideDown(){
        slide.setPower(0);
    }
}


