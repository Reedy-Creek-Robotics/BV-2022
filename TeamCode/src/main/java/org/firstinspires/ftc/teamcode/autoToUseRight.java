package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_TO_POSITION;
import static com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_USING_ENCODER;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(name="Competition Auto Right",group = "competition")
public class autoToUseRight extends LinearOpMode {
    private ElapsedTime aprilTagTimeout = new ElapsedTime();
    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    int timeOutTime = 3 * 1000;
    //int defaultPath = 2;

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

    int ID_TAG_OF_INTEREST = 1; // Tag ID 18 from the 36h11 family

    AprilTagDetection tagOfInterest = null;

    private DcMotor FL;
    private DcMotor FR;
    private DcMotor BL;
    private DcMotor BR;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(800, 448, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        double OpenClaw = 0.5;
        double ClosedClaw = 0.38;

        DcMotor LinearSlide = hardwareMap.get(DcMotor.class, "slide_motor");
        Servo ClawServo = hardwareMap.get(Servo.class, "servo");
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        LinearSlide.setDirection(DcMotor.Direction.REVERSE);
        FL.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.FORWARD);
        BL.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.FORWARD);
        LinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        ClawServo.setPosition(OpenClaw);

        waitForStart();

        int myReturnValue = 0;
        while (opModeIsActive()) {
            aprilTagTimeout.reset();
            while (myReturnValue == 0) {
                myReturnValue = returnAprilTagValue();
                if (aprilTagTimeout.milliseconds() > timeOutTime) {
                    break;
                }
                sleep(10);
            }

            telemetry.addData("Return Value:", myReturnValue);
            telemetry.update();

            ClawServo.setPosition(ClosedClaw);
            sleep(200);
            LinearSlide.setTargetPosition(200);
            LinearSlide.setMode(RUN_TO_POSITION);
            LinearSlide.setPower(0.8);
            while (opModeIsActive() && LinearSlide.isBusy()) {
                LinearSlide.setPower(0.8);
            }
            LinearSlide.setPower(0);
            sleep(200);
            moveForward(1200);
            sleep(200);
            strafe(-1850);
            sleep(200);
            LinearSlide.setTargetPosition(4100);
            LinearSlide.setMode(RUN_TO_POSITION);
            while (opModeIsActive() && LinearSlide.isBusy()) {
                LinearSlide.setPower(0.8);
            }
            LinearSlide.setPower(0);
            moveForward(200);
            sleep(200);
            ClawServo.setPosition(OpenClaw);
            sleep(200);
            moveForward(-200);
            sleep(200);
            ClawServo.setPosition(ClosedClaw);
            sleep(200);
            LinearSlide.setTargetPosition(0);
            LinearSlide.setMode(RUN_TO_POSITION);
            while (opModeIsActive() && LinearSlide.isBusy()) {
                LinearSlide.setPower(0.8);
            }
            LinearSlide.setPower(0);
            sleep(200);
            strafe(550);
            sleep(200);

            if (myReturnValue == 1) {

                strafe(2400);

                break;
            }
            else if (myReturnValue == 2) {

                strafe(1300);

                break;
            }

            else if (myReturnValue == 3) {
                break;
            }
        }
    }
    public int returnAprilTagValue () {
        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
        int tagID;
        if (currentDetections.size() != 0) {
            AprilTagDetection thisTag = currentDetections.get(0);
            telemetry.addData("tag type", thisTag.id);
            telemetry.update();
            tagID = thisTag.id;
            if (tagID > 3 || tagID < 0) {
                tagID = 0;
            }
        } else {
            tagID = 0;
        }
        return tagID;
    }

    public void moveForward(int numTicks) {
        double powerLevel = 0.5;
        FL.setTargetPosition(numTicks);
        FR.setTargetPosition(numTicks);
        BL.setTargetPosition(numTicks);
        BR.setTargetPosition(numTicks);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //telemetry.addLine("Post #1");

        while (opModeIsActive() && FL.isBusy() || FR.isBusy() || BL.isBusy() || BR.isBusy()) {

            FL.setPower(powerLevel);
            FR.setPower(powerLevel);
            BL.setPower(powerLevel);
            BR.setPower(powerLevel);
        }

        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        powerLevel = 0;
        FL.setPower(powerLevel);
        FR.setPower(powerLevel);
        BL.setPower(powerLevel);
        BR.setPower(powerLevel);

    }

    public void strafe(int numTicks){
        double powerLevel = 0.5;
        FL.setTargetPosition(numTicks);
        FR.setTargetPosition(-numTicks);
        BL.setTargetPosition(-numTicks);
        BR.setTargetPosition(numTicks);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive() && FL.isBusy() || FR.isBusy() || BL.isBusy() || BR.isBusy()) {
            FL.setPower(powerLevel);
            FR.setPower(powerLevel);
            BL.setPower(powerLevel);
            BR.setPower(powerLevel);
        }
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        powerLevel = 0;
        FL.setPower(powerLevel);
        FR.setPower(powerLevel);
        BL.setPower(powerLevel);
        BR.setPower(powerLevel);
    }
}