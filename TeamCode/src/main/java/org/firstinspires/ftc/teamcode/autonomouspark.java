package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Disabled
@Autonomous(name = "Early auto Park",group = "testing")
public class autonomouspark extends LinearOpMode {
    private ElapsedTime aprilTagTimeout = new ElapsedTime();
    int timeOutTime = 3 * 1000;
    int defaultPath = 1;


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

    int ID_TAG_OF_INTEREST = 1; // Tag ID 18 from the 36h11 family

    AprilTagDetection tagOfInterest = null;

    public DcMotor motorFrontLeft = hardwareMap.dcMotor.get("FL");
    public DcMotor motorBackLeft = hardwareMap.dcMotor.get("BL");
    public DcMotor motorFrontRight = hardwareMap.dcMotor.get("FR");
    public DcMotor motorBackRight = hardwareMap.dcMotor.get("BR");


    public void RobotMotorStartup(double DriveSpeed) {

        sleep(100000);
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFrontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBackRight.setDirection((DcMotorSimple.Direction.FORWARD));

        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /*while (opModeIsActive() &&
                motorFrontLeft.isBusy() || motorFrontRight.isBusy() || motorBackLeft.isBusy() || motorBackRight.isBusy()) {
            motorFrontLeft.setPower(Math.abs(DriveSpeed));
            motorFrontRight.setPower(Math.abs(DriveSpeed));
            motorBackRight.setPower(Math.abs(DriveSpeed));
            motorBackLeft.setPower(Math.abs(DriveSpeed));
        }*/

        motorFrontLeft.setPower(0);
        motorFrontRight.setPower(0);
        motorBackLeft.setPower(0);
        motorBackRight.setPower(0);

        /*motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(250);

        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);*/
    }


    public int DistanceOfInches(double Revolutions) {
        int Pos = motorFrontLeft.getCurrentPosition() + (int) (538 * Revolutions);
        return Pos;
    }

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

        telemetry.setMsTransmissionInterval(50);

        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        //while (!isStarted() && !isStopRequested()) {
        waitForStart();
        int myReturnValue = 0;
        while (opModeIsActive()) {
            aprilTagTimeout.reset();
            while (myReturnValue == 0) {
                myReturnValue = returnAprilTagValue();
                if (aprilTagTimeout.milliseconds() > timeOutTime) {
                    myReturnValue = defaultPath;
                    break;
                }
                sleep(10);
            }

            telemetry.addData("Return Value", myReturnValue);
            telemetry.update();

            if (myReturnValue == 1) {
                //right

                //forward
                /*motorFrontLeft.setTargetPosition(1000);
                motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorFrontRight.setTargetPosition(1000);
                motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorBackLeft.setTargetPosition(1000);
                motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorBackRight.setTargetPosition(1000);
                motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);*/
            }
            else if (myReturnValue == 2) {
                    //forward

                    motorFrontLeft.setTargetPosition(1000);
                    motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorFrontRight.setTargetPosition(1000);
                    motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorBackLeft.setTargetPosition(1000);
                    motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorBackRight.setTargetPosition(1000);
                    motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    motorBackLeft.setPower(0.5);
                    motorFrontLeft.setPower(0.5);
                    motorFrontRight.setPower(0.5);
                    motorBackRight.setPower(0.5);
                } else if (myReturnValue == 3) {
                    //left
                /*motorFrontLeft.setTargetPosition(-1000);
                motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorFrontRight.setTargetPosition(1000);
                motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorBackLeft.setTargetPosition(1000);
                motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorBackRight.setTargetPosition(-1000);
                motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //forward
                motorFrontLeft.setTargetPosition(1000);
                motorFrontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorFrontRight.setTargetPosition(1000);
                motorFrontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorBackLeft.setTargetPosition(1000);
                motorBackLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motorBackRight.setTargetPosition(1000);
                motorBackRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);*/
                }
                //motorBackLeft.setPower(.5);
                //motorFrontLeft.setPower(.5);
                //motorFrontRight.setPower(.5);
                //motorBackRight.setPower(.5);
            }

        }

    public int returnAprilTagValue()
    {
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
}
