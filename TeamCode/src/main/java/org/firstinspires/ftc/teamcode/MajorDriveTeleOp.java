package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.TestCode.ClawServo;

import java.lang.Math;
@TeleOp
public class MajorDriveTeleOp extends LinearOpMode {

    int intake = 0;
    int travel = 500;
    int low = 1200;
    int medium = 3000;
    int high = 4200;
    double limiter = 1;

    @Override

    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
         ElapsedTime runtime = new ElapsedTime();
         DcMotor LinearSlide= hardwareMap.get(DcMotor.class, "slide_motor");
         Servo ClawServo = hardwareMap.get(Servo.class, "servo");

        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("FL");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("BL");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("FR");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("BR");


        //--------------------------------------------

        int SafetyLockBool = 0;
        int CurrentPosition;
        int TargetPosition;
        int GrabTargetPosition = 0;
        int GroundTargetPosition = 100;
        int LowTargetPosition = 1700;
        int MediumTargetPosition = 2900;
        int HighestTargetPosition = 4050;
        double ClawPos;
        double OpenClaw = 0.5;
        double ClosedClaw = 0.38;
        double CurrentPower = 0.8;


        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        ClawServo.setPosition(0.0);
        LinearSlide.setDirection(DcMotor.Direction.REVERSE);
        LinearSlide.setTargetPosition(0);
        LinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            if (gamepad1.dpad_up) {
                limiter = 1;
            }
            if (gamepad1.dpad_down) {
                limiter = 0.3;
            }

            double correctionFactor = 1.1;
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * correctionFactor; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;
            //double slideY = gamepad2.right_stick_y;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = ((y + x + rx) / denominator) * limiter;
            double backLeftPower = ((y - x + rx) / denominator) * limiter;
            double frontRightPower = ((y - x - rx) / denominator) * limiter;
            double backRightPower = ((y + x - rx) / denominator) * limiter;


            if (gamepad1.x && runtime.milliseconds() > 1000) {
                LinearSlide.setTargetPosition(GrabTargetPosition);
                LinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if (gamepad1.x && runtime.milliseconds() > 1000 || SafetyLockBool == 1) {

                if (LinearSlide.isBusy()) {
                    ClawServo.setPosition(ClosedClaw);
                }
                if (motorBackLeft.isBusy() || motorBackRight.isBusy() || motorFrontLeft.isBusy() || motorFrontRight.isBusy() || LinearSlide.isBusy()) {
                    telemetry.addLine("Safety Push & Lock Enabled");
                }
                SafetyLockBool = 1;
                motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorBackLeft.setTargetPosition((int) -425.02);
                motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorBackRight.setTargetPosition((int) -425.02);
                motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorFrontLeft.setTargetPosition((int) -425.02);
                motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorFrontRight.setTargetPosition((int) -425.02);
                if (motorBackLeft.isBusy() || motorBackRight.isBusy() || motorFrontLeft.isBusy() || motorFrontRight.isBusy()) {
                    motorBackLeft.setPower(0.4);
                    motorFrontRight.setPower(0.4);
                    motorBackRight.setPower(0.4);
                    motorFrontLeft.setPower(0.4);
                } else {
                    telemetry.addLine("Safety Push & Lock Disabling... Restart Robot if you see this message for more than 5 seconds");
                    motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    SafetyLockBool = 0;
                }
            } else if (SafetyLockBool == 0) {
                telemetry.addLine("Safety Push & Lock Disabled");
            }
            if (gamepad2.a && runtime.milliseconds() > 1000) {
                ClawServo.setPosition(ClosedClaw);
                LinearSlide.setTargetPosition(GroundTargetPosition);
                LinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if (gamepad1.x && runtime.milliseconds() > 1000 || SafetyLockBool == 1) {
                if (LinearSlide.isBusy()) {
                    ClawServo.setPosition(0.35);
                }
                if (motorBackLeft.isBusy() || motorBackRight.isBusy() || motorFrontLeft.isBusy() || motorFrontRight.isBusy() || LinearSlide.isBusy()) {
                    telemetry.addLine("Safety Push & Lock Enabled");
                }
                SafetyLockBool = 1;
                motorBackLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorBackLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorBackLeft.setTargetPosition((int) -425.02);
                motorBackRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorBackRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorBackRight.setTargetPosition((int) -425.02);
                motorFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorFrontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorFrontLeft.setTargetPosition((int) -425.02);
                motorFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                motorFrontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                motorFrontRight.setTargetPosition((int) -425.02);
                if (motorBackLeft.isBusy() || motorBackRight.isBusy() || motorFrontLeft.isBusy() || motorFrontRight.isBusy()) {
                    motorBackLeft.setPower(0.4);
                    motorFrontRight.setPower(0.4);
                    motorBackRight.setPower(0.4);
                    motorFrontLeft.setPower(0.4);
                } else {
                    telemetry.addLine("Safety Push & Lock Disabling... Restart Robot if you see this message for more than 5 seconds");
                    motorBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    motorBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    motorFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    motorFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    SafetyLockBool = 0;
                }
            } else if (SafetyLockBool == 0) {
                telemetry.addLine("Safety Push & Lock Disabled");
            }
            if (gamepad2.x && runtime.milliseconds() > 1000) {
                LinearSlide.setTargetPosition(LowTargetPosition);
                LinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if (gamepad2.b && runtime.milliseconds() > 1000) {
                LinearSlide.setTargetPosition(MediumTargetPosition);
                LinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if (gamepad2.y && runtime.milliseconds() > 1000) {
                LinearSlide.setTargetPosition(HighestTargetPosition);
                LinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if (gamepad2.left_bumper && runtime.milliseconds() > 1000) {
                ClawServo.setPosition(ClosedClaw);
            }
            if (gamepad2.right_bumper && runtime.milliseconds() > 1000) {
                ClawServo.setPosition(OpenClaw);
            }
            LinearSlide.setPower(CurrentPower);
            ClawPos = ClawServo.getPosition();
            TargetPosition = LinearSlide.getTargetPosition();
            CurrentPosition = LinearSlide.getCurrentPosition();

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Current Position:", CurrentPosition);
            telemetry.addData("Target Position:", TargetPosition);
            telemetry.addData("Current Power:", CurrentPower);
            telemetry.addData("Claw Current Position:", ClawPos);

            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);

            telemetry.update();
        }
    }
}