package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import java.lang.Math;
@TeleOp
public class DrivePlusLiftTeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        double low = -0.1;
        double high = 0.1;
        // Make sure your ID's match your configuration
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("FL");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("BL");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("FR");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("BR");
        DcMotor slidemotor = hardwareMap.dcMotor.get("slide_motor");

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double correctionFactor = 1.1;
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * correctionFactor; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;
            double slideY = gamepad2.right_stick_y;
            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            if (Math.abs(slideY) < 0.1){
                slidemotor.setPower(0);
            }
            else{
                slidemotor.setPower(slideY);
            }

            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);

            telemetry.addData("slide power", slideY);
            telemetry.update();
        }
    }
}