package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class MotorDirectionTest extends LinearOpMode {

    public void runOpMode(){
        DcMotor backleft = hardwareMap.dcMotor.get("leftBackDrive");
        DcMotor frontleft = hardwareMap.dcMotor.get("leftFrontDrive");
        DcMotor frontright = hardwareMap.dcMotor.get("rightFrontDrive");
        DcMotor backright = hardwareMap.dcMotor.get("rightBackDrive");

        waitForStart();

        while(opModeIsActive()) {
            if (gamepad1.y) {
                frontright.setPower(0.3);//;(y + x - rx);
            } if (gamepad1.x) {
                frontleft.setPower(0.3);//(y - x - rx);
            } if (gamepad1.a) {
                backleft.setPower(0.3);//(y - x + rx);
            } if (gamepad1.b) {
                backright.setPower(0.3);//(y + x + rx);
            } if (gamepad1.y) {
                frontright.setPower(0);
            } if (gamepad1.x) {
                frontleft.setPower(0);
            } if (gamepad1.a) {
                backleft.setPower(0);
            } if (gamepad1.b) {
                backright.setPower(0);
            }
            telemetry.addData("FLPower", frontleft.getPower());
            telemetry.addData("FRPower", frontright.getPower());
            telemetry.addData("BLPower", backleft.getPower());
            telemetry.addData("BRPower", backright.getPower());
            telemetry.update();
        }
    }
}
