package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class FirstOpMode extends LinearOpMode {
    DcMotor m1=null;
    DcMotor m2=null;
    DcMotor m3=null;
    DcMotor m4=null;


    public void runOpMode(){
        DcMotor backleft = hardwareMap.dcMotor.get("leftBackDrive");
        DcMotor frontleft = hardwareMap.dcMotor.get("leftFrontDrive");
        DcMotor frontright = hardwareMap.dcMotor.get("rightFrontDrive");
        DcMotor backright = hardwareMap.dcMotor.get("rightBackDrive");
        frontleft.setDirection(DcMotor.Direction.REVERSE);
        backleft.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive()) {
            double x = 0;//-gamepad1.left_stick_y;
            double y = gamepad1.left_stick_y;
            double rx = 0;//gamepad1.right_stick_x;
            if (gamepad1.y) {
                frontleft.setPower(0.3);//;(y + x - rx);
            } if (gamepad1.x) {
                backleft.setPower(0.3);//(y - x - rx);
            } if (gamepad1.a) {
                frontright.setPower(0.3);//(y - x + rx);
            } if (gamepad1.b) {
                backright.setPower(0.3);//(y + x + rx);
            } if (gamepad1.y) {
                frontleft.setPower(0);
            } if (gamepad1.x) {
                backleft.setPower(0);
            } if (gamepad1.a) {
                frontright.setPower(0);
            } if (gamepad1.b) {
                backright.setPower(0);
            }
            telemetry.addData("L stick x",x);
            telemetry.addData("L stick y", y);
            telemetry.addData("R stick x", rx);
            telemetry.addData("FLPower", frontleft.getPower());
            telemetry.addData("FRPower", frontright.getPower());
            telemetry.addData("BLPower", backleft.getPower());
            telemetry.addData("BRPower", backright.getPower());
            telemetry.update();
        }
    }
}
