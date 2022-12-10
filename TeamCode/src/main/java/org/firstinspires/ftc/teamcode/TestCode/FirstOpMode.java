package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
@Disabled
public class FirstOpMode extends LinearOpMode {
   // DcMotor m1=null;
    DcMotor m2=null;
   // DcMotor m3=null;
    //DcMotor m4=null;


    public void runOpMode(){
        //DcMotor m1 = hardwareMap.dcMotor.get("backLeftDrive");
        DcMotor m2 = hardwareMap.dcMotor.get("frontLeftDrive");
        //DcMotor m3 = hardwareMap.dcMotor.get("frontRightDrive");
        // DcMotor m4 = hardwareMap.dcMotor.get("backRightDrive");
        // m1.setDirection(DcMotor.Direction.REVERSE);
        // m2.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            m2.setPower(y + x + rx);
           // m1.setPower(y - x + rx);
           // m3.setPower(y - x - rx);
           // m4.setPower(y + x - rx);

        }
    }
}
