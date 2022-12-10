package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class CrTestOpMode extends LinearOpMode {
    CRServo rotations = null;

    public void runOpMode() {

        rotations = hardwareMap.crservo.get("cr_servo");
        rotations.setDirection(DcMotorSimple.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {

            double power = gamepad1.left_stick_y;
            rotations.setPower(power);
            telemetry.addData("power", rotations.getPower());
            telemetry.update();

        }

    }
}
//Only works when you use a CRservo instead of a regular one.

