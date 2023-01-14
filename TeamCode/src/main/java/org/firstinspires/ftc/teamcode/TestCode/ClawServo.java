package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@Disabled
@TeleOp(name="Test Claw Servo",group = "testing")


public class ClawServo extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Servo servo;
        servo = hardwareMap.get(Servo.class, "servo");
        waitForStart();
        servo.setPosition(0.35);
        sleep(500);
        while (opModeIsActive()) {
            if (gamepad2.b){
                servo.setPosition(0.35);
                telemetry.addLine("Gamepad.B is pressed");
                telemetry.update();
            }
            if (gamepad2.y){
                servo.setPosition(0.5);
                telemetry.addLine("Gamepad.Y is pressed");
                telemetry.update();
            }

        }
    }
}
