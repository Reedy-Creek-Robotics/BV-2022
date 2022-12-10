package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp
public class newclawcode extends LinearOpMode {
    Servo servo = null;
    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(Servo.class, "servo");

        waitForStart();
        servo.setPosition(0.0);

        sleep(500);
        while (opModeIsActive()) {
            if (gamepad2.x){
                servo.setPosition(0.4);
            }
            if (gamepad1.b){
                servo.setPosition(0.0);
            }

        }
    }
}
