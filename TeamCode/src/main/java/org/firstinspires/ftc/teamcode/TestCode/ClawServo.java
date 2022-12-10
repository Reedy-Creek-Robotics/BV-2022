package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp
public class ClawServo extends LinearOpMode {
    Servo servo;
    double steps=0.05;
    double currentPosition=1.0;
    double firstPosition=0;
    double savedPosition = 0.0;
    int state = 0;
    // 0 Open claw, start loop to decrease claw
    // A is pressed go to state = 1
    // After two seconds go to state = 2
    // Store position as closed position
    // 2 Open wait for button presses
    // If button A claw open
    // If button B claw close to save position
    @Override
    public void runOpMode() throws InterruptedException {
        servo = hardwareMap.get(Servo.class, "servo");
        waitForStart();

        servo.setPosition(firstPosition);
        sleep(500);
        while (opModeIsActive()) {
            telemetry.addData("angle", "%.2f", currentPosition);
            telemetry.addData("state","%.2f",(double) state);
            telemetry.addData("Saved Position",savedPosition);
            telemetry.update();
            if(state==0){
                currentPosition -= steps;
                servo.setPosition(currentPosition);
                sleep(500);
            }
            if(state == 1) {
                savedPosition = currentPosition;
                //sleep(2000);
                servo.setPosition(firstPosition);
                //sleep(2000);
                state=2;
            }
            if(state == 2){
                servo.setPosition(firstPosition);
                state=1;
            }
            if(state == 3) {
                servo.setPosition(savedPosition);
                sleep(2000);
                state=1;
            }
            if(gamepad1.a){
                if(state == 0){
                    state = 1;
                } else if(state == 3) {
                    state = 2;
                }
            }
            if(gamepad1.b && state == 2){
                state = 3;
            }
        }
    }
}