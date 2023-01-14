/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoControllerEx;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorDigitalTouch;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Test Linear Slide Only", group="testing")

public class testLiftOnly extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime timeSinceLastPressHeightButton = new ElapsedTime();
    private ElapsedTime timeSinceLastPressBumper = new ElapsedTime();
    private ElapsedTime timeSinceLastPressClaw = new ElapsedTime();
    private DcMotor liftMotor = null;
    private enum liftModeTypes {MANUAL,FIXED};

    double clawCloseVal = 0.8;
    double clawOpenVal = 0.3;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        liftMotor = hardwareMap.get(DcMotor.class, "Slide");

        liftMotor.setDirection(DcMotor.Direction.REVERSE);
        liftMotor.setTargetPosition(0);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        CRServo intakeServo = hardwareMap.get(CRServo.class, "Intake");
        //Servo intakeServo = hardwareMap.get(Servo.class,"intakeServo");
        Servo turretServo = hardwareMap.get(Servo.class,"Turret");

        double currentClawValue = clawOpenVal;
        double intakeSpeed = 1;

        double leftPos = .4; //0.04;
        double middlePos = .5;//0.36;
        double rightPos = 0.7;
        double turretPos = .5;
        intakeServo.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeServo.setPower(intakeSpeed);
        //intakeServo.setPosition(intakeSpeed);
        turretServo.setPosition(turretPos);

        TouchSensor intakeStop;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        timeSinceLastPressBumper.reset();
        timeSinceLastPressHeightButton.reset();
        timeSinceLastPressClaw.reset();

        double liftPower = 0;
        double liftTarget = 0;
        liftModeTypes currentMode = liftModeTypes.MANUAL;


        intakeStop = hardwareMap.touchSensor.get("stopIntake");

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if(gamepad2.left_bumper && timeSinceLastPressBumper.milliseconds() > 800) {
                turretPos -= 0.01;
                turretServo.setPosition(turretPos);
                timeSinceLastPressBumper.reset();
            }
            if(gamepad2.right_bumper && timeSinceLastPressBumper.milliseconds() > 800) {
                turretPos += 0.01;
                turretServo.setPosition(turretPos);
                timeSinceLastPressBumper.reset();
            }


            if(gamepad2.dpad_up && timeSinceLastPressClaw.milliseconds() > 800) {
                intakeSpeed += 0.1;
                timeSinceLastPressClaw.reset();
            }
            if(gamepad2.dpad_down && timeSinceLastPressClaw.milliseconds() > 800) {
                intakeSpeed -= 0.1;
                timeSinceLastPressClaw.reset();
            }

            if(gamepad2.x && timeSinceLastPressHeightButton.milliseconds() > 800) {
                turretServo.setPosition(leftPos);
                turretPos = leftPos;
                timeSinceLastPressHeightButton.reset();
            }


            if(gamepad2.y && timeSinceLastPressHeightButton.milliseconds() > 800) {
                turretServo.setPosition(middlePos);
                turretPos = middlePos;
                timeSinceLastPressHeightButton.reset();
            }


            if(gamepad2.b && timeSinceLastPressHeightButton.milliseconds() > 800) {
                turretServo.setPosition(rightPos);
                turretPos = rightPos;
                timeSinceLastPressHeightButton.reset();
            }

            if(intakeSpeed > 1) {
                intakeSpeed = 1;
            }else if (intakeSpeed < 0) {
                intakeSpeed = 0;
            }

            if(intakeStop.isPressed()) {
                //intakeServo.setDirection(DcMotorSimple.Direction.FORWARD);
                intakeSpeed = 0;
                intakeServo.setPower(0);
                telemetry.addLine("HEEEEEERE");
                telemetry.update();
                sleep(10000);
                /*sleep(1000);
                intakeServo.setDirection(DcMotorSimple.Direction.REVERSE);
                intakeServo.setPower(0.5);
                sleep(1000);
                intakeServo.setPower(0);
                intakeServo.setDirection(DcMotorSimple.Direction.FORWARD);
                intakeServo.setPower(0.5);
                intakeSpeed=0.5;*/
                //ServoControllerEx controller = (ServoControllerEx) intakeServo.getController();
                //controller.setServoPwmDisable(intakeServo.getPortNumber());
                //controller.setServoPwmEnable(intakeServo.getPortNumber());
            }

            intakeServo.setPower(intakeSpeed);
            //intakeServo.setPosition(intakeSpeed);



            liftPower = -gamepad2.left_stick_y;
            liftMotor.setPower(liftPower);



            // Show the elapsed game time and wheel power.

            telemetry.addData("intakeSpeed",intakeSpeed);
            telemetry.addData("turrentPos",turretPos);
            telemetry.addData("Target: ", liftTarget);
            telemetry.addData("Current",liftMotor.getCurrentPosition());
            telemetry.addData("Motors", "lift (%.2f)", liftPower);
            telemetry.update();
        }
    }
}
