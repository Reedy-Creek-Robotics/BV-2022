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

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@Disabled
@TeleOp(name="GrabConeOpMode", group="testing")

public class GrabConeOpMode extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime servoTimeOut = new ElapsedTime();
    private DcMotor linearSlide = hardwareMap.get(DcMotor.class, "slide_motor");
    private Servo theClawServo = hardwareMap.get(Servo.class, "servo");
    private final double DEBOUNCE_TIME_MS = 500;
    private SlidePosition currentSlidePosition;
    private ClawPosition currentClawPosition;

    enum SlidePosition {
        START(0),
        LOW(100),
        MEDIUM(1500),
        HIGH(3000),
        HIGHEST(4200);

        public final int value;
        private SlidePosition(int _value) {
            value = _value;
        }
    }

    private void MoveSlide(SlidePosition newPosition) {
        telemetry.addData("MoveSlide", newPosition.toString());

        if (currentSlidePosition!=newPosition){
            telemetry.addLine("YOOOOOO");
            linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            linearSlide.setTargetPosition(newPosition.value);
            linearSlide.setPower(.5);
            currentSlidePosition=newPosition;
        }else {
            telemetry.addLine("NOT YOOOOO");
        }
        telemetry.update();

        //TODO: Re-home the motor to prevent encoder drifting here
        //TODO: if slide newPosition == SlidePosition.START, set Power(0), and stop and reset encoder
//        if (newPosition == SlidePosition.START){
//            linearSlide.setPower(0);
//            linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        }
    }

    enum ClawPosition {
        OPEN(0.5),
        CLOSE(0.35);

        public final double value;
        private ClawPosition(double _value) {
            value = _value;
        }
    }

    private void MoveClaw(){
        telemetry.addData("MoveClaw from", currentClawPosition);
        telemetry.update();

        // toggling claw between open and close
        if(currentClawPosition == currentClawPosition.OPEN) {
            theClawServo.setPosition(ClawPosition.CLOSE.value);
            currentClawPosition = currentClawPosition.CLOSE;
        }
        else{
            theClawServo.setPosition(ClawPosition.OPEN.value);
            currentClawPosition=currentClawPosition.OPEN;
        }
    }

    private void buttonCheck(){
        if (gamepad1.a){
            MoveSlide(SlidePosition.START);
        }
        else if (gamepad1.b){
            MoveSlide(SlidePosition.LOW);
        }
        else if (gamepad1.x){
            MoveSlide(SlidePosition.MEDIUM);
        }
        else if (gamepad1.y){
            MoveSlide(SlidePosition.HIGH);
        }
        else if(gamepad1.left_bumper && servoTimeOut.milliseconds() > DEBOUNCE_TIME_MS) {
            MoveClaw();
            servoTimeOut.reset();
        }else if( gamepad1.right_bumper) {
            MoveSlide(SlidePosition.HIGHEST);
        }
    }

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        linearSlide.setDirection(DcMotor.Direction.REVERSE);
        linearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        servoTimeOut.reset();

        //assume that slide is manually moved to the start position
        //with the claw closed
        linearSlide.setPower(0);
        currentSlidePosition = SlidePosition.START;
        currentClawPosition = ClawPosition.CLOSE;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Checking gamepad buttons
            buttonCheck();
        }
    }
}
