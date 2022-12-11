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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
//import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;


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

@TeleOp(name="GrabConeOpModeOld", group="Linear Opmode")

public class GrabConeOpModeOld extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor linearSlide = null;
    private Servo theClawServo = null;


    //figure out how to make a log output to debug
    //private android.util.Log log = Telemetry.log();

    SlidePosition currentSlidePosition;
    ClawPosition currentClawPosition;
    com.qualcomm.robotcore.hardware.DcMotor.Direction motorDirection;

    private void MoveSlide(SlidePosition newPosition) {

        telemetry.addData("MoveSlide", newPosition.toString());
        telemetry.update();

        if (currentSlidePosition!=newPosition){
            //linearSlide.setDirection(motorDirection);
            linearSlide.setTargetPosition(newPosition.value);
            currentSlidePosition=newPosition;
        }

    }

    private void buttonCheck(){
        if (gamepad1.a){
            telemetry.log();
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
        else if(gamepad1.left_bumper) {
            if(currentClawPosition == currentClawPosition.OPEN) {
                //theClawServo.setPosition(ClawPosition.CLOSE.value);
                currentClawPosition = currentClawPosition.CLOSE;
            }
            else{
                //theClawServo.setPosition(ClawPosition.OPEN.value);
                currentClawPosition=currentClawPosition.OPEN;
            }
        }else if( gamepad1.right_bumper) {
            MoveSlide(SlidePosition.HIGHEST);
        }
    }

    enum Direction {
        REVERSE,
        FORWARD
    }

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
    enum ClawPosition {
        OPEN(0),
        CLOSE(1);

        public final int value;
        private ClawPosition(int _value) {
            value = _value;
        }
    }
    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        linearSlide  = hardwareMap.get(DcMotor.class, "linear_slide");
        //theClawServo = hardwareMap.get(Servo.class, "clawServo");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        //leftDrive.setDirection(DcMotor.Direction.REVERSE);
        //rightDrive.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        //assume that slide is manually moved to the start position
        //with the claw closed
        linearSlide.setPower(.5);
        currentSlidePosition = SlidePosition.START;
        currentClawPosition = ClawPosition.CLOSE;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Checking gamepad buttons every 50 milliseconds
            buttonCheck();
            sleep(50);


            // Show the elapsed game time and wheel power

        }
    }
}
