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
@TeleOp(name="TestLinearSlide", group="Test")
public class TestLinearSlide extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor LinearSlide= hardwareMap.get(DcMotor.class, "slide_motor");
    private Servo ClawServo = hardwareMap.get(Servo.class, "Servo");

    //--------------------------------------------

    int CurrentPosition;
    int TargetPosition;
    int GrabTargetPosition = 0;
    int GroundTargetPosition = 100;
    int LowTargetPosition = 1500;
    int MediumTargetPosition = 3000;
    int HighestTargetPosition = 4500;
    double ClawServoTarget;
    double ClawPos;
    double OpenClaw = 0.8;
    double ClosedClaw = 0.35;
    double CurrentPower = 0.5;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        LinearSlide.setDirection(DcMotor.Direction.REVERSE);
        LinearSlide.setTargetPosition(0);
        LinearSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            if (gamepad1.x && runtime.milliseconds() > 1000) {
                LinearSlide.setTargetPosition(GrabTargetPosition);
                LinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if (gamepad2.a && runtime.milliseconds() > 1000) {
                LinearSlide.setTargetPosition(GroundTargetPosition);
                LinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if (gamepad2.x && runtime.milliseconds() > 1000) {
                LinearSlide.setTargetPosition(LowTargetPosition);
                LinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if (gamepad2.b && runtime.milliseconds() > 1000) {
                LinearSlide.setTargetPosition(MediumTargetPosition);
                LinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if (gamepad2.y && runtime.milliseconds() > 1000) {
                LinearSlide.setTargetPosition(HighestTargetPosition);
                LinearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if (gamepad2.left_bumper && runtime.milliseconds() > 1000) {
                ClawServo.setPosition(ClosedClaw);
                ClawServoTarget = ClosedClaw;
            }
            if (gamepad2.right_bumper && runtime.milliseconds() > 1000) {
                ClawServo.setPosition(OpenClaw);
                ClawServoTarget = OpenClaw;
            }
            LinearSlide.setPower(CurrentPower);
            ClawPos = ClawServo.getPosition();
            TargetPosition = LinearSlide.getTargetPosition();
            CurrentPosition = LinearSlide.getCurrentPosition();

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Current Position:", CurrentPosition);
            telemetry.addData("Target Position:", TargetPosition);
            telemetry.addData("Current Power:", CurrentPower);
            telemetry.addData("Claw Current Position:", ClawPos);
            telemetry.addData("Claw Servo Target Position:", ClawServoTarget);
            telemetry.update();
        }
    }
}
