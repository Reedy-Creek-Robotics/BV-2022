package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@Autonomous(name = "Not sure",group="testing")

public class OfficialAutonomousBolt extends LinearOpMode {
    private DcMotor FL;
    private DcMotor FR;
    private DcMotor BL;
    private DcMotor BR;

    public int DistanceOfInches(double Revolutions) {
        int Pos = FL.getCurrentPosition() + (int)(538*Revolutions);
        return Pos;
    }

    public void RobotMotorStartup(double DriveSpeed) {

        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive() &&
                FL.isBusy() || FR.isBusy() || BL.isBusy() || BR.isBusy() ) {
            FL.setPower(Math.abs(DriveSpeed));
            FR.setPower(Math.abs(DriveSpeed));
            BR.setPower(Math.abs(DriveSpeed));
            BL.setPower(Math.abs(DriveSpeed));
        }
        FL.setPower(0);
        FR.setPower(0);
        BL.setPower(0);
        BR.setPower(0);

        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(250);

        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void runOpMode() {
        FL = hardwareMap.get(DcMotor.class, "FL");
        FR = hardwareMap.get(DcMotor.class, "FR");
        BL = hardwareMap.get(DcMotor.class, "BL");
        BR = hardwareMap.get(DcMotor.class, "BR");
        FL.setDirection(DcMotor.Direction.REVERSE);
        FR.setDirection(DcMotor.Direction.FORWARD);
        BL.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.FORWARD);
        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();


        int Pos = DistanceOfInches(6.4); // 1 revolution & 0.6 Drive Speed is approx 11.37 in
        if (opModeIsActive()){

            //Go to https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html for strafing directions towards each motor (All motors are set forward by default)

            FL.setTargetPosition(Pos);
            FR.setTargetPosition(Pos);
            BL.setTargetPosition(Pos);
            BR.setTargetPosition(Pos);

            RobotMotorStartup(0.6);

            Pos = DistanceOfInches(2.37); // 1 revolution & 0.6 Drive Speed is approx 11.37 in

            //Go to https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html for strafing directions towards each motor (All motors are set forward by default)

            FL.setTargetPosition(Pos);
            FR.setTargetPosition(-Pos);
            BL.setTargetPosition(-Pos);
            BR.setTargetPosition(Pos);

            RobotMotorStartup(0.6);

            waitForStart();

            Pos = DistanceOfInches(6.4); // 1 revolution & 0.6 Drive Speed is approx 11.37 in

            //Go to https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html for strafing directions towards each motor (All motors are set forward by default)

            FL.setTargetPosition(-Pos);
            FR.setTargetPosition(-Pos);
            BL.setTargetPosition(-Pos);
            BR.setTargetPosition(-Pos);

            RobotMotorStartup(0.6);

            Pos = DistanceOfInches(2.37); // 1 revolution & 0.6 Drive Speed is approx 11.37 in

            //Go to https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html for strafing directions towards each motor (All motors are set forward by default)

            FL.setTargetPosition(-Pos);
            FR.setTargetPosition(Pos);
            BL.setTargetPosition(Pos);
            BR.setTargetPosition(-Pos);

            RobotMotorStartup(0.6);
        }
    }

}

//In this code, 1 tile is basically 2.37 Revolutions (although they can slightly overshoot and hit the poles so it should be slightly adjusted
// if necessary.)