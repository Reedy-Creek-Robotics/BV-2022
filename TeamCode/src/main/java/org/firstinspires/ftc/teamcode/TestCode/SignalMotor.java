package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
@Disabled
public class SignalMotor extends LinearOpMode {
    private DcMotor FL = null;
    private DcMotor FR = null;
    private DcMotor BL = null;
    private DcMotor BR = null;

    public int DistanceOfInches(double Revolutions) {
        int Pos = FL.getCurrentPosition() + (int)(538*Revolutions);
        return Pos;
    }

    public void RobotMotorStartup(double DriveSpeed) {

        FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        BL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive() && FL.isBusy() || FR.isBusy() || BL.isBusy() || BR.isBusy()) {
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

            if(opModeIsActive()){
                // This code moves the bot approximately 1 tile. Distance of Inches method determines the distance
                // while Robot Motor Startup activates the robot. Robot Motor Startup includes a wait period for the robot
                // to have a chance to get things through. DriveSpeed is 0.6 for default DriveSpeed. Remember to remove
                // this comment and try to replace it at the very beginning of this class, thanks!
                int Pos = DistanceOfInches(2.27); // 1 revolution & 0.6 Drive Speed is approx 11.37 in

                //Go to https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html for strafing directions towards each motor (All motors are set forward by default)

                FL.setTargetPosition(Pos);
                FR.setTargetPosition(Pos);
                BL.setTargetPosition(Pos);
                BR.setTargetPosition(Pos);

                RobotMotorStartup(0.6);
        }
    }





/*
    private enum startLocation {BlueLeft, BlueRight, RedLeft, RedRight}
    //startLocation localStartLocation = startLocation.BlueLeft;




    private void AutoPath(int signalValue, startLocation startlocation) {
        if(signalValue == 1) {
            if (startlocation == startLocation.BlueLeft) {
                telemetry.addLine("Signal Value = 1, Starting Position = BlueLeft");
                telemetry.update();
            } else if (startlocation == startLocation.BlueRight) {
                telemetry.addLine("Signal Value = 1, Starting Position = BlueRight");
                telemetry.update();
            } else if (startlocation == startLocation.RedLeft) {
                telemetry.addLine("Signal Value = 1, Starting Position = RedLeft");
                telemetry.update();
            } else if (startlocation == startLocation.RedRight) {
                telemetry.addLine("Signal Value = 1, Starting Position = RedRight");
                telemetry.update();
            }
        }
        else if(signalValue == 2) {
            if (startlocation == startLocation.BlueLeft){
                telemetry.addLine("Signal Value = 2, Starting Position = BlueLeft");
                telemetry.update();
            } else if (startlocation == startLocation.BlueRight) {
                telemetry.addLine("Signal Value = 2, Starting Position = BlueRight");
                telemetry.update();
            } else if (startlocation == startLocation.RedLeft) {
                telemetry.addLine("Signal Value = 2, Starting Position = RedLeft");
                telemetry.update();
            } else if (startlocation == startLocation.RedRight) {
                telemetry.addLine("Signal Value = 2, Starting Position = RedRight");
                telemetry.update();
            }
        }
        else if (signalValue == 3) {
            if (startlocation == startLocation.BlueLeft) {
                telemetry.addLine("Signal Value = 3, Starting Position = BlueLeft");
                telemetry.update();
            } else if (startlocation == startLocation.BlueRight) {
                telemetry.addLine("Signal Value = 3, Starting Position = BlueRight");
                telemetry.update();
            } else if (startlocation == startLocation.RedLeft) {
                telemetry.addLine("Signal Value = 3, Starting Position = RedLeft");
                telemetry.update();
            } else if (startlocation == startLocation.RedRight) {
                telemetry.addLine("Signal Value = 3, Starting Position = RedRight");
                telemetry.update();
            }
        }
    }*/
}
