package org.firstinspires.ftc.teamcode.TestCode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
@Disabled
public class LinearSlide extends LinearOpMode {
DcMotor LinearSlide = null;
    @Override
    public void runOpMode() throws InterruptedException {
        LinearSlide = hardwareMap.dcMotor.get("LinearSlide");
        double y = gamepad2.left_stick_y;
        /*enum HEIGHT {
            NONE,
            GROUND,
            LOW,
            MEDIUM,
            HIGH
        }
        HEIGHT DESIGNATED_HEIGHT;
        /*switch (DESIGNATED_HEIGHT) {
            case NONE:
                LinearSlide.setTargetPosition(0);
                break;
            case GROUND:
                LinearSlide.setTargetPosition(/*measured ground junction pasted here);
                break;
            case LOW:
                LinearSlide.setTargetPosition(/*measured low pole junction pasted here);
                break;
            case MEDIUM:
                LinearSlide.setTargetPosition(/*measured moderate pole junction pasted here);
                break;
            case HIGH:
                LinearSlide.setTargetPosition(/*measured high pole junction pasted here);
                break;
        }*/
        while (opModeIsActive()) {
            LinearSlide.setPower(y);
        }
    }
}






