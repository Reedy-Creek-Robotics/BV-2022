package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class ActaulLinearSlideCode extends LinearOpMode {
    DcMotor slidemotor=null;
    int state=1;


    public void runOpMode(){
        DcMotor slidemotor = hardwareMap.dcMotor.get("slide_motor");

        waitForStart();

        while(opModeIsActive()){
            //when button x is pressed the slide goes up(power 1)
            //when button x is pressed while slide is going up(power 0)
            //when button x is pressed when stopped(power -1)
            telemetry.addData("stage","stage");
            telemetry.update();



            if (gamepad1.x && state==1){
                slidemotor.setPower(0.2);
                state=2;
                sleep(500);
            }
            if (gamepad1.x && state==2){
                slidemotor.setPower(0);
                state=3;
                sleep(500);
            }
            if (gamepad1.x && state==3){
                slidemotor.setPower(-0.2);
                state=1;
                sleep(500);
            }



        }
    }

}
