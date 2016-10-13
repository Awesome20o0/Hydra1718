package org.firstinspires.ftc.teamcode.Lernaean;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Lernaean.MagazineOpMode;

/**
 * Created by Arib on 4/19/2016.
 */

@TeleOp(name = "teleOp" , group = "OpMode")
public class MagazineTeleOp extends MagazineOpMode {
    boolean isFrontOut = false;
    boolean isBackOut = false;

    @Override
    public void loop(){

        if(gamepad2.x){
            if(!isFrontOut){
                frontOut();
                isFrontOut = true;
            } else {
                frontIn();
                isFrontOut = false;
            }
        }

        if(gamepad2.a){
            if(!isBackOut){
                backOut();
                isBackOut = true;
            } else {
                backIn();
                isBackOut = false;
            }
        }

        if(gamepad1.right_bumper) {
            runManip(1);
        }
        if(gamepad1.left_bumper) {
            runManip(0);
        }

        if (Math.abs(gamepad1.right_stick_y) > .05 || (Math.abs(gamepad1.left_stick_x) > .05)){
            startMotors(gamepad1.right_stick_y,gamepad1.left_stick_y );
        } else {
            stopMotors();
        }
    }
    @Override
    public void stop(){

    }
}
