package org.firstinspires.ftc.teamcode.Libraries;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Arib on 10/6/2016.
 */
public class Manipulator {
    DcMotor manip;
    LinearOpMode opMode;

    private final String LOG_TAG = "DriveTrain";
    public Manipulator (LinearOpMode opMode){
        this.opMode = opMode;
        manip = this.opMode.hardwareMap.dcMotor.get("mani");
        this.opMode.telemetry.addData(LOG_TAG + "init", "finished drivetrain init");
        this.opMode.telemetry.update();
    }

    public void runCollector(double pow){
        manip.setPower(pow * -1);
    }
}
