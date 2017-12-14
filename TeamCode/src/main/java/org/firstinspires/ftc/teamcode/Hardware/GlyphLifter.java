package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by mcshirt on 11/21/17.
 */

public class GlyphLifter {

    private DcMotor smallLifter, glyphLifter;

    public GlyphLifter(){


    }

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.initialize(hardwareMap, telemetry);
    }


    private void initialize(HardwareMap hardwareMap, Telemetry telemetry) {

        glyphLifter = hardwareMap.get(DcMotor.class, "glyphLifter");
        smallLifter = hardwareMap.get(DcMotor.class, "smallLifter");

        glyphLifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        smallLifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        glyphLifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("GlyphLifter: ", "Done");
    }

    public void moveLift(double bigPower, double smallPower){

        glyphLifter.setPower(bigPower);
        smallLifter.setPower(smallPower);
    }

    public void moveLiftInACoolWay(double power){

        glyphLifter.setPower(power);
        smallLifter.setPower(power * 0.75);

    }

    public void stopMovement(){

        glyphLifter.setPower(0);
        smallLifter.setPower(0);


    }

    public void nudge(){

        glyphLifter.setPower(1);

    }

}
