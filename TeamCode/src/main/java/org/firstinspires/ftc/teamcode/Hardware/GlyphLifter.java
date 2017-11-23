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

    private DcMotor glyphLifter;

    public GlyphLifter(){


    }

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.initialize(hardwareMap, telemetry);
    }


    private void initialize(HardwareMap hardwareMap, Telemetry telemetry) {

        glyphLifter = hardwareMap.get(DcMotor.class, "glyphLifter");

        glyphLifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        glyphLifter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addData("GlyphLifter: ", "Done");
    }

    public void moveLift(double power){

        glyphLifter.setPower(power);

    }

    public void nudge(int position){

        glyphLifter.setTargetPosition(position);

        glyphLifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        glyphLifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        glyphLifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        glyphLifter.setPower(0.5);

        while(glyphLifter.isBusy()){

        }

        glyphLifter.setPower(0);

    }

}
