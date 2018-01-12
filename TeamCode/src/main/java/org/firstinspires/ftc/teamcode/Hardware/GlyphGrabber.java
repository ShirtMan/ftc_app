package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by mcshirt on 11/21/17.
 */

public class GlyphGrabber {

    private Servo leftSide, rightSide;

    private double subtraction;

    private final double SERVO_LEFT_CLOSED_POS = 1;
    private final double SERVO_LEFT_OPEN_POS = 0.3;
    private final double SERVO_RIGHT_CLOSED_POS = 0;
    private final double SERVO_RIGHT_OPEN_POS = 0.7;

    public GlyphGrabber(){


    }
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.initialize(hardwareMap, telemetry);
    }


    private void initialize(HardwareMap hardwareMap, Telemetry telemetry) {

        leftSide = hardwareMap.get(Servo.class, "leftGrabbers");
        rightSide = hardwareMap.get(Servo.class, "rightGrabbers");

        telemetry.addData("GlyphGrabbers: ", "Done");
    }

    public void setBothPosition(double gamepadValue){

        leftSide.setPosition(gamepadValue);
        rightSide.setPosition(gamepadValue);

    }

    public void openGrabber(){

        leftSide.setPosition(SERVO_LEFT_OPEN_POS);
        rightSide.setPosition(SERVO_RIGHT_OPEN_POS);

    }

    public void closeGrabber(){

        leftSide.setPosition(SERVO_LEFT_CLOSED_POS);
        rightSide.setPosition(SERVO_RIGHT_CLOSED_POS);

    }

    public void openSmallAmount(){

        leftSide.setPosition(0.5);
        rightSide.setPosition(0.5);

    }



}
