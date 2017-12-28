package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.ENUM.COLORS;

import static org.firstinspires.ftc.teamcode.ENUM.COLORS.BLUE;
import static org.firstinspires.ftc.teamcode.ENUM.COLORS.RED;
import static org.firstinspires.ftc.teamcode.ENUM.COLORS.UNDEFINED;

/**
 * Created by mcshirt on 11/29/17.
 */

public class JewelArm {

    private Servo arm;
    private Servo knocker;
    //private ColorSensor colorSensor;
    //private COLORS jewelColor;


    private final double ARM_UP_POS = 0;
    private final double ARM_DOWN_POS = 1;
    private final double ARM_AWAY_POS = 0.3;

    public JewelArm() {

    }

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.initialize(hardwareMap, telemetry);
    }


    private void initialize(HardwareMap hardwareMap, Telemetry telemetry) {

        arm = hardwareMap.get(Servo.class, "jArm");
        //knocker = hardwareMap.get(Servo.class, "jHitter");
        //colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        //hitterStraight();
        setArmUp();

        telemetry.addData("JewelArm: ", "Done");
    }

    public void setArmUp() {

        arm.setPosition(ARM_UP_POS);

    }

    public void setArmDown() {

        arm.setPosition(ARM_DOWN_POS);

    }

    public void setArmAway() {

        arm.setPosition(ARM_AWAY_POS);

    }

    /*public void hitBackJewel() {

        knocker.setPosition(1);

    }

    public void hitFrontJewel() {

        knocker.setPosition(0.3);

    }
    public void hitterStraight(){

        knocker.setPosition(0.8);

    }
    public void setHitterPos(double position){

        knocker.setPosition(position);

    }*/

    public COLORS getBackJewel() {
/*
        if (colorSensor.blue() > colorSensor.red()){

            jewelColor = BLUE;

        } else if (colorSensor.blue() < colorSensor.red()){

            jewelColor = RED;

        } else {

            jewelColor = UNDEFINED;

        }

        return jewelColor;
    }
*/
        return BLUE;
    }
}
