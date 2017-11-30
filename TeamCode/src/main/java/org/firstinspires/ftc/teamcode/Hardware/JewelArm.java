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

    private Servo jewelArm;
    private ColorSensor colorSensor;
    private COLORS jewelColor;


    private final double ARM_UP_POS = 0;
    private final double ARM_DOWN_POS = 0.7;
    private final double ARM_AWAY_POS = 0.3;

    public JewelArm(){

    }
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.initialize(hardwareMap, telemetry);
    }


    private void initialize(HardwareMap hardwareMap, Telemetry telemetry) {

        jewelArm = hardwareMap.get(Servo.class, "jewelArm");
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");

        setArmUp();

        telemetry.addData("JewelArm: ", "Done");
    }

    public void setArmUp(){

        jewelArm.setPosition(ARM_UP_POS);

    }
    public void setArmDown(){

        jewelArm.setPosition(ARM_DOWN_POS);

    }
    public void setArmAway(){

        jewelArm.setPosition(ARM_AWAY_POS);

    }

    public COLORS getBackJewel(){

        if (colorSensor.blue() > colorSensor.red()){

            jewelColor = BLUE;

        } else if (colorSensor.blue() < colorSensor.red()){

            jewelColor = RED;

        } else {

            jewelColor = UNDEFINED;

        }

        return jewelColor;
    }

}
