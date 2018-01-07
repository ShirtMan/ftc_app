package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Bot;

/**
 * Created by mcshirt on 12/28/17.
 */
@Autonomous(name = "JewelBlue", group = "AutoTest")
public class JewelBlue extends LinearOpMode {

    private Bot bot = new Bot();
    ColorSensor colorSensor;
    Servo hitter, arm;

    @Override
    public void runOpMode() throws InterruptedException {

        bot.init(hardwareMap, telemetry);

        colorSensor = hardwareMap.colorSensor.get("colorSensor");
        hitter = hardwareMap.servo.get("jHitter");
        arm = hardwareMap.servo.get("jArm");

        hitter.setPosition(0);
        arm.setPosition(0);

        waitForStart();

        hitter.setPosition(0.5);
        sleep(1000);
        arm.setPosition(0.4);
        sleep(500);
        arm.setPosition(0.55);
        sleep(2000);
        telemetry.addData("Color Data: ", colorSensor.blue() + " " + colorSensor.red());

        if (colorSensor.red() > colorSensor.blue()){

            telemetry.addData("JEWEL: ", "RED");
            hitter.setPosition(0);

        }
        if (colorSensor.blue() > colorSensor.red()) {

            telemetry.addData("JEWEL: ", "BLUE");
            hitter.setPosition(1);
        }
        sleep(2000);
        telemetry.update();
        hitter.setPosition(0.5);
        arm.setPosition(0);
        hitter.setPosition(0);
        sleep(3000);
    }
}
