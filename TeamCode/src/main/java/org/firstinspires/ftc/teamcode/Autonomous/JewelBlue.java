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
    private ColorSensor colorSensor;
    private Servo hitter;

    @Override
    public void runOpMode() throws InterruptedException {

        bot.init(hardwareMap, telemetry);

        colorSensor = hardwareMap.colorSensor.get("colorSensor");
        hitter = hardwareMap.servo.get("jHitter");

        int robo = 0;

        waitForStart();

        while(opModeIsActive()){

            switch(robo){
                case 0: {

                    hitter.setPosition(0.5);
                    sleep(1000);
                    bot.jewelArm.setArmDown();
                    sleep(2000);
                    robo++;
                    break;
                }
                case 1: {

                    telemetry.addData("Color Data: ", colorSensor.blue() + " " + colorSensor.red());

                    sleep(1000);

                    if (colorSensor.red() > colorSensor.blue()){

                        telemetry.addData("JEWEL: ", "RED");
                        hitter.setPosition(0);
                        robo++;

                    }
                    if (colorSensor.blue() > colorSensor.red()) {

                        telemetry.addData("JEWEL: ", "BLUE");
                        hitter.setPosition(1);
                        robo++;
                        break;

                    }

                    break;
                }
                case 2: {

                    telemetry.update();
                    hitter.setPosition(0.5);
                    bot.jewelArm.setArmAway();
                    hitter.setPosition(0);
                    sleep(500);
                    robo++;
                    break;
                }

                case 3:


                    break;

            }

        }

    }


}
