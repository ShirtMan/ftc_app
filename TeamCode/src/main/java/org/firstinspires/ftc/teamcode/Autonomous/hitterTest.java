package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by mcshirt on 12/30/17.
 */
@Autonomous (name = "Hitter test", group = "Test")
public class hitterTest extends LinearOpMode {

    @Override
    public void runOpMode(){

        Servo hitter;
        Servo arm;

        arm = hardwareMap.servo.get("jArm");
        hitter = hardwareMap.servo.get("jHitter");

        arm.setPosition(0);
        hitter.setPosition(0.5);

        waitForStart();

        while(opModeIsActive()) {
            hitter.setPosition(1);
            sleep(1000);
            hitter.setPosition(0);
            sleep(1000);
        }
    }

}
