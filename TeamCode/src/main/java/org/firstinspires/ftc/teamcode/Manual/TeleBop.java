package org.firstinspires.ftc.teamcode.Manual;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Bot;

/**
 * Created by mcshirt on 11/21/17.
 */

@TeleOp (name = "TeleBop", group = "TeleOp")
public class TeleBop extends OpMode{

    Bot robot = new Bot();

    Servo jArm;

    double powerMultiple;

    @Override
    public void init() {

        robot.init(hardwareMap, telemetry);

        powerMultiple = 1;

        jArm = hardwareMap.servo.get("jArm");

    }

    @Override
    public void start(){

        robot.glyphGrabber.openGrabber();
        jArm.setPosition(0);
    }

    @Override
    public void loop() {

        robot.drive.mecanumDrive(gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x, powerMultiple);


        robot.glyphLifter.moveLift(gamepad2.right_stick_y, gamepad2.left_stick_y);

        if (gamepad2.right_trigger > 0){

            robot.glyphGrabber.setBothPosition(0.5);


        } else if (gamepad2.right_bumper) {

            robot.glyphGrabber.openGrabber();

        } else {

            robot.glyphGrabber.closeGrabber();

        }

        if (gamepad1.left_stick_button){
                powerMultiple = 0.9;
        } else if (gamepad1.right_stick_button) {
                powerMultiple = 0.6;
        } else {
                powerMultiple = 1;
        }


    }
}
