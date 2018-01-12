package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by mcshirt on 1/1/18.
 */

@TeleOp (name = "OneDriver - Shirt", group = "Test")
public class Test_Telebop extends OpMode{

    Bot robot = new Bot();

    Servo jArm;

    @Override
    public void init() {

        telemetry.addData("Ready?", "NAH");

        robot.init(hardwareMap, telemetry);

        jArm = hardwareMap.servo.get("jArm");

        telemetry.addData("Ready?", "YE");

    }

    @Override
    public void start() {

        jArm.setPosition(0.1);

    }

    @Override
    public void loop() {

        robot.drive.mecanumDrive(-gamepad1.left_stick_y, gamepad1.right_stick_x, gamepad1.left_stick_x, 1);

        robot.glyphLifter.moveLift(gamepad1.right_trigger - gamepad1.left_trigger, 0);

        if (gamepad1.right_stick_button){
            robot.glyphLifter.encoderDrive(1, 1820, DcMotorSimple.Direction.FORWARD);
        } else if (gamepad1.left_stick_button){
            robot.glyphLifter.encoderDrive(1, 1820, DcMotorSimple.Direction.REVERSE);
        }

        if (gamepad1.right_bumper) {
            robot.glyphGrabber.openGrabber();
        } else if (gamepad1.left_bumper) {
            robot.glyphGrabber.openSmallAmount();
        } else {
            robot.glyphGrabber.closeGrabber();
        }

        telemetry.addData("Lift is busy: ", "maybe");
        telemetry.update();

    }
}
