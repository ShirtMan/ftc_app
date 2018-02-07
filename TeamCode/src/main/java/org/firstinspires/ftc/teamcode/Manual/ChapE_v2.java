package org.firstinspires.ftc.teamcode.Manual;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * 7571 Made by Nick.
 * If you took this code from GitHub,
 * my only request is that you make it better
 */
@TeleOp (name = "ChapE_v2", group = "TeleOp")
public class ChapE_v2 extends OpMode {

    DcMotor intakeLeft, intakeRight;

    Servo flipperServos;

    double i;
    double intakePower;

    @Override
    public void init() {

        flipperServos = hardwareMap.servo.get("flippers");

        intakeLeft = hardwareMap.dcMotor.get("intakeLeft");
        intakeRight = hardwareMap.dcMotor.get("intakeRight");

        intakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        i = 1;
        intakePower = 1;

        flipperServos.setPosition(0);

    }

    @Override
    public void loop() {

        if (gamepad1.left_bumper){
            flipperServos.setPosition(0.75);
        } else {
            flipperServos.setPosition(0);
        }

        if (gamepad1.dpad_up){
            intakePower = 1;
        } else if (gamepad1.dpad_right){
            intakePower = 0.85;
        } else if (gamepad1.dpad_left){
            intakePower = 0.75;
        } else if (gamepad1.dpad_down){
            intakePower = 0.5;
        }

        if (gamepad1.right_stick_button){
            i = -i;
        }

        if (gamepad1.right_bumper){
            changeIntakeSpeeds(intakePower);
        } else {
            changeIntakeSpeeds(0 );
        }

    }

    public void changeIntakeSpeeds(double speed){

        double newSpeed = speed * i;
        intakeLeft.setPower(newSpeed);
        intakeRight.setPower(-newSpeed);

    }

}
