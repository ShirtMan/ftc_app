package org.firstinspires.ftc.teamcode.Manual;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.internal.ui.GamepadUser;
import org.firstinspires.ftc.teamcode.Hardware.MecanumDrive;

/**
 * 7571 Made by Nick.
 * If you took this code from GitHub,
 * my only request is that you make it better
 */
@TeleOp (name = "TeleBop", group = "TeleOp")
public class ChapE_v2 extends OpMode {

    DcMotor intakeLeft, intakeRight, lifter, flipper;

    Servo jArm;

    MecanumDrive drive = new MecanumDrive();

    double intakePower;
    double drivePower;


    @Override
    public void init() {

        /*
        grabberLeft = hardwareMap.servo.get("grabberLeft");
        grabberRight = hardwareMap.servo.get("grabberRight");
        */

        intakeLeft = hardwareMap.dcMotor.get("intakeLeft");
        intakeRight = hardwareMap.dcMotor.get("intakeRight");

        lifter = hardwareMap.dcMotor.get("lifter");

        flipper = hardwareMap.dcMotor.get("flipper");

        intakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeRight.setDirection(DcMotorSimple.Direction.REVERSE);

        lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lifter.setDirection(DcMotorSimple.Direction.REVERSE);

        intakePower = 1;

        drive.init(hardwareMap, telemetry);

        drivePower = 1;

        jArm = hardwareMap.servo.get("jArm");


    }

    @Override
    public void loop() {

        jArm.setPosition(1);

        if (gamepad1.right_bumper) {
            drive.strafe(-1, 1);
        } else if (gamepad1.left_bumper){
            drive.strafe(1, 1);
        } else {
            drive.tankDrive(gamepad1.left_stick_y, gamepad1.right_stick_x , drivePower);
        }

        flipper.setPower((gamepad2.right_trigger - gamepad2.left_trigger) * 0.6);

        if (gamepad2.dpad_up){
            intakePower = 1;
        } else if (gamepad2.dpad_right){
            intakePower = 0.825;
        } else if (gamepad2.dpad_left){
            intakePower = 0.75;
        } else if (gamepad2.dpad_down){
            intakePower = 0.5;
        }

        if (gamepad2.right_bumper){
            changeIntakePower(-intakePower);
        } else if (gamepad2.left_bumper){
            changeIntakePower(intakePower);
        } else {
            changeIntakePower(0);
        }

        changeLiftPower(-gamepad2.right_stick_y);

        if (gamepad1.right_trigger > 0){
            drivePower = 0.3;
            int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
            final    View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.RED);
                }
            });
        } else {
            drivePower = 1;
            int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
            final    View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.GREEN);
                }
            });
        }

    }

    public void changeIntakePower(double power){

        intakeLeft.setPower(power);
        intakeRight.setPower(power);

    }

    public void changeLiftPower(double power){

        lifter.setPower(power);

    }

    public void encoderDrive(double speed,
                             int newPos, DcMotorSimple.Direction direction) {

        flipper.setDirection(direction);

        flipper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        flipper.setTargetPosition(newPos);

        flipper.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        flipper.setPower(Math.abs(speed));

        while (flipper.isBusy()) {

        }

        flipper.setPower(0);

        flipper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    @Override
    public void stop(){

        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.WHITE);
            }
        });

    }


}