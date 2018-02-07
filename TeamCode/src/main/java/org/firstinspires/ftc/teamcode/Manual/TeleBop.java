package org.firstinspires.ftc.teamcode.Manual;

import android.app.Activity;
import android.graphics.Color;
import android.view.Display;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Bot;



/**
 * Created by mcshirt on 11/21/17.
 */

@TeleOp (name = "TeleBop", group = "TeleOp")
public class TeleBop extends OpMode{

    int screenColor;

    Bot robot = new Bot();

    Servo jArm;

    double powerMultiple;



    @Override
    public void init() {

        telemetry.addData("Ready?", "NAH");

        robot.init(hardwareMap, telemetry);

        jArm = hardwareMap.servo.get("jArm");

        powerMultiple = 1;

        telemetry.addData("Ready?", "YE");
    }

    @Override
    public void start(){

        robot.glyphGrabber.openGrabber();

    }

    @Override
    public void loop() {

        jArm.setPosition(0.1);

        if (gamepad1.right_bumper) {
            robot.drive.strafe(1, 1);
        } else if (gamepad1.left_bumper){
            robot.drive.strafe(-1, 1);
        } else {
            robot.drive.tankDrive(gamepad1.left_stick_y, -gamepad1.right_stick_x , powerMultiple);
        }

        robot.glyphLifter.moveLift(gamepad2.right_stick_y, gamepad2.left_stick_y);

        if (gamepad2.left_bumper) {

            robot.glyphGrabber.setBothPosition(0.6);

        } else if (gamepad2.right_trigger > 0) {

            robot.glyphGrabber.setBothPosition(0.5);

        }  else if (gamepad2.right_bumper) {

            robot.glyphGrabber.openGrabber();

        } else {

            robot.glyphGrabber.closeGrabber();
        }

        if (gamepad1.right_trigger > 0){
            powerMultiple = 0.75;
            int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
            final    View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.YELLOW);
                }
            });

        } else if (gamepad1.left_trigger > 0){
            powerMultiple = 0.3;
            int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
            final    View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.RED);
                }
            });
        } else {
            powerMultiple = 1;
            int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
            final    View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);
            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.GREEN);
                }
            });
        }



    }

    @Override
    public void stop(){

        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final    View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);

        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.WHITE);
            }
        });

    }

}
