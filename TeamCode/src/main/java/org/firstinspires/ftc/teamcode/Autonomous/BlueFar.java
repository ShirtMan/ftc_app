package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.ENUM.COLORS;
import org.firstinspires.ftc.teamcode.ENUM.STEP;

import static org.firstinspires.ftc.teamcode.ENUM.COLORS.BLUE;
import static org.firstinspires.ftc.teamcode.ENUM.COLORS.RED;
import static org.firstinspires.ftc.teamcode.ENUM.STEP.KNOCKBACKJEWEL;
import static org.firstinspires.ftc.teamcode.ENUM.STEP.KNOCKFRONTJEWEL;
import static org.firstinspires.ftc.teamcode.ENUM.STEP.KNOCKJEWEL;
import static org.firstinspires.ftc.teamcode.ENUM.STEP.MOVETOSAFEZONE;

/**
 * Created by mcshirt on 11/29/17.
 */
@Autonomous (name = "AutoBlueFar", group = "Main")
public class BlueFar extends LinearOpMode {

    Bot robot = new Bot();

    //STEP autoStep;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap,telemetry);

        waitForStart();

        robot.drive.stopMovement();
        robot.glyphGrabber.openGrabber();
        sleep(500);
        robot.glyphLifter.moveLift(0.4, 0);
        sleep(1000);
        robot.glyphLifter.stopMovement();

        robot.drive.moveForward();
        sleep(2500);
        robot.drive.stopMovement();
        robot.glyphGrabber.closeGrabber();
        robot.drive.moveBackward();
        sleep(1000);
        robot.drive.stopMovement();
        sleep(500);
        robot.drive.setThrottle(0.3);
        sleep(2000);
        robot.drive.stopMovement();
        sleep(500);
        robot.drive.moveBackward();
        sleep(250);
        robot.drive.stopMovement();

    }
}
