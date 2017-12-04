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
@Autonomous (name = "AutoRedClose", group = "Main")
public class RedClose extends LinearOpMode {

    Bot robot = new Bot();

    //STEP autoStep;

    COLORS backJewel;

    int robo;
    @Override
    public void runOpMode() {

        robot.init(hardwareMap, telemetry);

        robo = 0;

        waitForStart();

        /*
                    robot.jewelArm.setArmDown();
                    sleep(1000);
                    backJewel = robot.jewelArm.getBackJewel();

                    if (backJewel == BLUE) {

                        robo = 1;

                    } else if (backJewel == RED) {

                        robo = 2;

                    } else {

                        robo = 3;

                    }
                    break;
                }


                    robot.drive.moveForward();
                    sleep(750);
                    robot.drive.stopMovement();
                    robot.jewelArm.setArmUp();

                    robot.drive.moveBackward();
                    sleep(750);
                    robot.drive.stopMovement();
                    robot.jewelArm.setArmUp();
        */


        /*BLUE
        robot.drive.moveForward();
        sleep(1000);
        robot.drive.stopMovement();
        robot.drive.setTurnPower(-0.4);
        sleep(350);
        robot.drive.stopMovement();
        robot.drive.moveForward();
        sleep(1000);
        robot.drive.stopMovement();
        robot.drive.moveBackward();
        sleep(1000);
        robot.drive.moveForward();
        sleep(1000);
        robot.drive.moveBackward();
        sleep(100);
        robot.drive.stopMovement();
        */

        //PARK RED
        robot.drive.moveForward();
        sleep(1250);
        robot.drive.stopMovement();
        robot.drive.setTurnPower(0.4);
        sleep(1000);
        robot.drive.stopMovement();
        robot.drive.moveForward();
        sleep(500);
        robot.drive.moveBackward();
        sleep(250);
        robot.drive.stopMovement();



    }
}
