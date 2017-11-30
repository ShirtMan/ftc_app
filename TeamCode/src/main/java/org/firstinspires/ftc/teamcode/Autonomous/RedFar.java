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
@Autonomous (name = "AutoRedFar", group = "Main")
public class RedFar extends LinearOpMode {

    Bot robot = new Bot();

    private STEP autoStep;

    COLORS backJewel;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap, telemetry);

        waitForStart();

        while(opModeIsActive()) {

            switch (autoStep) {

                case DROPANDCHECK: {

                    backJewel = robot.jewelArm.getBackJewel();

                    if (backJewel == BLUE) {

                        autoStep = KNOCKBACKJEWEL;

                    } else if (backJewel == RED) {

                        autoStep = KNOCKFRONTJEWEL;

                    } else {

                        autoStep = MOVETOSAFEZONE;

                    }
                    break;
                }
                case KNOCKFRONTJEWEL: {


                    robot.drive.moveForward();
                    sleep(750);
                    robot.drive.stopMovement();
                    robot.jewelArm.setArmUp();

                }
                case KNOCKBACKJEWEL: {

                    robot.drive.moveBackward();
                    sleep(750);
                    robot.drive.stopMovement();
                    robot.jewelArm.setArmUp();

                }


            }

        }
    }
}
