package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Bot;

/**
 * Created by mcshirt on 11/29/17.
 */
@Autonomous (name = "AutoRedFar", group = "Main")
public class RedFar extends LinearOpMode {
    
    enum JewelColor{
        
        BLUE, RED, UNDEFINED
           
    }
    enum Steps{
     
        DROPANDCHECK, KNOCKBACKJEWEL, KNOCKFRONTJEWEL, MOVETOSAFEZONE, PUTGLYPHIN
        
    }
    
    Bot robot = new Bot();

    private STEP autoStep;

    COLORS backJewel;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap, telemetry);

        autoStep = DROPANDCHECK;
        
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
                    autoStep = MOVETOSAFEZONE;
                    break;
                }
                case KNOCKBACKJEWEL: {

                    robot.drive.moveBackward();
                    sleep(750);
                    robot.drive.stopMovement();
                    robot.jewelArm.setArmUp();
                    autoStep = MOVETOSAFEZONE;
                    break;
                }
                case MOVETOSAFEZONE: {
                    
                    robot.drive.moveForward();
                    sleep(1000);
                    robot.drive.stopMovement();
                    
                }


            }

        }
    }
}
