package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Bot;

/**
 * Created by mcshirt on 11/21/17.
 */

@Autonomous(name = "EncoderTest")
public class EncoderTest extends LinearOpMode {

    Bot robot = new Bot();

    private int robo = 0;

    private ElapsedTime runtime = new ElapsedTime();



    @Override
    public void runOpMode() {

        robot.init(hardwareMap, telemetry);

        waitForStart();

        while (opModeIsActive()) {
            switch (robo) {

                case 0: {

                    encoderDrive(0.5,  48,  48, 5.0);  // S1: Forward 47 Inches with 5 Sec timeout
                    encoderDrive(0.6,   12, -12, 4.0);  // S2: Turn Right 12 Inches with 4 Sec timeout
                    encoderDrive(0.5, -24, -24, 4.0); // S3: Reverse 24 Inches with 4 Sec timeout

                    robo++;
                    break;
                }
                case 1: {

                    robot.glyphGrabber.closeGrabber();

                    break;
                }
            }
        }
    }


    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;
/*
        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.drive.getFL().getCurrentPosition() + (int) (leftInches * robot.drive.COUNTS_PER_INCH);
            newRightTarget = robot.drive.getFR().getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            robot.drive.getFL().setTargetPosition(newLeftTarget);
            robot.drive.getFR().setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.drive.setMotorMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.drive.setThrottle(Math.abs(speed));
*/

            // Stop all motion;
            robot.drive.setThrottle(0);

            // Turn off RUN_TO_POSITION
            robot.drive.setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move

    }
}
