package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Bot;

/**
 * Created by mcshirt on 12/28/17.
 */
@Autonomous (name = "Lifter + Encoder = Lit", group = "TestAuto")
public class EncoderLifter extends LinearOpMode {

    DcMotor smallLifter, frontLifter;
    int position;
    Bot robot = new Bot();

    @Override
    public void runOpMode() {

        robot.init(hardwareMap, telemetry);

        position = 0;

        /*frontLifter = hardwareMap.dcMotor.get("glyphLifter");
        smallLifter = hardwareMap.dcMotor.get("smallLifter");

        frontLifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        smallLifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        smallLifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);*/

        waitForStart();

        robot.glyphLifter.encoderDrive(0.5, 1800, DcMotorSimple.Direction.FORWARD);
        sleep(1000);
        robot.glyphLifter.encoderDrive(0.5, 1800, DcMotorSimple.Direction.REVERSE);

    }

    public void encoderDrive(double speed,
                             int newPos, DcMotorSimple.Direction direction) {

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            smallLifter.setDirection(direction);
            frontLifter.setDirection(direction);

            smallLifter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            smallLifter.setTargetPosition(newPos);


            // Turn On RUN_TO_POSITION
            smallLifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            smallLifter.setPower(Math.abs(speed));
            frontLifter.setPower(Math.abs(speed));
            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (smallLifter.isBusy())) {

                // Display it for the driver.
                telemetry.addData("Pos: ", smallLifter.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            smallLifter.setPower(0);
            frontLifter.setPower(0);

            // Turn off RUN_TO_POSITION
            smallLifter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }
}
