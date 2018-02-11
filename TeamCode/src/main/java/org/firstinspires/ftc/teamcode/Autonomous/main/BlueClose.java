package org.firstinspires.ftc.teamcode.Autonomous.main;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
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
 * 7571 Made by Nick.
 * If you took this code from GitHub,
 * my only request is that you make it better
 */

@Autonomous (name = "AutoBlueCloserThanEver", group = "Main")
public class BlueClose extends LinearOpMode {

    Bot robot = new Bot();

    ColorSensor colorSensor;
    Servo hitter, arm;

    BNO055IMU imu;
    Orientation angles;

    double angle;
    double nowAngle;

    boolean firstTurn;
    boolean turnDone;

    @Override
    public void runOpMode() {

        telemetry.addData("INIT: ", "NO ROCK");

        turnDone = false;

        robot.init(hardwareMap, telemetry);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "REVGyroCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        angle = 0;
        nowAngle = getCurrentAngle();

        firstTurn = true;

        colorSensor = hardwareMap.colorSensor.get("colorSensor");
        hitter = hardwareMap.servo.get("jHitter");
        arm = hardwareMap.servo.get("jArm");

        hitter.setPosition(1);
        arm.setPosition(0);


        telemetry.addData("INIT: ", "WE ROCKIN");

        waitForStart();

        robot.glyphGrabber.closeGrabber();
        sleep(500);
        robot.glyphLifter.encoderDrive(0.5, 1400, DcMotorSimple.Direction.FORWARD);

        hitter.setPosition(0.5);
        sleep(1000);
        arm.setPosition(0.4);
        sleep(500);
        arm.setPosition(1);
        sleep(2000);
        telemetry.addData("Color Data: ", colorSensor.blue() + " " + colorSensor.red());

        if (colorSensor.red() > colorSensor.blue()){

            telemetry.addData("JEWEL: ", "RED");
            hitter.setPosition(0);

        }
        if (colorSensor.blue() > colorSensor.red()) {

            telemetry.addData("JEWEL: ", "BLUE");
            hitter.setPosition(1);
        }
        sleep(2000);
        telemetry.update();
        hitter.setPosition(0.5);
        arm.setPosition(0.4);
        sleep(250);
        arm.setPosition(0.2);
        sleep(250);
        arm.setPosition(0);
        sleep(250);
        hitter.setPosition(1);
        sleep(2000);


        robot.drive.setThrottle(0.4);
        sleep(2000);
        robot.drive.stopMovement();

        moveToAngle(-90);

        sleep(500);
        robot.drive.setThrottle(-0.4);
        sleep(1000);
        robot.drive.stopMovement();
        robot.glyphGrabber.openGrabber();
        sleep(750);
        robot.drive.setThrottle(0.2);
        sleep(1000);
        robot.drive.stopMovement();
        robot.glyphGrabber.closeGrabber();
        robot.glyphLifter.encoderDrive(0.5, 1400, DcMotorSimple.Direction.REVERSE);
        robot.drive.setThrottle(-0.2);
        sleep(1000);
        robot.drive.stopMovement();
        robot.drive.setThrottle(0.3);
        sleep(500);
        robot.drive.stopMovement();

        sleep(1000);
        stop();

    }



    public double getCurrentAngle(){
        double anglesNorm;
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        anglesNorm = NormalizeDegrees(angles.angleUnit , angles.firstAngle);
        return anglesNorm;
    }

    public double NormalizeDegrees(AngleUnit angleUnit, double angle){
        double degrees;
        degrees = AngleUnit.DEGREES.fromUnit(angleUnit, angle);
        degrees = AngleUnit.DEGREES.normalize(degrees);
        return degrees;
    }

    public void moveToAngle(int targetAngle){

        robot.drive.setTurnPower(-0.20);

        while(opModeIsActive() && !turnDone) {
            if (getCurrentAngle() <= targetAngle + 1 && getCurrentAngle() >= targetAngle - 1) {

                robot.drive.stopMovement();
                telemetry.addData("TURN: ", "DONE");
                turnDone = true;

            } else if (getCurrentAngle() < targetAngle){

                robot.drive.setTurnPower(0.20);

            }
        }
    }
}
