package org.firstinspires.ftc.teamcode.Autonomous.main;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Bot;

/**
 * Created by mcshirt on 11/29/17.
 */
@Autonomous (name = "AutoRedFartherThanEVer", group = "Main")
public class RedFar extends LinearOpMode {

    Bot robot = new Bot();

    Servo hitter, arm;

    ColorSensor colorSensor;

    BNO055IMU imu;
    Orientation angles;

    boolean turnDone = false;

    @Override
    public void runOpMode() {

        telemetry.addData("INIT: ", "NO ROCK");

        robot.init(hardwareMap,telemetry);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "REVGyroCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        hitter = hardwareMap.servo.get("jHitter");
        arm = hardwareMap.servo.get("jArm");

        colorSensor = hardwareMap.colorSensor.get("colorSensor");

        arm.setPosition(0.1);

        telemetry.addData("INIT: ", "WE ROCKIN");

        waitForStart();

        robot.glyphGrabber.closeGrabber();
        sleep(500);
        robot.glyphLifter.encoderDrive(0.5, 1400, DcMotorSimple.Direction.FORWARD);

        hitter.setPosition(0.5);
        sleep(1000);
        arm.setPosition(0.4);
        sleep(500);
        arm.setPosition(0.6);
        sleep(2000);
        telemetry.addData("Color Data: ", colorSensor.blue() + " " + colorSensor.red());

        if (colorSensor.red() > colorSensor.blue()){

            telemetry.addData("JEWEL: ", "RED");
            hitter.setPosition(1);

        }
        if (colorSensor.blue() > colorSensor.red()) {

            telemetry.addData("JEWEL: ", "BLUE");
            hitter.setPosition(0);
        }
        sleep(2000);
        telemetry.update();
        hitter.setPosition(0.5);
        arm.setPosition(0.4);
        sleep(250);
        arm.setPosition(0.1);
        sleep(250);
        hitter.setPosition(0);
        sleep(2000);

        robot.drive.setThrottle(0.3);
        sleep(1000);
        robot.drive.stopMovement();

        robot.drive.strafe(-1, 0.75);
        sleep(800);
        robot.drive.stopMovement();

       // moveToAngle(startAngle);

        sleep(100);
        robot.drive.setThrottle(0.3);
        sleep(1000);
        robot.drive.stopMovement();
        robot.glyphGrabber.openGrabber();
        robot.drive.moveBackward();
        sleep(500);
        robot.drive.stopMovement();
        robot.glyphGrabber.closeGrabber();
        sleep(500);
        robot.glyphLifter.encoderDrive(0.5, 1000, DcMotorSimple.Direction.REVERSE);
        sleep(200);
        robot.drive.setThrottle(0.3);
        sleep(2000);
        robot.drive.stopMovement();
        sleep(500);
        robot.drive.moveBackward();
        sleep(250);
        robot.drive.stopMovement();

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

    public void moveToAngle(double targetAngle){

        robot.drive.setTurnPower(-0.1);

        while(opModeIsActive() && !turnDone) {
            if (getCurrentAngle() <= targetAngle + 1 && getCurrentAngle() >= targetAngle - 1) {

                robot.drive.stopMovement();
                telemetry.addData("TURN: ", "DONE");
                turnDone = true;

            } else if (getCurrentAngle() < targetAngle){

                robot.drive.setTurnPower(0.1);

            }
        }
    }
}
