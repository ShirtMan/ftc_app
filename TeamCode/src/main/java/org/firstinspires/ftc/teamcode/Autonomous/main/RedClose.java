package org.firstinspires.ftc.teamcode.Autonomous.main;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
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
import org.firstinspires.ftc.teamcode.Hardware.MecanumDrive;

import static org.firstinspires.ftc.teamcode.ENUM.COLORS.BLUE;
import static org.firstinspires.ftc.teamcode.ENUM.COLORS.RED;
import static org.firstinspires.ftc.teamcode.ENUM.STEP.KNOCKBACKJEWEL;
import static org.firstinspires.ftc.teamcode.ENUM.STEP.KNOCKFRONTJEWEL;
import static org.firstinspires.ftc.teamcode.ENUM.STEP.KNOCKJEWEL;
import static org.firstinspires.ftc.teamcode.ENUM.STEP.MOVETOSAFEZONE;

/**
 * Created by mcshirt on 11/29/17.
 */
@Autonomous (name = "AutoRedCloserThanEver", group = "Main")
public class RedClose extends LinearOpMode {

    MecanumDrive drive = new MecanumDrive();

    DcMotor intakeLeft, intakeRight, lifter, flipper;

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

        drive.init(hardwareMap, telemetry);

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

        hitter.setPosition(0);
        arm.setPosition(0);

        intakeLeft = hardwareMap.dcMotor.get("intakeLeft");
        intakeRight = hardwareMap.dcMotor.get("intakeRight");

        lifter = hardwareMap.dcMotor.get("lifter");

        flipper = hardwareMap.dcMotor.get("flipper");

        intakeLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intakeRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intakeRight.setDirection(DcMotorSimple.Direction.REVERSE);

        lifter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lifter.setDirection(DcMotorSimple.Direction.REVERSE);

        drive.init(hardwareMap, telemetry);

        telemetry.addData("INIT: ", "WE ROCKIN");

        waitForStart();

        hitter.setPosition(0.5);
        sleep(1000);
        arm.setPosition(0.75);
        sleep(500);
        arm.setPosition(0.35);
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
        arm.setPosition(0.2);
        sleep(250);
        arm.setPosition(0);
        sleep(250);
        hitter.setPosition(1);
        sleep(2000);

        drive.setThrottle(0.5);
        sleep(2000);
        drive.stopMovement();

        moveToAngle(90);

        drive.setThrottle(0.25);
        sleep(1000);
        drive.stopMovement();

        changeIntakePower(-1);
        sleep(750);
        changeIntakePower(0);

        moveToAngle(180);

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

        drive.setTurnPower(0.20);

        while(opModeIsActive() && !turnDone) {
            if (getCurrentAngle() <= targetAngle + 1 && getCurrentAngle() >= targetAngle - 1) {

                drive.stopMovement();
                telemetry.addData("TURN: ", "DONE");
                turnDone = true;

            } else if (getCurrentAngle() < targetAngle){

                drive.setTurnPower(-0.20);

            }
        }
    }

    public void changeIntakePower(double power){

        intakeLeft.setPower(power);
        intakeRight.setPower(power);

    }
}
