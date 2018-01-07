package org.firstinspires.ftc.teamcode.Autonomous;

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
 * Created by mcshirt on 11/29/17.
 */
@Autonomous (name = "AutoRedClose", group = "Main")
public class RedClose extends LinearOpMode {


    Bot robot = new Bot();

    ColorSensor colorSensor;
    Servo hitter, arm;

    VuforiaLocalizer vuforia;

    BNO055IMU imu;
    Orientation angles;

    double angle;
    double nowAngle;

    boolean firstTurn;
    boolean turnDone;

    @Override
    public void runOpMode() {

        turnDone = false;

        robot.init(hardwareMap, telemetry);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters vuParameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        vuParameters.vuforiaLicenseKey = "AWBfzdD/////AAAAGVCvW8Pbe006tYtykMBPXLVPBGYjqVQYwwmptNffpYDItCTleQ3m5gWmb1lQOp3QOLrJ4H+wzdZjUmfNezFQ+zt7IhpvjTTgDJAc8ZW9NdtY/FrEykrOl80iaRK7RG4HxC2J7rwmoENM1LiwBr/K6i+YMHMIEnA1YlnYRSLynP2Juv7316t7stmeZ7SrwOYTe8IP1OVPpHZrUYQDdIFEam9JQDpCtccK11260ILLuLej+80tAokdu+nJh++uCrP1AT2nLEm9WaIjEbOv8efXpa5xKCuKze9SxnIIJYM4VNeV7GwCzUu7mneqtU0BRFt5XRn6viWvMHpCnc67LNzdxZZNi3fQESobUWpiwu8WZ0Ep";

        vuParameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(vuParameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary


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

        int threshold = 10;

        firstTurn = true;

        colorSensor = hardwareMap.colorSensor.get("colorSensor");
        hitter = hardwareMap.servo.get("jHitter");
        arm = hardwareMap.servo.get("jArm");

        hitter.setPosition(0);
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
        arm.setPosition(0.55);
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
        hitter.setPosition(0);
        sleep(2000);


        robot.drive.setThrottle(0.2);
/*
        RelicRecoveryVuMark vuMark a= RelicRecoveryVuMark.from(relicTemplate);

        while (opModeIsActive() && (vuMark == null || vuMark == RelicRecoveryVuMark.UNKNOWN)){

            telemetry.addData("WE ", "GOIN");

        }

        // either do some crappy angle crap or do sleeps OR encoders?

        if (vuMark == RelicRecoveryVuMark.LEFT) {
            threshold = 30;
        } else if (vuMark == RelicRecoveryVuMark.CENTER) {
            threshold = 55;
        } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
            threshold = 80;
        }
*/
        //threshold = -100;
        sleep(2500);
        robot.drive.stopMovement();

        moveToAngle(-88);

        robot.drive.setThrottle(0.4);
        sleep(1000);
        robot.drive.stopMovement();
        robot.glyphGrabber.openGrabber();
        sleep(750);
        robot.drive.setThrottle(-0.2);
        sleep(1000);
        robot.drive.stopMovement();
        robot.glyphGrabber.closeGrabber();
        robot.glyphLifter.encoderDrive(0.5, 1400, DcMotorSimple.Direction.REVERSE);
        robot.drive.setThrottle(0.2);
        sleep(1000);
        robot.drive.stopMovement();
        robot.drive.setThrottle(-0.3);
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

        robot.drive.setTurnPower(0.25);

        while(opModeIsActive() && !turnDone) {
            if (getCurrentAngle() <= targetAngle + 1 && getCurrentAngle() >= targetAngle - 1) {

                robot.drive.stopMovement();
                telemetry.addData("TURN: ", "DONE");
                turnDone = true;

            } else if (getCurrentAngle() < targetAngle){

                robot.drive.setTurnPower(0.25);

            }
        }
    }
}
