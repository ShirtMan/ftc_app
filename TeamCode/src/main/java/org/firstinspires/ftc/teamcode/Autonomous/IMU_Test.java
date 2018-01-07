package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Bot;

/**
 * Created by mcshirt on 12/30/17.
 */
@Autonomous (name = "RED CLOSE?!?!?!?!", group = "Test")
public class IMU_Test extends LinearOpMode{


    Bot robot = new Bot();

    ColorSensor colorSensor;
    Servo hitter, arm;

    VuforiaLocalizer vuforia;

    BNO055IMU imu;
    Orientation angles;

    double angle;
    double nowAngle;

    boolean firstTurn;

    @Override
    public void runOpMode() {

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


        telemetry.addData("INIT: ", "WE ROCKIN");

        waitForStart();

        robot.drive.setThrottle(0.3);
        sleep(1500);
        robot.drive.setThrottle(0);

        sleep(1000);

        threshold = -90;
        
        if (getCurrentAngle() >= threshold - 1 && getCurrentAngle() <= threshold + 1) {

            robot.drive.stopMovement();
            firstTurn = false;

        } else if (getCurrentAngle() > threshold) {

            robot.drive.setTurnPower(0.1);

        }
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
}
