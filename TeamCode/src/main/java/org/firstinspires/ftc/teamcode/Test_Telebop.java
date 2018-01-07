package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by mcshirt on 1/1/18.
 */

@TeleOp (name = "Test - Telebop", group = "Test")
public class Test_Telebop extends OpMode{

    Bot robot = new Bot();

    BNO055IMU imu;
    Orientation angles;

    double powerMultiplier = 1;
    double currentAngle = 0;
    int targetAngle;

    @Override
    public void init() {

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

        targetAngle = 80;

        int index = 0;
    }

    @Override
    public void loop() {

        robot.drive.tankDrive(gamepad1.left_stick_y, gamepad1.right_stick_y, powerMultiplier);

        if (gamepad2.right_stick_button){
            robot.glyphLifter.encoderDrive(0.5, (1800/3), DcMotorSimple.Direction.FORWARD);
        }
        if (gamepad2.left_stick_button){
            robot.glyphLifter.encoderDrive(0.5, (1800/3), DcMotorSimple.Direction.REVERSE);
        }


        if (gamepad1.y){
            currentAngle = 0;
            targetAngle = 80;
        } else {
            currentAngle = getCurrentAngle();
        }

        if (gamepad1.right_bumper) {
            moveToAngle();
        }

        telemetry.addData("Angles", currentAngle);
        telemetry.update();

    }

    public void moveToAngle(){

        robot.drive.setTurnPower(-0.4);

        if (getCurrentAngle() <= targetAngle + 1 && getCurrentAngle() >= targetAngle - 1){

            robot.drive.stopMovement();
            telemetry.addData("TURN: ", "DONE");

        } else if (getCurrentAngle() > targetAngle){

            robot.drive.setTurnPower(0.2);

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
