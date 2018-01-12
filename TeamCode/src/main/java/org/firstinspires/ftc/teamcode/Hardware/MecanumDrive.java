package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by mcshirt on 11/21/17.
 */

public class MecanumDrive {

    private DcMotor FL, FR, BL, BR;

    private double powerMultiplier;

    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: TETRIX Motor Encoder
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV) /
            (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.5;

            public MecanumDrive(){


            }

            public void init(HardwareMap hardwareMap, Telemetry telemetry) {
                this.initialize(hardwareMap, telemetry);
            }


            private void initialize(HardwareMap hardwareMap, Telemetry telemetry) {

                FL = hardwareMap.get(DcMotor.class, "leftFront");
                FR = hardwareMap.get(DcMotor.class, "rightFront");
                BL = hardwareMap.get(DcMotor.class, "leftBack");
                BR = hardwareMap.get(DcMotor.class, "rightBack");

                FL.setDirection(DcMotorSimple.Direction.REVERSE);
                BL.setDirection(DcMotorSimple.Direction.REVERSE);

                DcMotor.ZeroPowerBehavior zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE;

                FL.setZeroPowerBehavior(zeroPowerBehavior);
                FR.setZeroPowerBehavior(zeroPowerBehavior);
                BL.setZeroPowerBehavior(zeroPowerBehavior);
                BR.setZeroPowerBehavior(zeroPowerBehavior);

                powerMultiplier = 1;

                telemetry.addData("Drive: ", "Done");
            }

            public void setMotorMode(DcMotor.RunMode motorMode){

                FL.setMode(motorMode);
                FR.setMode(motorMode);

            }

            public void setTurnPower(double power) {

                FL.setPower(power);
                BL.setPower(power);
                FR.setPower(-power);
                BR.setPower(-power);
            }

            public void setThrottle(double power) {

                FL.setPower(power);
                FR.setPower(power);
                BR.setPower(power);
                BL.setPower(power);

            }

            public void forwardDriveEncoder(int rotations) {

                int targetTicks;

                targetTicks = 1120 * rotations;

                FR.setTargetPosition(targetTicks);

                FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                FR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                // Wait for encoders reset
                while (FR.getCurrentPosition() != 0){}

                setThrottle(0.4);

                while (FR.isBusy()) {}

                setThrottle(0);


            }

            public void moveForward(){

                setThrottle(DRIVE_SPEED);

            }
            public void moveBackward(){

                setThrottle(-DRIVE_SPEED);

            }
			public void turn(){
			
				setTurnPower(TURN_SPEED);
				
			}
            public void stopMovement(){

                 setThrottle(0);

            }

            public void strafe(int direction, double power){

                // 1 for right, -1 for left

                FL.setPower(1 * direction * power); // ik its the same thing but it looks nicer
                BL.setPower(-1 * direction * power);
                FR.setPower(-1 * direction * power);
                BR.setPower(1 * direction * power);
            }

            public void mecanumDrive(float left_stick_y, float right_stick_x, float left_stick_x, double powerMultiplier){

                float[] motorVals = {0, 0, 0, 0};

                motorVals[0] = left_stick_y + right_stick_x + left_stick_x;
                motorVals[1] = left_stick_y + right_stick_x - left_stick_x;
                motorVals[2] = left_stick_y - right_stick_x + left_stick_x;
                motorVals[3] = left_stick_y - right_stick_x - left_stick_x;

                //Adjust range to that allowed by DcMotors
                motorVals = mapValues(motorVals, -1, 1);
                //Set power to motors11
                FL.setPower(motorVals[0] * powerMultiplier);
                BL.setPower(motorVals[1] * powerMultiplier);
                BR.setPower(motorVals[2] * powerMultiplier);
                FR.setPower(motorVals[3] * powerMultiplier);

            }
            public void tankDrive(float left_stick_y, float right_stick_x, double powerMultiplier){

                float[] motorVals = {0, 0, 0, 0};

                motorVals[0] = left_stick_y + right_stick_x;
                motorVals[1] = left_stick_y + right_stick_x;
                motorVals[2] = left_stick_y - right_stick_x;
                motorVals[3] = left_stick_y - right_stick_x;

                //Adjust range to that allowed by DcMotors
                motorVals = mapValues(motorVals, -1, 1);
                //Set power to motors11
                FL.setPower(motorVals[0] * powerMultiplier);
                BL.setPower(motorVals[1] * powerMultiplier);
                BR.setPower(motorVals[2] * powerMultiplier);
                FR.setPower(motorVals[3] * powerMultiplier);

            }

            public float[] mapValues(float[] a, double minR, double maxR) {
                float min = Float.MAX_VALUE;
                float max = Float.MIN_VALUE;
                for (int i = 0; i <= 3; i++) {
                    if (a[i] < min) {
                        min = a[i];
                    }
                    if (a[i] > max) {
                        max = a[i];
                    }
                }
                if (min >= -1 && max <= 1) {
                    return a;
                }
                double scalemin = Math.abs(minR / min);
                double scalemax = Math.abs(minR / max);
                double finalscale;

                if (scalemin < scalemax)
                    finalscale = scalemin;
                else
                    finalscale = scalemax;
                for (int i = 0; i <= 3; i++) {
                    a[i] = a[i] * (float) finalscale;
                }

                return a;
            }

}
