package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by mcshirt on 11/21/17.
 */

public class MecanumDrive {

            public DcMotor FL, FR, BL, BR;

            public double powerMultiplier;

    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
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

                FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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

            public void forwardDriveInches(int inches) {
                int targetTicks = 0;

                targetTicks = 1440 * inches;

                FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                // Wait for encoders reset
                while (FR.getCurrentPosition() != 0){}

                setThrottle(0.4);

                while (FR.getCurrentPosition() < targetTicks) {}

                setThrottle(0);


            }


            public DcMotor getFL(){


                return FL;
            }
            public DcMotor getFR(){


                return FR;
            }
            public DcMotor getBL(){


                return BL;
            }
            public DcMotor getBR(){


                return BR;
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
