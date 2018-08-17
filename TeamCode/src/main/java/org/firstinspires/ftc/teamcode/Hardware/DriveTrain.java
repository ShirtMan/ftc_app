package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class DriveTrain extends BaseHardware {

    Telemetry telemetry;

    public DriveTrain(Telemetry tele) {
        this.telemetry = tele;
    }

    public enum DriveTypes{
        MECANUM, STANDARD, TANK, SIXWHEEL
    }

//---------------------------------------------------------------------------------------

    DriveTypes driveType;

    public DcMotor FL, FR, BL, BR;


    public DriveTrain(DriveTypes driveType){

        this.driveType = driveType;

    }

    public void init(HardwareMap hardwareMap, Telemetry telemetry, DriveTypes type){
        this.initialize(hardwareMap, type, telemetry);
    }

//----------------------------------------------------------------------------------------

    private void initialize(HardwareMap hardwareMap, DriveTypes type, Telemetry telemetry){

        driveType = type;

        FL = hardwareMap.dcMotor.get("fl");
        FR = hardwareMap.dcMotor.get("fr");
        BL = hardwareMap.dcMotor.get("bl");
        BR = hardwareMap.dcMotor.get("br");

        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        FL.setDirection(DcMotorSimple.Direction.REVERSE);
        BL.setDirection(DcMotorSimple.Direction.REVERSE);

        switch (driveType) {
            case MECANUM: {
                telemetry.addLine("MECANUM");
                break;
            }
            case STANDARD: {
                telemetry.addLine("STANDARD");
                break;
            }
        }

        telemetry.update();


    }

//------------------------------------------------------------------------------------------


    public void setThrottle(double power){

        FL.setPower(power);
        FR.setPower(power);
        BL.setPower(power);
        BR.setPower(power);

    }


    public void strafe(double power){

        FL.setPower(-power);
        FR.setPower(-power);
        BL.setPower(power);
        BR.setPower(power);

    }

    //-----------------------------------------------------------------------------

    public void resetUpEncoders(){

        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        BR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }

    public void runMotorEncoders(int ticks){

        FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FL.setTargetPosition(ticks);
        FL.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    public void checkMotors(double power){

        telemetry.addLine("IsBusy?: " + FL.isBusy());
        telemetry.addLine("Mode?: " + FL.getMode() + FR.getMode() + BL.getMode() + BR.getMode());

        if (FL.isBusy()){
            FL.setPower(power);
            telemetry.addLine("settingPower");
        } else {
            FL.setPower(0);
            FL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            telemetry.addLine("stopping");
        }

        telemetry.update();

    }

    public void setMotorModes(DcMotor.RunMode runMode){


    }

    //-----------------------------------------------------------------------------------------------------------------

    public void manualDrive(Gamepad gamepad){

        switch (driveType){

            case MECANUM:

                FL.setPower(gamepad.left_stick_y + gamepad.right_stick_y + gamepad.left_stick_x);
                FR.setPower(gamepad.left_stick_y - gamepad.right_stick_y + gamepad.left_stick_x);
                BL.setPower(gamepad.left_stick_y + gamepad.right_stick_y - gamepad.left_stick_x);
                BR.setPower(gamepad.left_stick_y - gamepad.right_stick_y - gamepad.left_stick_x);

                break;

            case TANK:

                if (gamepad.left_bumper){

                    strafe(1);

                } else if (gamepad.right_bumper){

                    strafe(-1);

                } else {

                    FL.setPower(gamepad.left_stick_y + gamepad.right_stick_y);
                    FR.setPower(gamepad.left_stick_y - gamepad.right_stick_y);
                    BL.setPower(gamepad.left_stick_y + gamepad.right_stick_y);
                    BR.setPower(gamepad.left_stick_y - gamepad.right_stick_y);

                }
        }
    }
}
