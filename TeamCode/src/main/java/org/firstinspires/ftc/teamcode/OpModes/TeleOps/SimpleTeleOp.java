package org.firstinspires.ftc.teamcode.OpModes.TeleOps;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.DriveTrain;
import org.firstinspires.ftc.teamcode.Hardware.Robot;

@TeleOp(name = "SimpleTeleOp", group = "simple")
public class SimpleTeleOp extends OpMode {

    Robot robot = new Robot();

    @Override
    public void init() {
        // wow Nick C sucks
        robot.init(hardwareMap, telemetry, DriveTrain.DriveTypes.TANK);

    }

    @Override
    public void loop() {

        robot.drive.manualDrive(gamepad1);
        robot.glyphSystem.controlSystem(gamepad2);

    }

}
