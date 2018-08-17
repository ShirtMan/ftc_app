package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Robot{

    Telemetry telemetry;

    public DriveTrain drive = new DriveTrain(telemetry);
    public GlyphSystem glyphSystem = new GlyphSystem();


    public Robot() {

    }

    public void init(HardwareMap hardwareMap, Telemetry telemetry, DriveTrain.DriveTypes type){
        this.initialize(hardwareMap, telemetry, type);
    }

    private void initialize(HardwareMap hardwareMap, Telemetry telemetry, DriveTrain.DriveTypes type) {

        drive.init(hardwareMap, telemetry, type);
        glyphSystem.init(hardwareMap, telemetry);

    }

}
