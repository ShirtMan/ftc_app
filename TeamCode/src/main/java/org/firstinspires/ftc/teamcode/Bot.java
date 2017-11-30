package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.GlyphGrabber;
import org.firstinspires.ftc.teamcode.Hardware.GlyphLifter;
import org.firstinspires.ftc.teamcode.Hardware.JewelArm;
import org.firstinspires.ftc.teamcode.Hardware.MecanumDrive;

/**
 * Created by mcshirt on 11/21/17.
 */

//Chap-E
public class Bot {

    public GlyphLifter glyphLifter = new GlyphLifter();
    public GlyphGrabber glyphGrabber = new GlyphGrabber();
    public MecanumDrive drive = new MecanumDrive();
    public JewelArm jewelArm = new JewelArm();

    public Bot(){

    }
    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.initialize(hardwareMap, telemetry);
    }


    private void initialize(HardwareMap hardwareMap, Telemetry telemetry) {

        glyphLifter.init(hardwareMap, telemetry);
        glyphGrabber.init(hardwareMap, telemetry);
        drive.init(hardwareMap, telemetry);
        jewelArm.init(hardwareMap, telemetry);

        telemetry.update();

    }

}
