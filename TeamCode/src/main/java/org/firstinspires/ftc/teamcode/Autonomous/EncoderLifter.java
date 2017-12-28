package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by mcshirt on 12/28/17.
 */
@Autonomous (name = "Lifter + Encoder = Lit", group = "TestAuto")
public class EncoderLifter extends LinearOpMode{

    DcMotor lifter;

    @Override
    public void runOpMode() throws InterruptedException {

        lifter = hardwareMap.dcMotor.get("glyphLifter");
        lifter.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        waitForStart();

        lifter.setTargetPosition(1440/2);
        lifter.setPower(0.5);
    }
}
