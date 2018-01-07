package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.teamcode.Bot;
import org.firstinspires.ftc.teamcode.ENUM.COLORS;

@Autonomous (name = "VuRedFar", group = "Main")
public class VuRedFar extends LinearOpMode {

    Bot robot = new Bot();
    private long timeToTurn;

    VuforiaLocalizer vuforia;

    @Override
    public void runOpMode() {

        robot.init(hardwareMap, telemetry);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = "AWBfzdD/////AAAAGVCvW8Pbe006tYtykMBPXLVPBGYjqVQYwwmptNffpYDItCTleQ3m5gWmb1lQOp3QOLrJ4H+wzdZjUmfNezFQ+zt7IhpvjTTgDJAc8ZW9NdtY/FrEykrOl80iaRK7RG4HxC2J7rwmoENM1LiwBr/K6i+YMHMIEnA1YlnYRSLynP2Juv7316t7stmeZ7SrwOYTe8IP1OVPpHZrUYQDdIFEam9JQDpCtccK11260ILLuLej+80tAokdu+nJh++uCrP1AT2nLEm9WaIjEbOv8efXpa5xKCuKze9SxnIIJYM4VNeV7GwCzUu7mneqtU0BRFt5XRn6viWvMHpCnc67LNzdxZZNi3fQESobUWpiwu8WZ0Ep";

        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);

        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        waitForStart();

        robot.drive.stopMovement();
        robot.glyphGrabber.closeGrabber();
        sleep(500);
        robot.glyphLifter.moveLift(0.4, 0);
        sleep(1000);
        robot.glyphLifter.stopMovement();


        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
        if (vuMark == RelicRecoveryVuMark.LEFT) {
           timeToTurn = 1000;
        } else if (vuMark == RelicRecoveryVuMark.CENTER) {
            timeToTurn = 750;
        } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
            timeToTurn = 500;
        } else {
            telemetry.addData("VuMark", "not visible");
        }

        telemetry.update();

        //TODO: JEWEL
/*
        robot.jewelArm.setArmDown();
        if (robot.jewelArm.getBackJewel() == COLORS.BLUE) {
            turnForTime(0.2, 500);
            turnForTime(-0.2, 500);
        } else if (robot.jewelArm.getBackJewel() == COLORS.RED){
            turnForTime(-0.2, 500);
            turnForTime(0.2, 500);
        }*/
        turnForTime(-0.2, timeToTurn);
        moveForTime(1000);
        robot.glyphGrabber.openGrabber();
        robot.drive.moveBackward();
        sleep(750);
        robot.drive.stopMovement();
        robot.glyphGrabber.closeGrabber();
        sleep(500);
        robot.drive.setThrottle(0.3);
        sleep(2000);
        robot.drive.stopMovement();
        sleep(500);
        robot.drive.moveBackward();
        sleep(250);
        robot.drive.stopMovement();

    }

    private void moveForTime(long timeToSleep){

        robot.drive.moveForward();
        sleep(timeToSleep);
        robot.drive.stopMovement();

    }
    private void turnForTime(double turnPower, long timeToSleep){

        robot.drive.setTurnPower(turnPower);
        sleep(timeToSleep);
        robot.drive.stopMovement();

    }

    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
}
