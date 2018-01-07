
package ComputerVision;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.*;
import com.disnodeteam.dogecv.filters.LeviColorFilter;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Bot;
import org.opencv.core.Point;
import org.opencv.core.Size;

import java.io.IOException;


@Autonomous(name="DogeCV Relic/Generic Detector", group="DogeCV")

public class RelicOpMode extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();

    Bot robot = new Bot();

    private GenericDetector genericDetector = null;
    /*
     * Code to run ONCE when the driver hits INIT
     */
    Point locationOfBlob;
    double xOfBlob;

    boolean runningToTarget = false;

    @Override
    public void init() {
        robot.init(hardwareMap, telemetry);

        genericDetector = new GenericDetector();
        genericDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());
        genericDetector.colorFilter = new LeviColorFilter(LeviColorFilter.ColorPreset.RED);
        //genericDetector.colorFilter = new HSVColorFilter(new Scalar(30,200,200), new Scalar(15,50,50));
        genericDetector.debugContours = false;
        genericDetector.minArea = 700;
        genericDetector.perfectRatio = 1.8;
        genericDetector.stretch = true;
        genericDetector.stretchKernal = new Size(2,50);

        genericDetector.enable();

        telemetry.addData("Status", "Initialized " + genericDetector.detectionMode);

    }

    @Override
    public void init_loop() {
        telemetry.addData("Status", "Initialized.");
    }

    @Override
    public void start() {
        runtime.reset();
    }

    @Override
    public void loop() {
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        if(genericDetector.getFound() == true) {

            locationOfBlob = genericDetector.getLocation();

            xOfBlob = locationOfBlob.x;

            telemetry.addData("Location", locationOfBlob);
            telemetry.addData("Rect", genericDetector.getRect().toString());


            if ((xOfBlob >= -5) || xOfBlob <= 5) {

                 robot.drive.setThrottle(0);

            } else {

                robot.drive.setThrottle(0.2);

            }
        }
    }

    @Override
    public void stop() {
        genericDetector.disable();
    }


}
