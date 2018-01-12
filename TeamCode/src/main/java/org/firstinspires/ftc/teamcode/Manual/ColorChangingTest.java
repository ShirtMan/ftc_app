package org.firstinspires.ftc.teamcode.Manual;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by mcshirt on 1/11/18.
 */
@TeleOp (name = "Color Change Test", group = "Test")
public class ColorChangingTest extends OpMode {

    int screenColor;

    @Override
    public void init(){



    }

    @Override
    public void loop(){

        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);


        if (gamepad1.dpad_left){
            screenColor = Color.BLUE;
        } else {
            screenColor = Color.GREEN;
        }
        relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(screenColor);
                }
        });


    }

    @Override
    public void stop(){


    }

}
