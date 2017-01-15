package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import ftc.vision.BeaconColorResult;
import ftc.vision.FrameGrabber;
import ftc.vision.ImageProcessorResult;

/**
 * Created by Mac on 12/19/2016.
 */
@Autonomous(name="Colors", group="NullBot")
public class DetectColors extends OpMode{
        FrameGrabber frameGrabber = FtcRobotControllerActivity.frameGrabber; //Get the frameGrabber

    public void init() {}

    @Override
    public void start() {}

    @Override
    public void loop() {
        frameGrabber.grabSingleFrame(); //Tell it to grab a frame
        while (!frameGrabber.isResultReady()) { //Wait for the result
            sleepCool(5); //sleep for 5 milliseconds
        }

        //Get the result
        ImageProcessorResult imageProcessorResult = frameGrabber.getResult();
        BeaconColorResult result = (BeaconColorResult) imageProcessorResult.getResult();

        BeaconColorResult.BeaconColor leftColor = result.getLeftColor();
        BeaconColorResult.BeaconColor rightColor = result.getRightColor();

        telemetry.addData("Result", result);
        telemetry.update();
        //wait before quitting (quitting clears telemetry)
        sleepCool(1);
    }

    //delay method below
    public static void sleepCool(long sleepTime)
    {
        long wakeupTime = System.currentTimeMillis() + sleepTime;
        while (sleepTime > 0)
        {
            try
            {
                Thread.sleep(sleepTime);
            }
            catch (InterruptedException e)
            {
            }
            sleepTime = wakeupTime - System.currentTimeMillis();
        }
    } //sleep

    @Override
    public void stop() {}
}
