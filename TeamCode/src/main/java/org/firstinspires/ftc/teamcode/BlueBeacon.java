package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import ftc.vision.BeaconColorResult;
import ftc.vision.FrameGrabber;
import ftc.vision.ImageProcessorResult;

/**
 * Created by Mac on 12/19/2016.
 */
@Autonomous(name="BlueBeacon", group="NullBot")
//@Disabled
public class BlueBeacon extends OpMode{
    FrameGrabber frameGrabber = FtcRobotControllerActivity.frameGrabber; //Get the frameGrabber
    DcMotor motorRB, motorRF, motorLB, motorLF, spin, shoot;
    double timeAuto = 0, timeStart = 0, timeLine = 0;
    Servo hold, push;
    ColorSensor colorSensor;
    boolean bPrevState = false, bCurrState = false, bLedOn = true, sawLine = false;
    float hsvValues[] = {0F,0F,0F};
    final float values[] = hsvValues;
    final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(com.qualcomm.ftcrobotcontroller.R.id.RelativeLayout);
    BeaconColorResult result;

    public BlueBeacon()  {}

    public void init() {
        motorRB = hardwareMap.dcMotor.get("motor_1");
        motorRF = hardwareMap.dcMotor.get("motor_2");
        motorLB = hardwareMap.dcMotor.get("motor_3");
        motorLF = hardwareMap.dcMotor.get("motor_4");
        motorRB.setDirection(DcMotor.Direction.REVERSE);
        motorRF.setDirection(DcMotor.Direction.REVERSE);
        hold = hardwareMap.servo.get("hold");
        push = hardwareMap.servo.get("push");
        spin = hardwareMap.dcMotor.get("spin");
        shoot = hardwareMap.dcMotor.get("shoot");
        shoot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        colorSensor = hardwareMap.colorSensor.get("line");
        colorSensor.enableLed(bLedOn);
    }

    @Override
    public void start() {
        // defines timeStart as the timer at the start of autonomous to preserve an initial value
        timeAuto = 0;
        timeLine = 0;
        timeStart = this.time;
    }

    @Override
    public void loop() {
        timeAuto = this.time - timeStart;

        // check for button state transitions.
        if ((bCurrState == true) && (bCurrState != bPrevState))  {
            bLedOn = !bLedOn;
            colorSensor.enableLed(bLedOn);
        }
        // update previous state variable.
        bPrevState = bCurrState;
        // convert the RGB values to HSV values.
        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);

        if(white() && !sawLine) {
            sawLine = true;
        } else  {
            motorLB.setPower(.5);
            motorRB.setPower(.5);
            motorLF.setPower(.5);
            motorRF.setPower(.5);
        }

        if(sawLine) {
            timeLine = this.time - timeAuto;
            if (timeLine < .5) {
                motorLB.setPower(-.5);
                motorRB.setPower(-.5);
                motorLF.setPower(-.5);
                motorRF.setPower(-.5);
            } else if (timeLine < 1) {
                motorLB.setPower(0);
                motorRB.setPower(.5);
                motorLF.setPower(.5);
                motorRF.setPower(0);
            }
            frameGrabber.grabSingleFrame();
            while (!frameGrabber.isResultReady()) {
                sleepCool(5); //sleep for 5 milliseconds
            }
            ImageProcessorResult imageProcessorResult = frameGrabber.getResult();
            result = (BeaconColorResult) imageProcessorResult.getResult();
            BeaconColorResult.BeaconColor leftColor = result.getLeftColor();
            BeaconColorResult.BeaconColor rightColor = result.getRightColor();
            if(leftColor.toString().equals("BLUE"))  {
                push.setPosition(1);
            } else if(rightColor.toString().equals("BLUE"))  {
                push.setPosition(0);
            }
            if (timeLine < 5) {
                motorLB.setPower(0);
                motorRB.setPower(.5);
                motorLF.setPower(.5);
                motorRF.setPower(0);
            }
        }

        telemetry.addData("Result", result);
        telemetry.addData("White", white());
        telemetry.addData("Hue", hsvValues[0]);
        telemetry.addData("Saturation", hsvValues[1]);
        telemetry.addData("Value", hsvValues[2]);
        //relativeLayout.post(new Runnable() {
        // public void run() {relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));}
        //});
        telemetry.update();
        //wait before quitting (quitting clears telemetry)
        sleepCool(1000);
    }

    //delay method below
    public static void sleepCool(long sleepTime)    {
        long wakeupTime = System.currentTimeMillis() + sleepTime;
        while (sleepTime > 0) {
            try {
                Thread.sleep(sleepTime);
            }
            catch (InterruptedException e) {}
            sleepTime = wakeupTime - System.currentTimeMillis();
        }
    } //sleep

    public boolean white()   {
        if(hsvValues[0] < 5 && hsvValues[1] < .05 && hsvValues[2] > .9) {
            return true;
        }
        return false;
    }

    @Override
    public void stop() {}
}