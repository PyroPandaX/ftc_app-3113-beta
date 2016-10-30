package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="DemoTest", group="Demo Bot")
//@Disabled
public class DemoTest extends OpMode {
    final static double MOTOR_POWER = 0.2;
    DcMotor motorRB, motorRF, motorLB, motorLF;
    //ColorSensor colorSensor;
    //float hsvValues[] = {0F,0F,0F};
    //final float values[] = hsvValues;
    //final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(com.qualcomm.ftcrobotcontroller.R.id.RelativeLayout);
    //boolean bPrevState;
    //boolean bCurrState;
    //boolean bLedOn;

    public DemoTest() {}

    public void init() {
        //bPrevState = false;
        //bCurrState = true;
        //bLedOn = true;
        motorRB = hardwareMap.dcMotor.get("motor_1");
        motorRF = hardwareMap.dcMotor.get("motor_2");
        motorLB = hardwareMap.dcMotor.get("motor_3");
        motorLF = hardwareMap.dcMotor.get("motor_4");
        motorLB.setDirection(DcMotor.Direction.REVERSE);
        motorLF.setDirection(DcMotor.Direction.REVERSE);
        //colorSensor = hardwareMap.colorSensor.get("line");
    }

    @Override
    public void start() {}

    @Override
    public void loop() {
        /*
        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
        if ((bCurrState == true) && (bCurrState != bPrevState))  {
            bLedOn = !bLedOn;
            colorSensor.enableLed(bLedOn);
        }
        */
        if (this.time <= 3) {
            motorRB.setPower(MOTOR_POWER);
            motorRF.setPower(MOTOR_POWER);
            motorLB.setPower(MOTOR_POWER);
            motorLF.setPower(MOTOR_POWER);
        }
        else if (this.time <= 6) {
            motorRB.setPower(MOTOR_POWER);
            motorRF.setPower(MOTOR_POWER);
            motorLB.setPower(-MOTOR_POWER);
            motorLF.setPower(-MOTOR_POWER);
        }
        else if (this.time <= 9) {
            motorRB.setPower(-MOTOR_POWER);
            motorRF.setPower(-MOTOR_POWER);
            motorLB.setPower(MOTOR_POWER);
            motorLF.setPower(MOTOR_POWER);
        }
        /*
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
            }
        });
        */
        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("time", "elapsed time: " + Double.toString(this.time));
    }

    @Override
    public void stop() {}
}

