package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import ftc.vision.BeaconColorResult;
import ftc.vision.FrameGrabber;
import ftc.vision.ImageProcessorResult;

/**
 * Created by Mac on 12/19/2016.
 */
@Autonomous(name="Beacon", group="NullBot")
@Disabled
public class RedBeacon extends OpMode{
    FrameGrabber frameGrabber = FtcRobotControllerActivity.frameGrabber; //Get the frameGrabber
    DcMotor motorRB, motorRF, motorLB, motorLF, spin, shoot;
    double timeAuto = 0, timeStart = 0, timeLine = 0;
    Servo hold, push;
    byte[] colorCcache;
    I2cDevice colorC;
    I2cDeviceSynch colorCreader;
    BeaconColorResult result;
    boolean sawLine = false;
    ModernRoboticsI2cGyro gyro;
    private int xVal, yVal, zVal;     // Gyro rate Values
    private int heading;              // Gyro integrated heading
    private int angleZ;
    public int resetState;
    public RedBeacon()  {}

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
        colorC = hardwareMap.i2cDevice.get("line");
        colorCreader = new I2cDeviceSynchImpl(colorC, I2cAddr.create8bit(0x3c), false);
        colorCreader.engage();
        colorCreader.write8(3, 0);
        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        switch (resetState) {
            case 0:
                telemetry.addData(">", "Gyro Calibrating. Do Not move!" + resetState);
                gyro.calibrate();
                if (!gyro.isCalibrating()) {
                    resetState++;
                }
            case 1:
                telemetry.addData(">", "Gyro Calibrated.  Press Start.");
        }
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
        heading = gyro.getHeading();
        angleZ = gyro.getIntegratedZValue();
        // get the x, y, and z values (rate of change of angle).
        xVal = gyro.rawX();
        yVal = gyro.rawY();
        zVal = gyro.rawZ();

        timeAuto = this.time - timeStart;

        colorCcache = colorCreader.read(0x04, 1);

        if(colorCcache[0] > 6) {
            sawLine = true;
            motorLB.setPower(0);
            motorRB.setPower(0);
            motorLF.setPower(0);
            motorRF.setPower(0);
        } else  {
            motorLB.setPower(.2);
            motorRB.setPower(.2);
            motorLF.setPower(.2);
            motorRF.setPower(.2);
            sawLine = false;
        }

        if(sawLine) {
            timeLine = this.time - timeAuto;
            /*
            if (timeLine < .5) {
                motorLB.setPower(-.2);
                motorRB.setPower(-.2);
                motorLF.setPower(-.2);
                motorRF.setPower(-.2);
            } else if (timeLine < 1) {
                motorLB.setPower(0);
                motorRB.setPower(.4);
                motorLF.setPower(.4);
                motorRF.setPower(0);
            }
            */
            frameGrabber.grabSingleFrame();
            while (!frameGrabber.isResultReady()) {
                sleepCool(5); //sleep for 5 milliseconds
            }
            ImageProcessorResult imageProcessorResult = frameGrabber.getResult();
            result = (BeaconColorResult) imageProcessorResult.getResult();
            BeaconColorResult.BeaconColor leftColor = result.getLeftColor();
            BeaconColorResult.BeaconColor rightColor = result.getRightColor();
            if(leftColor.toString().equals("RED"))  {
                //push.setPosition(1);
                spin.setPower(.5);
            } else if(leftColor.toString().equals("BLUE"))  {
                //push.setPosition(0);
                shoot.setPower(.35);
            }
            /*
            if (timeLine < 5) {
                motorLB.setPower(0);
                motorRB.setPower(.5);
                motorLF.setPower(.5);
                motorRF.setPower(0);
            }
            */
        }

        telemetry.addData("Result", result);

        telemetry.addData("1", "Int. Ang. %03d", angleZ);

        telemetry.update();
        //wait before quitting (quitting clears telemetry)
        sleepCool(1);
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

    @Override
    public void stop() {}
}

