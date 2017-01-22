package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

import java.util.ArrayList;

import ftc.vision.BeaconColorResult;
import ftc.vision.FrameGrabber;
import ftc.vision.ImageProcessorResult;

/**
 * Created by Mac on 12/19/2016.
 */
@Autonomous(name="StrafeRed", group="NullBot")
//@Disabled
public class StrafeBeacon extends OpMode{
    FrameGrabber frameGrabber = FtcRobotControllerActivity.frameGrabber; //Get the frameGrabber
    DcMotor motorRB, motorRF, motorLB, motorLF, spin, shoot;
    double timeAuto = 0, timeStart = 0, timeLine = 0, timeColor = 0;
    //ArrayList<Double> timeStep = new ArrayList<Double>();
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
    int count = 0;

    public StrafeBeacon()  {}

    public void init() {
        motorRF = hardwareMap.dcMotor.get("motor_1");
        motorRB = hardwareMap.dcMotor.get("motor_2");
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
        hold.setPosition(1);
    }

    @Override
    public void start() {
        // defines timeStart as the timer at the start of autonomous to preserve an initial value
        timeAuto = 0;
        timeLine = 0;
        timeColor = 0;
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
        //idealAngle();

        timeAuto = this.time - timeStart;

        colorCcache = colorCreader.read(0x04, 1);

        if (timeAuto < 1.5) {
            motorLB.setPower(0);
            motorRB.setPower(0);
            motorLF.setPower(0);
            motorRF.setPower(0);
            hold.setPosition(.5);
            shoot.setPower(.35);
        } else if (timeAuto > 1.5 && timeAuto < 5.5) {
            spin.setPower(.6);
        } else if (timeAuto > 5.5 && timeAuto < 10.5)    {
            hold.setPosition(1);
            shoot.setPower(0);
            spin.setPower(0);
            motorLB.setPower(.7);
            motorRB.setPower(0);
            motorLF.setPower(0);
            motorRF.setPower(.7);
        }  else if (timeAuto > 10.5 && timeAuto < 11.5)    {
                motorLB.setPower(-.2);
                motorRB.setPower(-.2);
                motorLF.setPower(-.2);
                motorRF.setPower(-.2);
        } else if(timeAuto > 12 && !sawLine)   {
            if(colorCcache[0] > 6) {
                motorLB.setPower(0);
                motorRB.setPower(0);
                motorLF.setPower(0);
                motorRF.setPower(0);
                if(zero())
                    sawLine = true;
            } else  {
                motorLB.setPower(.2);
                motorRB.setPower(.2);
                motorLF.setPower(.2);
                motorRF.setPower(.2);
            }
        }

        if(sawLine) {
            timeLine = this.time - timeAuto;
            if (timeLine < .5) {
                motorLB.setPower(-.7);
                motorRB.setPower(.7);
                motorLF.setPower(.7);
                motorRF.setPower(-.7);
            } else if(timeLine > .5) {
                frameGrabber.grabSingleFrame();
                while (!frameGrabber.isResultReady()) {
                    sleepCool(5); //sleep for 5 milliseconds
                }
                ImageProcessorResult imageProcessorResult = frameGrabber.getResult();
                result = (BeaconColorResult) imageProcessorResult.getResult();
                BeaconColorResult.BeaconColor leftColor = result.getLeftColor();
                BeaconColorResult.BeaconColor rightColor = result.getRightColor();

                if (leftColor.toString().equals("RED")) {
                    timeColor = this.time - timeLine;
                    if (timeColor < 1.5) {
                        motorLB.setPower(.7);
                        motorRB.setPower(-.7);
                        motorLF.setPower(-.7);
                        motorRF.setPower(.7);
                    } else if (timeColor > 1.5 && timeColor < 3) {
                        motorLB.setPower(-.7);
                        motorRB.setPower(.7);
                        motorLF.setPower(.7);
                        motorRF.setPower(-.7);
                    }
                } else if (leftColor.toString().equals("BLUE")) {
                    timeColor = this.time - timeLine;
                    if (timeColor < .5) {
                        motorLB.setPower(.2);
                        motorRB.setPower(.2);
                        motorLF.setPower(.2);
                        motorRF.setPower(.2);
                    } else if (timeColor > .5 && timeColor < 2) {
                        motorLB.setPower(.7);
                        motorRB.setPower(-.7);
                        motorLF.setPower(-.7);
                        motorRF.setPower(.7);
                    } else if (timeColor > 2 && timeColor < 3.5) {
                        motorLB.setPower(-.7);
                        motorRB.setPower(.7);
                        motorLF.setPower(.7);
                        motorRF.setPower(-.7);
                    }
                }

                if (leftColor.toString().equals("RED") && rightColor.toString().equals("RED")) {
                    if (count == 0)
                        sawLine = false;
                    count++;
                }
            }
        }

        telemetry.addData("Result", result);
        telemetry.addData("1", "Int. Ang. %03d", angleZ);
        telemetry.addData("White", sawLine);
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

    public boolean zero() {
        if(heading > 2) {
            motorLB.setPower(.2);
            motorRB.setPower(-.2);
            motorLF.setPower(.2);
            motorRF.setPower(-.2);
        } else if(heading < -2)  {
            motorLB.setPower(-.2);
            motorRB.setPower(.2);
            motorLF.setPower(-.2);
            motorRF.setPower(.2);
        }

        if(heading > -2 && heading < 2)   {
            return true;
        } else  {
            return false;
        }
    }

    @Override
    public void stop() {}
}

