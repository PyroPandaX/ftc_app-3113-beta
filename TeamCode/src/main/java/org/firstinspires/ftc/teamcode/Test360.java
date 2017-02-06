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

/**
 * Created by Mac on 12/19/2016.
 */
@Autonomous(name="Test 180", group="Test")
//@Disabled
public class Test360 extends OpMode{
    FrameGrabber frameGrabber = FtcRobotControllerActivity.frameGrabber; //Get the frameGrabber
    DcMotor driveRB, driveRF, driveLB, driveLF, spin, shoot;
    double timeAuto = 0, timeStart = 0, timeLine = 0;
    ArrayList<Double> timeStep = new ArrayList<Double>();
    Servo hold;
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

    public Test360()  {}

    public void init() {
        driveRF = hardwareMap.dcMotor.get("driveRF");
        driveRB = hardwareMap.dcMotor.get("driveRB");
        driveLB = hardwareMap.dcMotor.get("driveLB");
        driveLF = hardwareMap.dcMotor.get("driveLF");
        driveRB.setDirection(DcMotor.Direction.REVERSE);
        driveRF.setDirection(DcMotor.Direction.REVERSE);
        hold = hardwareMap.servo.get("hold");
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

//        if (timeAuto < 5) {
//            driveLB.setPower(0);
//            driveRB.setPower(0);
//            driveLF.setPower(0);
//            driveRF.setPower(0);
//        }
//        if (timeAuto > 5){
//            zero();
//        }
        turnToAngle(180);

        telemetry.addData("1", "Heading %03d", heading);
        telemetry.addData("1", "Int. Ang. %03d", angleZ);
        telemetry.update();
        //wait before quitting (quitting clears telemetry)
        //sleepCool(1);
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

    boolean turnToAngle(double angle) {
//        if(angleZ > angle + 2) {
//            turn(.3, "RIGHT");
//        } else if(angleZ < angle - 2)  {
//            turn(.3, "LEFT");
//        }
        if(angleZ > angle + 10) {
            turn(.4, "RIGHT");
        } else if(angleZ < angle - 10)  {
            turn(.4, "LEFT");
        } else if(angleZ > angle + 5) {
            turn(.3, "RIGHT");
        } else if(angleZ < angle - 5)  {
            turn(.3, "LEFT");
        } else if(angleZ > angle + 2) {
            turn(.2, "RIGHT");
        } else if(angleZ < angle - 2)  {
            turn(.2, "LEFT");
        }

        if(angleZ < angle + 4 && angleZ > angle - 4)   {
            resetDrive();
            return true;
        } else  {
            return false;
        }
    }

    void resetDrive() {
        driveLB.setPower(0);
        driveRB.setPower(0);
        driveLF.setPower(0);
        driveRF.setPower(0);
    }

    void turn(double power, String direction) {
        if(direction.equals("LEFT")) {
            driveLB.setPower(-power);
            driveRB.setPower(power);
            driveLF.setPower(-power);
            driveRF.setPower(power);
        } else if(direction.equals("RIGHT"))    {
            driveLB.setPower(power);
            driveRB.setPower(-power);
            driveLF.setPower(power);
            driveRF.setPower(-power);
        }
    }


    @Override
    public void stop() {}
}

