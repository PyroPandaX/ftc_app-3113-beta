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
import com.qualcomm.robotcore.util.ElapsedTime;

import ftc.vision.BeaconColorResult;

/**
 * Created by Mac on 12/19/2016.
 */
@Autonomous(name="Test Strafe", group="Test")
//@Disabled
public class TestStrafe extends OpMode  {
    DcMotor driveRB, driveRF, driveLB, driveLF, spin, shoot;
    double timeAuto;
    Servo hold;
    byte[] colorCcache;
    I2cDevice colorC;
    I2cDeviceSynch colorCreader;
    BeaconColorResult result;
    ModernRoboticsI2cGyro gyro;
    int xVal, yVal, zVal, heading, angleZ, resetState;
    ElapsedTime elapsed = new ElapsedTime();

    public TestStrafe()  {}

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
                break;
            case 1:
                telemetry.addData(">", "Gyro Calibrated.  Press Start.");
                break;
        }
        hold.setPosition(1);
    }

    @Override
    public void start() {
        elapsed.reset();
    }

    @Override
    public void loop() {
        heading = gyro.getHeading();
        angleZ = gyro.getIntegratedZValue();
        // get the x, y, and z values (rate of change of angle).
        xVal = gyro.rawX();
        yVal = gyro.rawY();
        zVal = gyro.rawZ();

        timeAuto = elapsed.time();

        if (timeAuto < 10)    {
            driveLB.setPower(.7);
            driveRB.setPower(0);
            driveLF.setPower(0);
            driveRF.setPower(.7);
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

