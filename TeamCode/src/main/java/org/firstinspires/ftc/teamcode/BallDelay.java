package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Ball with Delay", group="NullBot Shoot")
@Disabled
public class BallDelay extends OpMode {
    int xVal, yVal, zVal, heading, angleZ, resetState = 0, count = 0;
    DcMotor driveRB, driveRF, driveLB, driveLF, spin, shoot;
    double timeAuto = 0;
    Servo hold;
    ModernRoboticsI2cGyro gyro;
    ElapsedTime elapsed = new ElapsedTime();

    public BallDelay() {}

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
    }

    @Override
    public void start() {
        elapsed.reset();
    }

    @Override
    public void loop() {
        sleepCool(10000);
        // time since autonomous began
        timeAuto = elapsed.time() - 10;
        heading = gyro.getHeading();
        angleZ = gyro.getIntegratedZValue();
        // get the x, y, and z values (rate of change of angle).
        xVal = gyro.rawX();
        yVal = gyro.rawY();
        zVal = gyro.rawZ();

        if (timeAuto < .8 && timeAuto > 0) {
            driveLB.setPower(.5);
            driveRB.setPower(.5);
            driveLF.setPower(.5);
            driveRF.setPower(.5);
            hold.setPosition(1);
        } else if (timeAuto < 3.5 && timeAuto > .8) {
            driveLB.setPower(0);
            driveRB.setPower(0);
            driveLF.setPower(0);
            driveRF.setPower(0);
            hold.setPosition(.5);
            shoot.setPower(.35);
        } else if (timeAuto > 3.5 && timeAuto < 9) {
            spin.setPower(.6);
        } else if (timeAuto < 11 && timeAuto > 9) {
            driveLB.setPower(.5);
            driveRB.setPower(.5);
            driveLF.setPower(.5);
            driveRF.setPower(.5);
            shoot.setPower(0);
            spin.setPower(0);
        } else if (timeAuto > 11) {
            driveLB.setPower(0);
            driveRB.setPower(0);
            driveLF.setPower(0);
            driveRF.setPower(0);
        }

        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("Time", "elapsed time: " + Double.toString(elapsed.time()));
        telemetry.addData("0", "Heading %03d", heading);
        telemetry.addData("1", "Int. Ang. %03d", angleZ);
        telemetry.addData("2", "X av. %03d", xVal);
        telemetry.addData("3", "Y av. %03d", yVal);
        telemetry.addData("4", "Z av. %03d", zVal);
        telemetry.addData("5", "resetState %03d", resetState);
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
    }

    @Override
    public void stop() {
    }
}