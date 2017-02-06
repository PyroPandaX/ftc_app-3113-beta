package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Test Shooter with Encoder", group="Test")
//@Disabled
public class TestShooterEncoderAuto extends OpMode {
    DcMotor driveRB, driveRF, driveLB, driveLF, spin, shoot;
    Servo hold;
    ModernRoboticsI2cGyro gyro;
    int xVal, yVal, zVal, heading, angleZ, resetState;
    ElapsedTime elapsed = new ElapsedTime();

    public TestShooterEncoderAuto() {}

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
        shoot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        gyro.calibrate();
        switch (resetState) {
            case 0:
                telemetry.addData(">", "Gyro Calibrating. Do Not move! " + resetState);
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
        hold.setPosition(.5);
        shoot.setMaxSpeed(500);
        shoot.setPower(1);
        spin.setPower(.6);


        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("time", "elapsed time: " + elapsed.toString());
    }

    @Override
    public void stop() {
    }
}