package org.firstinspires.ftc.teamcode;
import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Ball", group="NullBot Shoot")
//@Disabled
public class Ball extends OpMode {
    int xVal, yVal, zVal, heading, angleZ, resetState = 0, count = 0;
    DcMotor motorRB, motorRF, motorLB, motorLF, spin, shoot;
    double timeAuto = 0;
    Servo hold;
    ModernRoboticsI2cGyro gyro;
    ElapsedTime elapsed = new ElapsedTime();

    public Ball() {}

    public void init() {
        motorRB = hardwareMap.dcMotor.get("motor_1");
        motorRF = hardwareMap.dcMotor.get("motor_2");
        motorLB = hardwareMap.dcMotor.get("motor_3");
        motorLF = hardwareMap.dcMotor.get("motor_4");
        motorRB.setDirection(DcMotor.Direction.REVERSE);
        motorRF.setDirection(DcMotor.Direction.REVERSE);
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
        timeAuto = elapsed.time();
        heading = gyro.getHeading();
        angleZ = gyro.getIntegratedZValue();
        // get the x, y, and z values (rate of change of angle).
        xVal = gyro.rawX();
        yVal = gyro.rawY();
        zVal = gyro.rawZ();

        if (timeAuto < .8) {
            motorLB.setPower(.5);
            motorRB.setPower(.5);
            motorLF.setPower(.5);
            motorRF.setPower(.5);
            hold.setPosition(1);
        } else if (timeAuto < 3.5 && timeAuto > .8) {
            motorLB.setPower(0);
            motorRB.setPower(0);
            motorLF.setPower(0);
            motorRF.setPower(0);
            hold.setPosition(.5);
            shoot.setPower(.5);
        } else if (timeAuto > 3.5 && timeAuto < 9) {
            spin.setPower(.6);
        } else if (timeAuto < 11 && timeAuto > 9) {
            motorLB.setPower(.5);
            motorRB.setPower(.5);
            motorLF.setPower(.5);
            motorRF.setPower(.5);
            shoot.setPower(0);
            spin.setPower(0);
        } else if (timeAuto > 11) {
            motorLB.setPower(0);
            motorRB.setPower(0);
            motorLF.setPower(0);
            motorRF.setPower(0);
        }

        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("Time", "elapsed time: " + Double.toString(elapsed.time()));
        telemetry.addData("0", "Heading %03d", heading);
        telemetry.addData("1", "Int. Ang. %03d", angleZ);
        telemetry.addData("2", "X av. %03d", xVal);
        telemetry.addData("3", "Y av. %03d", yVal);
        telemetry.addData("4", "Z av. %03d", zVal);
        telemetry.addData("5", "resetState %03d", resetState);
        telemetry.update();
    }

    @Override
    public void stop() {
    }
}