

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

@Autonomous(name="RedBallRamp", group="Demo Bot")
@Disabled
public class RedBallRamp extends OpMode {
    private int xVal, yVal, zVal;     // Gyro rate Values
    private int heading;              // Gyro integrated heading
    private int angleZ;
    boolean lastResetState = false;
    boolean curResetState  = false;
    public  int resetState = 0, v_state = 0;
    public RedBallRamp() {}
    ModernRoboticsI2cGyro gyro;
    DcMotor motorRB, motorRF, motorLB, motorLF, spin, shoot;
    public double timeAuto = 0;
    public double timeStart = 0;
    public double time0, time1,time2,time3,time4, pos0, pos1,pos2,pos3,pos4 = 0;
    public int count = 0;
    Servo hold;
    public static double powerShoot = 0;

    public void init() {
        //bPrevState = false;
        //bCurrState = true;
        //bLedOn = true;
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
        gyro= (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        //colorSensor = hardwareMap.colorSensor.get("line");
        //  while (true) {
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
        // }
    }

    @Override
    public void start() {
        timeAuto = 0;
        // defines timeStart as the timer at the start of autonomous to preserve an initial value
        timeStart = this.time;
    }

    @Override
    public void loop() {
        // time since autonomous began
        timeAuto = this.time - timeStart;
        heading = gyro.getHeading();
        angleZ = gyro.getIntegratedZValue();
        // get the x, y, and z values (rate of change of angle).
        xVal = gyro.rawX();
        yVal = gyro.rawY();
        zVal = gyro.rawZ();

        if (timeAuto < 1) {
            motorLB.setPower(.5);
            motorRB.setPower(.5);
            motorLF.setPower(.5);
            motorRF.setPower(.5);
            hold.setPosition(1);
            powerShoot = 0;

        }
        else if (timeAuto <3.5 && timeAuto >1)
        {
            motorLB.setPower(0);
            motorRB.setPower(0);
            motorLF.setPower(0);
            motorRF.setPower(0);
            hold.setPosition(.5);
            shoot.setPower(.4-powerShoot);
        }
        else  if (timeAuto> 3.5 && timeAuto<7) {
            spin.setPower(.500);
        }
        else if(timeAuto<9 && timeAuto >7){
            motorLB.setPower(.5);
            motorRB.setPower(.5);
            motorLF.setPower(.5);
            motorRF.setPower(.5);
            powerShoot = .400;
            spin.setPower(0);

        }
        else if(timeAuto >9 && timeAuto <16) {
            motorLB.setPower(-1);
            motorRB.setPower(-1);
            motorLF.setPower(1);
            motorRF.setPower(1);
            shoot.setPower(0);
        }
        else if (timeAuto >16){
            motorLB.setPower(0);
            motorRB.setPower(0);
            motorLF.setPower(0);
            motorRF.setPower(0);
            shoot.setPower(0);
        }

        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("time", "elapsed time: " + Double.toString(timeAuto));
        telemetry.addData("0", "Heading %03d", heading);
        telemetry.addData("1", "Int. Ang. %03d", angleZ);
        telemetry.addData("2", "X av. %03d", xVal);
        telemetry.addData("3", "Y av. %03d", yVal);
        telemetry.addData("4", "Z av. %03d", zVal);
        telemetry.addData("5", "resetState %03d", resetState);
    }

    @Override
    public void stop() {}
}