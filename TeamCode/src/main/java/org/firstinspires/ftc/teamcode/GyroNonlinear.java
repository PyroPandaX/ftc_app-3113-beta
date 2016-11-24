

package org.firstinspires.ftc.teamcode;
import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="GyroNonlinear", group="Demo Bot")
//@Disabled
public class GyroNonlinear extends OpMode {
    final static double MOTOR_POWER = 0.2;
   private int xVal, yVal, zVal;     // Gyro rate Values
   private int heading;              // Gyro integrated heading
   private int angleZ;
    boolean lastResetState = false;
    boolean curResetState  = false;
  public  int resetState = 0;
    public GyroNonlinear() {}
    ModernRoboticsI2cGyro gyro;
    DcMotor motorRB, motorRF, motorLB, motorLF, spin, shoot;


    public void init() {
        //bPrevState = false;
        //bCurrState = true;
        //bLedOn = true;
     /*   motorRB = hardwareMap.dcMotor.get("motor_1");
        motorRF = hardwareMap.dcMotor.get("motor_2");
        motorLB = hardwareMap.dcMotor.get("motor_3");
        motorLF = hardwareMap.dcMotor.get("motor_4");
        motorLB.setDirection(DcMotor.Direction.REVERSE);
        motorLF.setDirection(DcMotor.Direction.REVERSE);
       */ gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");
        //colorSensor = hardwareMap.colorSensor.get("line");


            switch (resetState) {
                case 0:
                    telemetry.addData(">", "Gyro Calibrating. Do Not move!");
                    gyro.calibrate();
                    if (!gyro.isCalibrating()) {
                        resetState++;
                    }
                case 1:
                    telemetry.addData(">", "Gyro Calibrated.  Press Start.");
            }





    }

    @Override
    public void start() {}

    @Override
    public void loop() {
        heading = gyro.getHeading();
        angleZ = gyro.getIntegratedZValue();
        // get the x, y, and z values (rate of change of angle).
        xVal = gyro.rawX();
        yVal = gyro.rawY();
        zVal = gyro.rawZ();
        if (this.time <= 6) {
            xVal =0;
            yVal =0;
            zVal = 0;
        }
        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("time", "elapsed time: " + Double.toString(this.time));
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

