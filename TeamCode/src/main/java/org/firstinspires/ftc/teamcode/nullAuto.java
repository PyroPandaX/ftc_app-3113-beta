package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "NULLAuto", group = "Autonomous")
public class nullAuto extends LinearOpMode {
    ModernRoboticsI2cGyro gyro;
    ColorSensor colorSensor;

    @Override
    public void runOpMode() {
        DcMotor motorRB, motorRF, motorLB, motorLF, spin, shoot;

        int xVal, yVal, zVal;
        int heading;
        int angleZ;
        boolean lastResetState = false;
        boolean curResetState  = false;
        int resetState = 0;
        float hsvValues[] = {0F,0F,0F};
        final float values[] = hsvValues;
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(com.qualcomm.ftcrobotcontroller.R.id.RelativeLayout);
        boolean bLedOn = true;

        colorSensor = hardwareMap.colorSensor.get("line");
        colorSensor.enableLed(bLedOn);
        gyro = (ModernRoboticsI2cGyro)hardwareMap.gyroSensor.get("gyro");

        telemetry.addData(">", "Gyro Calibrating. Do Not move!");
        telemetry.update();
        gyro.calibrate();

        while (!isStopRequested() && gyro.isCalibrating())  {
            sleep(50);
            idle();
        }

        telemetry.addData(">", "Gyro Calibrated.  Press Start.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive())  {
            while(opModeIsActive()&& resetState ==0)
            {
                gyro.resetZAxisIntegrator();
                resetState++;
            }

            xVal = gyro.rawX();
            yVal = gyro.rawY();
            zVal = gyro.rawZ();

            heading = gyro.getHeading();
            angleZ  = gyro.getIntegratedZValue();

            telemetry.addData(">", "Press A & B to reset Heading.");
            telemetry.addData("0", "Heading %03d", heading);
            telemetry.addData("1", "Int. Ang. %03d", angleZ);
            telemetry.addData("2", "X av. %03d", xVal);
            telemetry.addData("3", "Y av. %03d", yVal);
            telemetry.addData("4", "Z av. %03d", zVal);

            Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
            telemetry.addData("LED", bLedOn ? "On" : "Off");
            telemetry.addData("Clear", colorSensor.alpha());
            telemetry.addData("Red  ", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Blue ", colorSensor.blue());
            telemetry.addData("Hue", hsvValues[0]);

            relativeLayout.post(new Runnable() {
                public void run() {relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));}
            });

            telemetry.update();
            idle();
        }
    }
}