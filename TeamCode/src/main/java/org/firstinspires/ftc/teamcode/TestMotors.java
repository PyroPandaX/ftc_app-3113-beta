package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Test Motors", group="Test")
//@Disabled
public class TestMotors extends OpMode {
    final static double MOTOR_POWER = 0.5;
    DcMotor driveRB, driveRF, driveLB, driveLF;
    //ColorSensor colorSensor;
    //float hsvValues[] = {0F,0F,0F};
    //final float values[] = hsvValues;
    //final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(com.qualcomm.ftcrobotcontroller.R.id.RelativeLayout);
    //boolean bPrevState;
    //boolean bCurrState;
    //boolean bLedOn;


    public TestMotors() {}

    public void init() {
        //bPrevState = false;
        //bCurrState = true;
        //bLedOn = true;
        driveRF = hardwareMap.dcMotor.get("driveRF");
        driveRB = hardwareMap.dcMotor.get("driveRB");
        driveLB = hardwareMap.dcMotor.get("driveLB");
        driveLF = hardwareMap.dcMotor.get("driveLF");
        driveLB.setDirection(DcMotor.Direction.REVERSE);
        driveLF.setDirection(DcMotor.Direction.REVERSE);
        //colorSensor = hardwareMap.colorSensor.get("line");

    }

    @Override
    public void start() {}

    @Override
    public void loop() {
        /*
        Color.RGBToHSV(colorSensor.red() * 8, colorSensor.green() * 8, colorSensor.blue() * 8, hsvValues);
        if ((bCurrState == true) && (bCurrState != bPrevState))  {
            bLedOn = !bLedOn;
            colorSensor.enableLed(bLedOn);
        }
        */
        if (this.time <= 3) {
            driveRB.setPower(MOTOR_POWER);
            driveRF.setPower(MOTOR_POWER);
            driveLB.setPower(MOTOR_POWER);
            driveLF.setPower(MOTOR_POWER);
        }
        else if (this.time <= 6) {
            driveRB.setPower(MOTOR_POWER);
            driveRF.setPower(MOTOR_POWER);
            driveLB.setPower(-MOTOR_POWER);
            driveLF.setPower(-MOTOR_POWER);
        }
        else if (this.time <= 9) {
            driveRB.setPower(-MOTOR_POWER);
            driveRF.setPower(-MOTOR_POWER);
            driveLB.setPower(MOTOR_POWER);
            driveLF.setPower(MOTOR_POWER);
        }
        /*
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
            }
        });
        */
        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("time", "elapsed time: " + Double.toString(this.time));
    }

    @Override
    public void stop() {}
}

