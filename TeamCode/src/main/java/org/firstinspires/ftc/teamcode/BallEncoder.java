package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="Ball with Encoder", group="NullBot Shoot")
//@Disabled
public class BallEncoder extends OpMode {
    private int xVal, yVal, zVal;     // Gyro rate Values
    private int heading;              // Gyro integrated heading
    private int angleZ;
    boolean lastResetState = false;
    boolean curResetState = false;
    public int resetState = 0, v_state = 0;

    public BallEncoder() {}

    ModernRoboticsI2cGyro gyro;
    DcMotor driveRB, driveRF, driveLB, driveLF, spin, shoot;
    public double timeAuto = 0;
    public double timeStart = 0;
    public double time0, time1, time2, time3, time4, pos0, pos1, pos2, pos3, pos4 = 0;
    public int count = 0;
    Servo hold, push;

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
        // defines timeStart as the timer at the start of autonomous to preserve an initial value
        timeAuto = 0;
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

        if (timeAuto < .8) {
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
            shoot.setPower(.1);
            shoot.setMaxSpeed(150);
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
        telemetry.addData("time", "elapsed time: " + Double.toString(timeAuto));
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