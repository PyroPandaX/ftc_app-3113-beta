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
import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import java.util.ArrayList;
import ftc.vision.*;

/*
code description
 */
@Autonomous(name="Red Beacon Testing", group="NullBot Beacon")
//@Disabled
public class RedBeaconTesting extends OpMode{
    //hardware variables
    DcMotor driveRB, driveRF, driveLB, driveLF, spin, shoot; //add lift motors here
    Servo hold;
    //sensor variables
    FrameGrabber frameGrabber = FtcRobotControllerActivity.frameGrabber;
    byte[] colorCcache;
    I2cDevice colorC;
    I2cDeviceSynch colorCreader;
    BeaconColorResult result;
    ModernRoboticsI2cGyro gyro;
    int xVal, yVal, zVal, heading, angleZ, resetState;
    BeaconColorResult.BeaconColor leftColor;
    BeaconColorResult.BeaconColor rightColor;
    //time variables
    ElapsedTime elapsed = new ElapsedTime();
    ArrayList<Double> timeStep = new ArrayList<>();
    double displacement;
    //counters
    int pushed = 0, step = 0;
    //standard powers
    final double STRAFE_POWER = .7, DRIVE_POWER = .2, SHOOT_POWER = .6, CONVEYOR_POWER = .6;
    //standard servo positions
    final double UP_POSITION = .5, DOWN_POSITION = 1;

    public RedBeaconTesting()  {}

    public void init() {
        //hardware config
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
        //sensor config
        colorC = hardwareMap.i2cDevice.get("line");
        colorCreader = new I2cDeviceSynchImpl(colorC, I2cAddr.create8bit(0x3c), false);
        colorCreader.engage();
        colorCreader.write8(3, 0);
        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        //initialize position(s)
        hold.setPosition(DOWN_POSITION);
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
        elapsed.reset(); //time starts on game start instead of init
    }

    @Override
    public void loop() {
        //calculate time from start of most recent step
        timeStep.add(elapsed.time());
        displacement = elapsed.time() - timeStep.get(0);

        //get sensor values
        heading = gyro.getHeading();
        angleZ = gyro.getIntegratedZValue();
        xVal = gyro.rawX();
        yVal = gyro.rawY();
        zVal = gyro.rawZ();
        colorCcache = colorCreader.read(0x04, 1);

        if (step == 0 && shoot(2, SHOOT_POWER, CONVEYOR_POWER)) {
            step++;
            timeStep.clear();
        } else if (step == 1 && move("STRAFE", 5.25, STRAFE_POWER, "45", "FORWARD_LEFT")) {
            step++;
            timeStep.clear();
        } else if (step == 2 && turnToAngle(0)) {
            step++;
            timeStep.clear();
        } else if (step == 3 && move("STRAIGHT", 1.75, -DRIVE_POWER, "", "")) {
            step++;
            timeStep.clear();
        } else if (step == 4) {
            if (pushed == 0) {
                findLine();
            } else if (pushed == 1) {
//                if (displacement < 2) {
//                    straight(DRIVE_POWER);
//                } else if(displacement > 2 && !turnToAngle(0)) {
//                } else  {
//                    findLine();
//                }
                move("STRAIGHT", DRIVE_POWER, 2, "", "");
                if(displacement > 2 && !turnToAngle(0)){}
                else
                    findLine();
            } else if (pushed > 1) {
                resetDrive();
                step = 7;
                timeStep.clear();
            }
        } else if(step == 5) {
            captureFrame();
            if(!getLeftColor().equals("UNKNOWN")) {
                step++;
                timeStep.clear();
            }
        }   else if(step == 6) {
            beacon();
        } else if(step == 7)    {
            resetRobot();
        }

        telemetry.addData("Result", result);
        telemetry.addData("Angle", angleZ);
        telemetry.addData("White", white());
        telemetry.addData("Pushed", pushed);
        telemetry.update();
        //wait before quitting (quitting clears telemetry)
        //sleepCool(1);
    }

    /*
    delay method below
    measured in milliseconds
     */
    static void sleepCool(long sleepTime)    {
        long wakeupTime = System.currentTimeMillis() + sleepTime;
        while (sleepTime > 0) {
            try {
                Thread.sleep(sleepTime);
            } catch (InterruptedException e) {}
            sleepTime = wakeupTime - System.currentTimeMillis();
        }
    }

    void captureFrame() {
        frameGrabber.grabSingleFrame();
        while (!frameGrabber.isResultReady()) {
            sleepCool(5);
        }
        ImageProcessorResult imageProcessorResult = frameGrabber.getResult();
        result = (BeaconColorResult) imageProcessorResult.getResult();
        leftColor = result.getLeftColor();
        rightColor = result.getRightColor();
    }

    String getLeftColor()    {
        return leftColor.toString();
    }

    String getRightColor()    {
        return rightColor.toString();
    }

    void findLine() {
        if (white()) {
            resetDrive();
            timeStep.clear();
            step++;
        } else {
            straight(DRIVE_POWER);
        }
    }

    boolean white() {
        if(colorCcache[0] > 6)
            return true;
        return false;
    }

    void beacon()   {
//        if (leftColor.toString().equals("RED")) {
//            if (displacement > 0 && displacement < 1.5) {
//                strafe(STRAFE_POWER, "90", "LEFT");
//            } else if (displacement > 1.5 && displacement < 2.5) {
//                strafe(STRAFE_POWER, "90", "RIGHT");
//            } else if(displacement > 2.5 && !turnToAngle(0)) {
//            } else {
//                resetDrive();
//                timeStep.clear();
//                pushed++;
//                step = 4;
//            }
//        } else if (leftColor.toString().equals("BLUE")) {
//            if (displacement < .5) {
//                straight(DRIVE_POWER);
//            } else if (displacement > .5 && displacement < 2) {
//                strafe(STRAFE_POWER, "90", "LEFT");
//            } else if (displacement > 2 && displacement < 3) {
//                strafe(STRAFE_POWER, "90", "RIGHT");
//            } else if(displacement > 3 && !turnToAngle(0)) {
//            } else  {
//                resetDrive();
//                timeStep.clear();
//                pushed++;
//                step = 4;
//            }
//        }
        if (leftColor.toString().equals("RED")) {
            if (displacement > 0 && displacement < 1.5) {
                strafe(STRAFE_POWER, "90", "LEFT");
            } else if (displacement > 1.5 && displacement < 2.5) {
                strafe(STRAFE_POWER, "90", "RIGHT");
            } else if(displacement > 2.5 && !turnToAngle(0)) {
            } else {
                resetDrive();
                timeStep.clear();
                pushed++;
                step = 4;
            }
        } else if (leftColor.toString().equals("BLUE")) {
            if (displacement < .5) {
                straight(DRIVE_POWER);
            } else if (displacement > .5 && displacement < 2) {
                strafe(STRAFE_POWER, "90", "LEFT");
            } else if (displacement > 2 && displacement < 3) {
                strafe(STRAFE_POWER, "90", "RIGHT");
            } else if(displacement > 3 && !turnToAngle(0)) {
            } else  {
                resetDrive();
                timeStep.clear();
                pushed++;
                step = 4;
            }
        }
    }

    boolean shoot(int balls, double powerShoot, double powerConveyor)  {
        if(balls == 1)  {
            if(displacement < 3.5)   {
                if(displacement < 1.5)   {
                    hold.setPosition(UP_POSITION);
                    shoot.setPower(powerShoot);
                } else {
                    spin.setPower(powerConveyor);
                }
            } else  {
                resetShoot();
                return true;
            }
        } else if(balls == 2)   {
            if(displacement < 5.5)   {
                if(displacement < 1.5)   {
                    hold.setPosition(.5);
                    shoot.setPower(powerShoot);
                } else {
                    spin.setPower(powerConveyor);
                }
            } else  {
                resetShoot();
                return true;
            }
        }
        return false;
    }

    boolean move(String type, double time, double power, String angle, String direction) {
        if(displacement < time) {
            if (type.equals("STRAIGHT")) {
                straight(power);
            } else if(type.equals("TURN"))  {
                turn(power, direction);
            } else if(type.equals("STRAFE"))    {
                strafe(power, angle, direction);
            }
        } else  {
            resetDrive();
            return true;
        }
        return false;
    }

    void straight(double power) {
        driveLB.setPower(power);
        driveRB.setPower(power);
        driveLF.setPower(power);
        driveRF.setPower(power);
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

    boolean turnToAngle(double angle) {
        if(angleZ > angle + 2) {
            turn(.2, "RIGHT");
        } else if(angleZ < angle - 2)  {
            turn(.2, "LEFT");
        }
        if(angleZ < angle + 5 && angleZ > angle - 5)   {
            resetDrive();
            return true;
        } else  {
            return false;
        }
    }

    void strafe(double power, String angle, String direction)   {
        if(angle.equals("90"))  {
            if(direction.equals("LEFT"))    {
                driveLB.setPower(power);
                driveRB.setPower(-power);
                driveLF.setPower(-power);
                driveRF.setPower(power);
            } else if(direction.equals("RIGHT"))    {
                driveLB.setPower(-power);
                driveRB.setPower(power);
                driveLF.setPower(power);
                driveRF.setPower(-power);
            }
        } else if(angle.equals("45"))   {
            if(direction.equals("FORWARD_LEFT"))    {
                driveLB.setPower(power);
                driveRB.setPower(0);
                driveLF.setPower(0);
                driveRF.setPower(power);
            } else if(direction.equals("FORWARD_RIGHT"))    {
                driveLB.setPower(0);
                driveRB.setPower(power);
                driveLF.setPower(power);
                driveRF.setPower(0);
            } else if(direction.equals("BACKWARD_LEFT"))    {
                driveLB.setPower(-power);
                driveRB.setPower(0);
                driveLF.setPower(0);
                driveRF.setPower(-power);
            } else if(direction.equals("BACKWARD_RIGHT"))    {
                driveLB.setPower(0);
                driveRB.setPower(-power);
                driveLF.setPower(-power);
                driveRF.setPower(0);
            }
        }
    }

    void resetDrive() {
        driveLB.setPower(0);
        driveRB.setPower(0);
        driveLF.setPower(0);
        driveRF.setPower(0);
    }

    void resetShoot() {
        hold.setPosition(1);
        shoot.setPower(0);
        spin.setPower(0);
    }

    void resetRobot()   {
        driveLB.setPower(0);
        driveRB.setPower(0);
        driveLF.setPower(0);
        driveRF.setPower(0);
        hold.setPosition(1);
        shoot.setPower(0);
        spin.setPower(0);
    }

    @Override
    public void stop() {}
}

