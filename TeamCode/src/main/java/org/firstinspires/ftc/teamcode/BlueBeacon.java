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

import ftc.vision.BeaconColorResult;
import ftc.vision.FrameGrabber;
import ftc.vision.ImageProcessorResult;

/*
code description
 */
@Autonomous(name="Blue Beacon", group="NullBot Beacon")
//@Disabled
public class BlueBeacon extends OpMode{
    //hardware variables
    DcMotor motorRB, motorRF, motorLB, motorLF, spin, shoot; //add lift motors here
    Servo hold;
    //sensor variables
    FrameGrabber frameGrabber = FtcRobotControllerActivity.frameGrabber;
    byte[] colorCcache;
    I2cDevice colorC;
    I2cDeviceSynch colorCreader;
    BeaconColorResult result;
    ModernRoboticsI2cGyro gyro;
    int xVal, yVal, zVal, heading, angleZ, resetState;
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

    public BlueBeacon()  {}

    public void init() {
        //hardware config
        motorRF = hardwareMap.dcMotor.get("motor_1");
        motorRB = hardwareMap.dcMotor.get("motor_2");
        motorLB = hardwareMap.dcMotor.get("motor_3");
        motorLF = hardwareMap.dcMotor.get("motor_4");
        motorRB.setDirection(DcMotor.Direction.REVERSE);
        motorRF.setDirection(DcMotor.Direction.REVERSE);
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

        if(step == 0 && shoot(2, SHOOT_POWER, CONVEYOR_POWER))
            step++;
        else if(step == 1 && move("STRAFE", 6.5, STRAFE_POWER, "45", "FORWARD_RIGHT"))
            step++;
        else if(step == 2 && turnToAngle(180))
            step++;
        else if(step == 3 && move("STRAIGHT", 1, DRIVE_POWER, "", ""))
            step++;
        else if(step == 4 && pushed <= 1) {
            if(white()) {
                resetDrive();
                timeStep.clear();
                step++;
            } else {
                straight(-DRIVE_POWER);
            }
        } else if(pushed > 1)   {
            step = 7;
        } else if(step == 5 && move("STRAIGHT", .5, -DRIVE_POWER, "", ""))    {
            step++;
        } else if(step == 6) {
            frameGrabber.grabSingleFrame();
            while (!frameGrabber.isResultReady()) {
                sleepCool(5);
            }
            ImageProcessorResult imageProcessorResult = frameGrabber.getResult();
            result = (BeaconColorResult) imageProcessorResult.getResult();
            BeaconColorResult.BeaconColor leftColor = result.getLeftColor();
            BeaconColorResult.BeaconColor rightColor = result.getRightColor();
            if (rightColor.toString().equals("BLUE") || leftColor.toString().equals("RED") && !leftColor.toString().equals("BLUE")) {
                if (displacement < .5) {
                    strafe(STRAFE_POWER, "90", "LEFT");
                } else if (displacement > .5 && displacement < 1) {
                    strafe(STRAFE_POWER, "90", "RIGHT");
                } else  {
                    resetDrive();
                    timeStep.clear();
                    pushed++;
                    step = 4;
                }
            } else if (leftColor.toString().equals("BLUE") || rightColor.toString().equals("RED") && !rightColor.toString().equals("BLUE")) {
                if (displacement < .5) {
                    straight(DRIVE_POWER);
                } else if (displacement > .5 && displacement < 1) {
                    strafe(STRAFE_POWER, "90", "LEFT");
                } else if (displacement > 1 && displacement < 1.5) {
                    strafe(STRAFE_POWER, "90", "RIGHT");
                } else  {
                    resetDrive();
                    timeStep.clear();
                    pushed++;
                    step = 4;
                }
            }
        } else if(step == 7)    {
            resetRobot();
        }

        telemetry.addData("Result", result);
        telemetry.addData("", "Int. Ang. %03d", angleZ);
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

    boolean white() {
        if(colorCcache[0] > 6)
            return true;
        return false;
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
                timeStep.clear();
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
                timeStep.clear();
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
            timeStep.clear();
            return true;
        }
        return false;
    }

    void straight(double power) {
        motorLB.setPower(power);
        motorRB.setPower(power);
        motorLF.setPower(power);
        motorRF.setPower(power);
    }

    void turn(double power, String direction) {
        if(direction.equals("LEFT")) {
            motorLB.setPower(-power);
            motorRB.setPower(power);
            motorLF.setPower(-power);
            motorRF.setPower(power);
        } else if(direction.equals("RIGHT"))    {
            motorLB.setPower(power);
            motorRB.setPower(-power);
            motorLF.setPower(power);
            motorRF.setPower(-power);
        }
    }

    boolean turnToAngle(double angle) {
        if(angleZ > angle + 2) {
            turn(.2, "LEFT");
        } else if(angleZ < angle - 2)  {
            turn(.2, "RIGHT");
        }
        if(angleZ < 5 + angle && angleZ > angle - 5)   {
            resetDrive();
            timeStep.clear();
            return true;
        } else  {
            return false;
        }
    }

    void strafe(double power, String angle, String direction)   {
        if(angle.equals("90"))  {
            if(direction.equals("LEFT"))    {
                motorLB.setPower(power);
                motorRB.setPower(-power);
                motorLF.setPower(-power);
                motorRF.setPower(power);
            } else if(direction.equals("RIGHT"))    {
                motorLB.setPower(-power);
                motorRB.setPower(power);
                motorLF.setPower(power);
                motorRF.setPower(-power);
            }
        } else if(angle.equals("45"))   {
            if(direction.equals("FORWARD_LEFT"))    {
                motorLB.setPower(power);
                motorRB.setPower(0);
                motorLF.setPower(0);
                motorRF.setPower(power);
            } else if(direction.equals("FORWARD_RIGHT"))    {
                motorLB.setPower(0);
                motorRB.setPower(power);
                motorLF.setPower(power);
                motorRF.setPower(0);
            } else if(direction.equals("BACKWARD_LEFT"))    {
                motorLB.setPower(-power);
                motorRB.setPower(0);
                motorLF.setPower(0);
                motorRF.setPower(-power);
            } else if(direction.equals("BACKWARD_RIGHT"))    {
                motorLB.setPower(0);
                motorRB.setPower(-power);
                motorLF.setPower(-power);
                motorRF.setPower(0);
            }
        }
    }

    void resetDrive() {
        motorLB.setPower(0);
        motorRB.setPower(0);
        motorLF.setPower(0);
        motorRF.setPower(0);
    }

    void resetShoot() {
        hold.setPosition(1);
        shoot.setPower(0);
        spin.setPower(0);
    }

    void resetRobot()   {
        motorLB.setPower(0);
        motorRB.setPower(0);
        motorLF.setPower(0);
        motorRF.setPower(0);
        hold.setPosition(1);
        shoot.setPower(0);
        spin.setPower(0);
    }

    @Override
    public void stop() {}
}

