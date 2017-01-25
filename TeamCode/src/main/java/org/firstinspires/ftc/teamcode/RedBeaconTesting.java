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

@Autonomous(name="Red Beacon Testing", group="NullBot Beacon")
//@Disabled
public class RedBeaconTesting extends OpMode{
    FrameGrabber frameGrabber = FtcRobotControllerActivity.frameGrabber; //Get the frameGrabber
    DcMotor motorRB, motorRF, motorLB, motorLF, spin, shoot;
    double timeAuto, timeColor;
    ArrayList<Double> timeStep = new ArrayList<>();
    Servo hold;
    byte[] colorCcache;
    I2cDevice colorC;
    I2cDeviceSynch colorCreader;
    BeaconColorResult result;
    boolean sawLine = false, strafe = false;
    ModernRoboticsI2cGyro gyro;
    int xVal, yVal, zVal, heading, angleZ, resetState, countWhite = 0, countPushed = 0;
    ElapsedTime elapsed = new ElapsedTime();

    public RedBeaconTesting()  {}

    public void init() {
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
        colorC = hardwareMap.i2cDevice.get("line");
        colorCreader = new I2cDeviceSynchImpl(colorC, I2cAddr.create8bit(0x3c), false);
        colorCreader.engage();
        colorCreader.write8(3, 0);
        gyro = (ModernRoboticsI2cGyro) hardwareMap.gyroSensor.get("gyro");
        hold.setPosition(1);
    }

    @Override
    public void start() {
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
        elapsed.reset();
    }

    @Override
    public void loop() {
        heading = gyro.getHeading();
        angleZ = gyro.getIntegratedZValue();
        // get the x, y, and z values (rate of change of angle).
        xVal = gyro.rawX();
        yVal = gyro.rawY();
        zVal = gyro.rawZ();
        timeAuto = elapsed.time();
        colorCcache = colorCreader.read(0x04, 1);

        if (timeAuto < 1.5) {
            hold.setPosition(.5);
            shoot.setPower(.6);
        } else if (timeAuto > 1.5 && timeAuto < 5.5) {
            spin.setPower(.6);
        } else if (timeAuto > 5.5 && timeAuto < 10) {
            resetShoot();
            strafe(.7, "45", "FORWARD_LEFT");
        } else if (timeAuto > 10 && timeAuto < 11 && zero()) {
            straight(-.2);
        } else if (!sawLine) {
            if (colorCcache[0] > 6) {
                stopDrive();
                if (countWhite == 0) {
                    sawLine = true;
                    strafe = true;
                    countWhite++;
                }
            } else {
                straight(.2);
            }
        }

        if (strafe) {
                frameGrabber.grabSingleFrame();
                while (!frameGrabber.isResultReady()) {
                    sleepCool(5); //sleep for 5 milliseconds
                }
                ImageProcessorResult imageProcessorResult = frameGrabber.getResult();
                result = (BeaconColorResult) imageProcessorResult.getResult();
                BeaconColorResult.BeaconColor leftColor = result.getLeftColor();
                BeaconColorResult.BeaconColor rightColor = result.getRightColor();
                timeStep.add(elapsed.time());
                timeColor = elapsed.time() - timeStep.get(0);
                if (leftColor.toString().equals("RED") || rightColor.toString().equals("BLUE") && !rightColor.toString().equals("RED")) {
                    if (timeColor < .5) {
                        strafe(.7, "90", "LEFT");
                    } else if (timeColor > .5 && timeColor < 4.5) {
                        strafe(.7, "90", "RIGHT");
                    } else  {
                        stopDrive();
                    }
                } else if (leftColor.toString().equals("BLUE") || rightColor.toString().equals("RED") && !leftColor.toString().equals("RED")) {
                    if (timeColor < .5) {
                        straight(.2);
                    } else if (timeColor > .5 && timeColor < 1) {
                        strafe(.7, "90", "LEFT");
                    } else if (timeColor > 1 && timeColor < 1.5) {
                        strafe(.7, "90", "RIGHT");
                    } else  {
                        stopDrive();
                    }
                }
            strafe = false;
            }

        telemetry.addData("Result", result);
        telemetry.addData("", "Int. Ang. %03d", angleZ);
        telemetry.addData("White", sawLine);
        telemetry.addData("Pushed", countPushed);
        telemetry.update();
        //wait before quitting (quitting clears telemetry)
        //sleepCool(1);
    }

    //delay method below
    static void sleepCool(long sleepTime)    {
        long wakeupTime = System.currentTimeMillis() + sleepTime;
        while (sleepTime > 0) {
            try {
                Thread.sleep(sleepTime);
            }
            catch (InterruptedException e) {}
            sleepTime = wakeupTime - System.currentTimeMillis();
        }
    }

    boolean zero() {
        if(angleZ > 2) {
            turn(.2, "LEFT");
        } else if(angleZ < -2)  {
            turn(.2, "RIGHT");
        }
        if(angleZ < 5 && angleZ > -5)   {
            return true;
        } else  {
            return false;
        }
    }

    void straight(double power) {
        motorLB.setPower(power);
        motorRB.setPower(power);
        motorLF.setPower(power);
        motorRF.setPower(power);
    }

    void stopDrive() {
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

    void curve()    {}



    @Override
    public void stop() {}
}

