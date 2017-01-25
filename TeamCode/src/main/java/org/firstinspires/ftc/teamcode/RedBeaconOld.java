package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.opencv.core.Mat;

import java.util.ArrayList;

import ftc.vision.BeaconColorResult;
import ftc.vision.FrameGrabber;
import ftc.vision.ImageProcessorResult;

@Autonomous(name="StrafeRed", group="NullBot Beacon")
@Disabled
public class RedBeaconOld extends OpMode{
    FrameGrabber frameGrabber = FtcRobotControllerActivity.frameGrabber; //Get the frameGrabber
    DcMotor motorRB, motorRF, motorLB, motorLF, spin, shoot;
    double timeAuto, timeStart, timeLine, timeColor, timeLine2, timeColor2, timePushed;
    ArrayList<Double> timeStep = new ArrayList<Double>();
    Servo hold, push;
    byte[] colorCcache;
    I2cDevice colorC;
    I2cDeviceSynch colorCreader;
    BeaconColorResult result;
    boolean sawLine = false, sawLine2 = false;
    ModernRoboticsI2cGyro gyro;
    int xVal, yVal, zVal, heading, angleZ, resetState;
    int countColor = 0, countWhite = 0, countZero = 0, countWhite2 = 0, countZero2 = 0, countPushed = 0, countColor2 = 0;

    public RedBeaconOld()  {}

    public void init() {
        motorRF = hardwareMap.dcMotor.get("motor_1");
        motorRB = hardwareMap.dcMotor.get("motor_2");
        motorLB = hardwareMap.dcMotor.get("motor_3");
        motorLF = hardwareMap.dcMotor.get("motor_4");
        motorRB.setDirection(DcMotor.Direction.REVERSE);
        motorRF.setDirection(DcMotor.Direction.REVERSE);
        hold = hardwareMap.servo.get("hold");
        push = hardwareMap.servo.get("push");
        spin = hardwareMap.dcMotor.get("spin");
        shoot = hardwareMap.dcMotor.get("shoot");
        shoot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        colorC = hardwareMap.i2cDevice.get("line");
        colorCreader = new I2cDeviceSynchImpl(colorC, I2cAddr.create8bit(0x3c), false);
        colorCreader.engage();
        colorCreader.write8(3, 0);
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
        hold.setPosition(1);
    }

    @Override
    public void start() {
        // defines timeStart as the timer at the start of autonomous to preserve an initial value
        timeStart = this.time;
    }

    @Override
    public void loop() {
        heading = gyro.getHeading();
        angleZ = gyro.getIntegratedZValue();
        // get the x, y, and z values (rate of change of angle).
        xVal = gyro.rawX();
        yVal = gyro.rawY();
        zVal = gyro.rawZ();

        timeAuto = this.time - timeStart;
        colorCcache = colorCreader.read(0x04, 1);

        if (timeAuto < 1.5) {
            motorLB.setPower(0);
            motorRB.setPower(0);
            motorLF.setPower(0);
            motorRF.setPower(0);
            hold.setPosition(.5);
            shoot.setPower(.35);
        } else if (timeAuto > 1.5 && timeAuto < 5.5) {
            spin.setPower(.6);
        } else if (timeAuto > 5.5 && timeAuto < 10) {
            hold.setPosition(1);
            shoot.setPower(0);
            spin.setPower(0);
            motorLB.setPower(.7);
            motorRB.setPower(0);
            motorLF.setPower(0);
            motorRF.setPower(.7);
        } else if (timeAuto > 10 && timeAuto < 11 && zero()) {
            motorLB.setPower(-.2);
            motorRB.setPower(-.2);
            motorLF.setPower(-.2);
            motorRF.setPower(-.2);
        } else if (!sawLine) {
            if (colorCcache[0] > 6) {
                motorLB.setPower(0);
                motorRB.setPower(0);
                motorLF.setPower(0);
                motorRF.setPower(0);
                if (countWhite == 0) {
                    sawLine = true;
                    countWhite++;
                }
            } else {
                motorLB.setPower(.2);
                motorRB.setPower(.2);
                motorLF.setPower(.2);
                motorRF.setPower(.2);
            }
        }

        if (sawLine) {
            if (zero() && countZero == 0) {
                timeStep.add(this.time);
                countZero++;
            }
            if (countZero == 1) {
                //            if (timeLine < .5) {
                //                motorLB.setPower(-.7);
                //                motorRB.setPower(.7);
                //                motorLF.setPower(.7);
                //                motorRF.setPower(-.7);
                //            } else if(timeLine > .5) {
                timeLine = this.time - timeStep.get(0);
                frameGrabber.grabSingleFrame();
                while (!frameGrabber.isResultReady()) {
                    sleepCool(5); //sleep for 5 milliseconds
                }
                ImageProcessorResult imageProcessorResult = frameGrabber.getResult();
                result = (BeaconColorResult) imageProcessorResult.getResult();
                BeaconColorResult.BeaconColor leftColor = result.getLeftColor();
                BeaconColorResult.BeaconColor rightColor = result.getRightColor();

                if (leftColor.toString().equals("RED")) {
                    if (countColor == 0) {
                        timeStep.add(timeAuto);
                        countColor++;
                    }                 if (countColor == 0) {
                        timeStep.add(this.time);
                        countColor++;
                    }
                    timeColor = this.time - timeStep.get(1);
                    if (timeColor < 1.5) {
                        motorLB.setPower(.7);
                        motorRB.setPower(-.7);
                        motorLF.setPower(-.7);
                        motorRF.setPower(.7);
                    } else if (timeColor > 1.5 && timeColor < 2.5) {
                        motorLB.setPower(-.7);
                        motorRB.setPower(.7);
                        motorLF.setPower(.7);
                        motorRF.setPower(-.7);
                    }
                } else if (leftColor.toString().equals("BLUE")) {
                    if (countColor == 0) {
                        timeStep.add(this.time);
                        countColor++;
                    }
                    timeColor = this.time - timeStep.get(1);
                    if (timeColor < .5) {
                        motorLB.setPower(.2);
                        motorRB.setPower(.2);
                        motorLF.setPower(.2);
                        motorRF.setPower(.2);
                    } else if (timeColor > .5 && timeColor < 2) {
                        motorLB.setPower(.7);
                        motorRB.setPower(-.7);
                        motorLF.setPower(-.7);
                        motorRF.setPower(.7);
                    } else if (timeColor > 2 && timeColor < 3.5) {
                        motorLB.setPower(-.7);
                        motorRB.setPower(.7);
                        motorLF.setPower(.7);
                        motorRF.setPower(-.7);
                    }
                }

//                if (leftColor.toString().equals("RED") && rightColor.toString().equals("RED")) {
//                    if (countPushed == 0 && zero()) {
//                        timeStep.add(timeAuto);
//                        countPushed++;
//                    }
//                }
                //timePushed = this.time - timeStep.get(3);
//                if (countPushed == 1) {
//                      if(timePushed < .5) {
//                          motorLB.setPower(.2);
//                          motorRB.setPower(.2);
//                          motorLF.setPower(.2);
//                          motorRF.setPower(.2);
//                      }
//                    else if (timePushed > .5) {
//                        if (!sawLine2) {
//                            if (colorCcache[0] > 6) {
//                                motorLB.setPower(0);
//                                motorRB.setPower(0);
//                                motorLF.setPower(0);
//                                motorRF.setPower(0);
//                                if (countWhite2 == 0) {
//                                    sawLine = true;
//                                    countWhite2++;
//                                }
//                            } else {
//                                motorLB.setPower(.2);
//                                motorRB.setPower(.2);
//                                motorLF.setPower(.2);
//                                motorRF.setPower(.2);
//                            }
//                        }
//                    }
//                }
//                if (sawLine2) {
//                    if (zero() && countZero2 == 0) {
//                        timeStep.add(this.time);
//                        countZero2++;
//                    }
//                    if (countZero2 == 1) {
//                        timeLine2 = this.time - timeStep.get(4);
//                        frameGrabber.grabSingleFrame();
//                        while (!frameGrabber.isResultReady()) {
//                            sleepCool(5); //sleep for 5 milliseconds
//                        }
//                        ImageProcessorResult imageProcessorResult2 = frameGrabber.getResult();
//                        result = (BeaconColorResult) imageProcessorResult.getResult();
//                        BeaconColorResult.BeaconColor leftColor2 = result.getLeftColor();
//                        BeaconColorResult.BeaconColor rightColor2 = result.getRightColor();
//
//                        if (leftColor.toString().equals("RED")) {
//                            if (countColor2 == 0) {
//                                timeStep.add(this.time);
//                                countColor2++;
//                            }
//                            timeColor2 = this.time - timeStep.get(5);
//                            if (timeColor2 < 1.5) {
//                                motorLB.setPower(.7);
//                                motorRB.setPower(-.7);
//                                motorLF.setPower(-.7);
//                                motorRF.setPower(.7);
//                            } else if (timeColor2 > 1.5 && timeColor2 < 3) {
//                                motorLB.setPower(-.7);
//                                motorRB.setPower(.7);
//                                motorLF.setPower(.7);
//                                motorRF.setPower(-.7);
//                            }
//                        } else if (leftColor.toString().equals("BLUE")) {
//                            if (countColor2 == 0) {
//                                timeStep.add(this.time);
//                                countColor2++;
//                            }
//                            timeColor2 = this.time - timeStep.get(4);
//                            if (timeColor2 < .5) {
//                                motorLB.setPower(.2);
//                                motorRB.setPower(.2);
//                                motorLF.setPower(.2);
//                                motorRF.setPower(.2);
//                            } else if (timeColor2 > .5 && timeColor2 < 2) {
//                                motorLB.setPower(.7);
//                                motorRB.setPower(-.7);
//                                motorLF.setPower(-.7);
//                                motorRF.setPower(.7);
//                            } else if (timeColor2 > 2 && timeColor2 < 3.5) {
//                                motorLB.setPower(-.7);
//                                motorRB.setPower(.7);
//                                motorLF.setPower(.7);
//                                motorRF.setPower(-.7);
//                            }
//                        }
//                    }
//                }
            }
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
    public static void sleepCool(long sleepTime)    {
        long wakeupTime = System.currentTimeMillis() + sleepTime;
        while (sleepTime > 0) {
            try {
                Thread.sleep(sleepTime);
            }
            catch (InterruptedException e) {}
            sleepTime = wakeupTime - System.currentTimeMillis();
        }
    } //sleep

    public boolean zero() {
        if(angleZ > 2) {
            motorLB.setPower(.2);
            motorRB.setPower(-.2);
            motorLF.setPower(.2);
            motorRF.setPower(-.2);
        } else if(angleZ < -2)  {
            motorLB.setPower(-.2);
            motorRB.setPower(.2);
            motorLF.setPower(-.2);
            motorRF.setPower(.2);
        }

        if(angleZ < 5 && angleZ > -5)   {
            return true;
        } else  {
            return false;
        }
    }

    @Override
    public void stop() {}
}

