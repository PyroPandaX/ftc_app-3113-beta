package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class DemoTest extends OpMode {
    final static double MOTOR_POWER = 0.2;
    DcMotor motorRF, motorRB;
    DcMotor motorLF, motorLB;


    public DemoTest() {

    }

    public void init() {
        motorRF = hardwareMap.dcMotor.get("motor_1");
        motorRB = hardwareMap.dcMotor.get("motor_2");
        motorLF = hardwareMap.dcMotor.get("motor_3");
        motorLB = hardwareMap.dcMotor.get("motor_4");
        motorLB.setDirection(DcMotor.Direction.REVERSE);
        motorLF.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void start() {

    }

    @Override
    public void loop() {
        if (this.time <= 3) {
            motorRF.setPower(MOTOR_POWER);
            motorLF.setPower(MOTOR_POWER);
            motorRB.setPower(MOTOR_POWER);
            motorLB.setPower(MOTOR_POWER);
        }
        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("time", "elapsed time: " + Double.toString(this.time));
    }

    @Override
    public void stop() {

    }
}

