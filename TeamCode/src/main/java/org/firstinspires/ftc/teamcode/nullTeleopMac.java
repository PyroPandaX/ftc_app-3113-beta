package org.firstinspires.ftc.teamcode;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="MacNULL", group="Teleop")
//@Disabled
public class nullTeleopMac extends OpMode {
    DcMotor RB, RF, LB, LF, spinner, shooter;
    double threshold = .1;
    double rightY = 0, leftY = 0, rightX = 0, leftX = 0;

    public nullTeleopMac() {}

    public void init() {
        RB = hardwareMap.dcMotor.get("motor_1");
        RF = hardwareMap.dcMotor.get("motor_2");
        LB = hardwareMap.dcMotor.get("motor_3");
        LF = hardwareMap.dcMotor.get("motor_4");
//        spinner = hardwareMap.dcMotor.get("spin");
//        shooter = hardwareMap.dcMotor.get("shoot");   `
    }

    @Override
    public void start() {}

    @Override
    public void loop() {
        gamepad();
        drive();
//        spin();
        tele();
    }

    /*
     * Convert gamepad values into float values within a range of -1 to 1
     */
    public void gamepad()   {
        rightY = gamepad1.right_stick_y;
        rightX = gamepad1.right_stick_x;
        leftY = -gamepad1.left_stick_y;
        leftX = -gamepad1.left_stick_x;

        rightY = Range.clip(rightY, -1, 1);
        leftY = Range.clip(leftY, -1, 1);
        rightX  = Range.clip(rightX, -1, 1);
        leftX  = Range.clip(leftX, -1, 1);
    }

    /*
     * Powering Mecanum drive motors in arcade mode
     */
    public void drive() {
        if(Math.abs(rightY) > threshold || Math.abs(rightX) > threshold)    {
            RF.setPower(((rightY-rightX)));
            LF.setPower(((-rightY-rightX)));
            RB.setPower(((-rightY-rightX)));
            LB.setPower(((rightY-rightX)));
        } else  {
            RF.setPower(0);
            LF.setPower(0);
            RB.setPower(0);
            LB.setPower(0);
        }
        if(Math.abs(leftX) > threshold) {
            RF.setPower((leftX));
            LF.setPower((leftX));
            RB.setPower((leftX));
            LB.setPower((leftX));
        } else  {
            RF.setPower(0);
            LF.setPower(0);
            RB.setPower(0);
            LB.setPower(0);
        }
    }

    /*
     * Powers Spinner and Shooter
     */
    public void spin()  {
        if(gamepad1.a) {spinner.setPower(1);}
        if(gamepad1.b) {spinner.setPower(0);}
        if(gamepad1.y) {spinner.setPower(-1);}
        if(gamepad1.dpad_up) {shooter.setPower(1);}
        if(gamepad1.dpad_down) {shooter.setPower(0);}
    }

    /*
     * Prints out telemetry
     */
    public void tele()  {
        telemetry.addData("RX", (rightX));
        telemetry.addData("RY", (rightY));
        telemetry.addData("LX", (leftX));
        telemetry.update();
    }

    @Override
    public void stop() {}
}