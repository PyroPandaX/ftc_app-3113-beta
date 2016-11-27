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

@TeleOp(name="Macnull", group="Teleop")
//@Disabled
public class nullTeleopMac extends OpMode {
    DcMotor motorRB, motorRF, motorLB, motorLF;
    //spinner, shooter;
    int threshold = 10;
    float rightY, leftY, rightX, leftX;
   // boolean spin, shoot;

    public nullTeleopMac() {
    }

    public void init() {
        motorRB = hardwareMap.dcMotor.get("motor_1");
        motorRF = hardwareMap.dcMotor.get("motor_2");
        motorLB = hardwareMap.dcMotor.get("motor_3");
        motorLF = hardwareMap.dcMotor.get("motor_4");
     //   spinner = hardwareMap.dcMotor.get("spin");
       // shooter = hardwareMap.dcMotor.get("shoot");
    }
    @Override
    public void stop() {}

    @Override
    public void start() {
    }

    @Override
    public void loop() {
        rightY = gamepad1.right_stick_y;
        rightX = gamepad1.right_stick_x;
        leftY = -gamepad1.left_stick_y;
        leftX = -gamepad1.left_stick_x;

        if(Math.abs(rightY) > threshold || Math.abs(rightX) > threshold)    {
            motorRF.setPower((rightY-rightX)/2);
            motorLF.setPower((-rightY-rightX)/2);
            motorRB.setPower((-rightY-rightX)/2);
            motorLB.setPower((rightY-rightX)/2);
        }
        if(Math.abs(leftX) > threshold) {
            motorRF.setPower(leftX/2);
            motorLF.setPower(leftX/2);
            motorRB.setPower(leftX/2);
            motorLB.setPower(leftX/2);
        }
/*
        if(gamepad1.a)
            spinner.setPower(1);
        if(gamepad1.b)
            spinner.setPower(0);
        if(gamepad1.y)
            spinner.setPower(-1);
        if(gamepad1.dpad_up)
            shooter.setPower(1);
        if(gamepad1.dpad_down)
            shooter.setPower(0);
*/
        telemetry.addData(">", "X", "%5.2f", (rightX));
        telemetry.addData(">", "Y", "%5.2f", (rightY));
        telemetry.addData(">", "Rot", "%5.2f", (leftX));
//        telemetry.addData(">", "Spin", "%5.2f", (spin));
//        telemetry.addData(">", "Shoot", "%5.2f", (shoot));
        telemetry.update();

    }
}