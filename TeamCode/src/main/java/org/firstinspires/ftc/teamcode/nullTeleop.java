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

@TeleOp(name="NULL", group="Teleop")
//@Disabled
public class nullTeleop extends OpMode {
    final static double MOTOR_POWER = 0.2;
    DcMotor motorRB, motorRF, motorLB, motorLF, spin, shoot;
    //ColorSensor colorSensor;
    //float hsvValues[] = {0F,0F,0F};
    //final float values[] = hsvValues;
    //final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(com.qualcomm.ftcrobotcontroller.R.id.RelativeLayout);
    //boolean bPrevState;
    //boolean bCurrState;
    //boolean bLedOn;
    public double right;
    public double left;
    public int mechDrive;
    public double mechVar;
    public double rightX;
    public double leftX;
    public double leftNet;
    public double rightNet;
    public double LF_Power, RF_Power,RB_Power, LB_Power, LF_Per, LB_Per, RB_Per, RF_Per;
    public double rawTotal;


    public nullTeleop() {
    }

    public void init() {
        //bPrevState = false;
        //bCurrState = true;
        //bLedOn = true;
        motorRB = hardwareMap.dcMotor.get("motor_1");
        motorRF = hardwareMap.dcMotor.get("motor_2");
        motorLB = hardwareMap.dcMotor.get("motor_3");
        motorLF = hardwareMap.dcMotor.get("motor_4");

       // spin = hardwareMap.dcMotor.get("spin");
        //shoot = hardwareMap.dcMotor.get("shoot");
      /*  if (gamepad1.a && mechDrive <= 10) {
            mechDrive++;
        }
*/

    }
    @Override
    public void start() {
    }

    @Override
    public void loop() {


        right = gamepad1.right_stick_y;
        left = -gamepad1.left_stick_y;
        leftX = -gamepad1.left_stick_x;
        rightX = gamepad1.right_stick_x;


//sets the maximum range of the joystick
        right = Range.clip(right, -1, 1);
        left = Range.clip(left, -1, 1);
        rightX  = Range.clip(rightX, -1, 1);
        leftX  = Range.clip(leftX, -1, 1);


        RB_Per  = Range.clip(RB_Per, -1, 1);
        RF_Per  = Range.clip(RF_Per, -1, 1);
        LF_Per = Range.clip(LF_Per, -1, 1);
        LB_Per  = Range.clip(LB_Per, -1, 1);

        motorRB.setPower(RB_Per);
        motorLB.setPower(LB_Per);
        motorLF.setPower(LF_Per);
        motorRF.setPower(RF_Per);

        //references for joystick values
        RF_Power = (right - rightX + leftX);
        LF_Power = (-right + rightX + leftX);
        LB_Power =(-right - rightX + leftX);

        //Sum of all Joystick values. The small ".000000001" is used to simulate a limit function such that the code never divides by zero.
        RB_Power = Math.abs((right + rightX + leftX) + .000000001);

        //takes each joystick value and divides it by the total to yield a percent that has range of -1<=[motor]_Per <=1
        RB_Per = ((right/RB_Power) + (rightX/RB_Power) + (leftX/RB_Power));
        RF_Per = ((right/RB_Power) - (rightX/RB_Power) + (leftX/RB_Power));
        LB_Per = (-(right/RB_Power) - (rightX/RB_Power) + (leftX/RB_Power));
        LF_Per = (-(right/RB_Power) + (rightX/RB_Power) + (leftX/RB_Power));


        telemetry.addData("1", "rightPower", "%5.2f", (right));
        telemetry.addData("2", "leftPower", "%5.2f", (left));
        telemetry.addData("C: mechVar",":",  String.format("%.24f",(mechVar)));
        telemetry.addData(("4"), ":", String.format("%.24f", (rawTotal)));
        telemetry.update();

    }

    @Override
    public void stop() {}
}