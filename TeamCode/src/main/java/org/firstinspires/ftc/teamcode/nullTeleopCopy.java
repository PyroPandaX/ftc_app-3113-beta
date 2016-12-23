package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Servo;
import android.app.Activity;
import android.graphics.Color;
import android.view.View;

<<<<<<< HEAD
=======
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
>>>>>>> 410573ea0a0e0972a510999ba319691301cfa1b9
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

<<<<<<< HEAD
@TeleOp(name="NULL TeleOp", group="Teleop")
=======
@TeleOp(name="Copy of NULL", group="Teleop")
>>>>>>> 410573ea0a0e0972a510999ba319691301cfa1b9
//@Disabled
public class nullTeleopCopy extends OpMode {
    final static double MOTOR_POWER = 0.2;
    DcMotor motorRB, motorRF, motorLB, motorLF, spin, shoot;
    //ColorSensor colorSensor;
    //float hsvValues[] = {0F,0F,0F};
    //final float values[] = hsvValues;
    //final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(com.qualcomm.ftcrobotcontroller.R.id.RelativeLayout);
    //boolean bPrevState;
    //boolean bCurrState;
    //boolean bLedOn;
    public double joyRadius;
    Servo hold;
    public double right;
    public double left;
    public int mechDrive;
    public double mechVar;
    public double rightX;
    public double leftX;
    public double leftNet;
    public double rightNet;
    public double LF_Power, RF_Power, RB_Power, LB_Power, LF_Per, LB_Per, RB_Per, RF_Per;
    public double rawTotal;
    public double holdValue;
    public double servoPos;
    public double timeAuto = 0;
    public double timeStart = 0;
    public int autoState, count;
    public double time1;
    public boolean seq = true;

<<<<<<< HEAD
    public nullTeleopCopy() {}
=======
    public nullTeleopCopy() {
    }
>>>>>>> 410573ea0a0e0972a510999ba319691301cfa1b9

    public void init() {
        //bPrevState = false;
        //bCurrState = true;
        //bLedOn = true;
        motorRB = hardwareMap.dcMotor.get("motor_1");
        motorRF = hardwareMap.dcMotor.get("motor_2");
        motorLB = hardwareMap.dcMotor.get("motor_3");
        motorLF = hardwareMap.dcMotor.get("motor_4");
        spin = hardwareMap.dcMotor.get("spin");
        shoot = hardwareMap.dcMotor.get("shoot");

        //below is the PID control implemented
        shoot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hold = hardwareMap.servo.get("hold");
      /*  if (gamepad1.a && mechDrive <= 10) {
            mechDrive++;
        }
*/
    }

    @Override
    public void start() {
<<<<<<< HEAD
        //following initializes the timer variable and initializes the servo position
        timeAuto = 0;
        timeStart = this.time;
        hold.setPosition(1);
=======
        //following intializes the timer variable and initializes the servo position

        timeAuto = 0;
        timeStart = this.time;
        hold.setPosition(1);

>>>>>>> 410573ea0a0e0972a510999ba319691301cfa1b9
    }

    @Override
    public void loop() {
<<<<<<< HEAD
=======

>>>>>>> 410573ea0a0e0972a510999ba319691301cfa1b9
        right = gamepad1.left_stick_y;
        left = gamepad1.right_stick_y;
        leftX = gamepad1.right_stick_x;
        rightX = gamepad1.left_stick_x;

<<<<<<< HEAD
        //sets the maximum range of the joystick
=======
//sets the maximum range of the joystick
>>>>>>> 410573ea0a0e0972a510999ba319691301cfa1b9
        right = Range.clip(right, -1, 1);
        left = Range.clip(left, -1, 1);
        rightX = Range.clip(rightX, -1, 1);
        leftX = Range.clip(leftX, -1, 1);

        RB_Per = Range.clip(RB_Per, -1, 1);
        RF_Per = Range.clip(RF_Per, -1, 1);
        LF_Per = Range.clip(LF_Per, -1, 1);
        LB_Per = Range.clip(LB_Per, -1, 1);

        motorRB.setPower(RB_Per);
        motorLB.setPower(LB_Per);
        motorLF.setPower(LF_Per);
        motorRF.setPower(RF_Per);


<<<<<<< HEAD
        // upon pressing button b, will perform autoshoot command
=======
// upon pressing button b, will perform autoshoot command
>>>>>>> 410573ea0a0e0972a510999ba319691301cfa1b9
        if (gamepad2.b) {
            shootingSeq();
        }
        // kill switch button that stops the shooting and collecting mechanism
        if (gamepad2.x) {
            shoot.setPower(0);
            spin.setPower(0);
            hold.setPosition(1);
        }
<<<<<<< HEAD

        //auto collecting method implemented below
=======
//auto collecting method implemented below
>>>>>>> 410573ea0a0e0972a510999ba319691301cfa1b9
        if(gamepad2.a){
            hold.setPosition(1);
            shoot.setPower(0);
            spin.setPower(.7);
        }

        if (gamepad2.right_bumper)
            hold.setPosition(.5);
        if (gamepad2.left_bumper)
            hold.setPosition(1);

        if (gamepad2.right_trigger > .15)
            shoot.setPower(.35);
        else if (gamepad2.left_trigger > .15)
            shoot.setPower(0);

<<<<<<< HEAD
=======

>>>>>>> 410573ea0a0e0972a510999ba319691301cfa1b9
        if (gamepad2.dpad_up)
            spin.setPower(.7);
        else if (gamepad2.dpad_down)
            spin.setPower(-.7);
        else if (gamepad2.dpad_left)
            spin.setPower(0);
        else if (gamepad2.dpad_right)
            spin.setPower(.6);

<<<<<<< HEAD
=======

>>>>>>> 410573ea0a0e0972a510999ba319691301cfa1b9
        //references for joystick values
        RF_Power = (right - rightX + leftX);
        LF_Power = (-right + rightX + leftX);
        LB_Power = (-right - rightX + leftX);

        //Sum of all Joystick values. The small ".000000001" is used to simulate a limit function such that the code never divides by zero.
        RB_Power = Math.abs((right + rightX + leftX) + .000000001);

        //takes each joystick value and divides it by the total to yield a percent that has range of -1<=[motor]_Per <=1
        RB_Per = ((right / RB_Power) + (rightX / RB_Power) + (leftX / RB_Power));
        RF_Per = ((right / RB_Power) - (rightX / RB_Power) + (leftX / RB_Power));
        LB_Per = (-(right / RB_Power) - (rightX / RB_Power) + (leftX / RB_Power));
        LF_Per = (-(right / RB_Power) + (rightX / RB_Power) + (leftX / RB_Power));

<<<<<<< HEAD
        // radius of joystick calculated using pythagorean theorem
=======
// radius of joystick calculated using pythagorean theorem
>>>>>>> 410573ea0a0e0972a510999ba319691301cfa1b9
        joyRadius = Math.sqrt((right * right) + (rightX * rightX));

        //Threshold established that will allow robot to move slower if the raius of the joystick is less than threshold
        if (joyRadius < .5) {
            RB_Per = RB_Per / 2;
            RF_Per = RF_Per / 2;
            LF_Per = LF_Per / 2;
            LB_Per = LB_Per / 2;

        }
        telemetry.addData("1", "rightPower", "%5.2f", (right));
        telemetry.addData("2", "leftPower", "%5.2f", (left));
        telemetry.addData("C: mechVar", ":", String.format("%.24f", (joyRadius)));
        telemetry.addData(("4"), ":", String.format("%.24f", (rawTotal)));
        telemetry.update();
    }
<<<<<<< HEAD

=======
>>>>>>> 410573ea0a0e0972a510999ba319691301cfa1b9
    //delay method below
    public static void sleepCool(long sleepTime)
    {
        long wakeupTime = System.currentTimeMillis() + sleepTime;
<<<<<<< HEAD
=======

>>>>>>> 410573ea0a0e0972a510999ba319691301cfa1b9
        while (sleepTime > 0)
        {
            try
            {
                Thread.sleep(sleepTime);
            }
            catch (InterruptedException e)
            {
            }
            sleepTime = wakeupTime - System.currentTimeMillis();
        }
    } //sleep

<<<<<<< HEAD
=======

>>>>>>> 410573ea0a0e0972a510999ba319691301cfa1b9
    public void shootingSeq() {
        spin.setPower(0);
        hold.setPosition(1);
        shoot.setPower(.35);
        sleepCool(1500);
        hold.setPosition(.5);
        sleepCool(500);
        spin.setPower(.6);
<<<<<<< HEAD
    }

    public void autoCollect()
    {}
=======
        }



    public void autoCollect()
    {

    }











>>>>>>> 410573ea0a0e0972a510999ba319691301cfa1b9

    @Override
    public void stop() {
    }
}