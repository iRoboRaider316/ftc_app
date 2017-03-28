package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Reeeeecerooni the rerece", group="TeleOp")

public class cf_teleOp_Reecerooni extends OpMode{
    ColorSensor color;
    DcMotor rDrive1;
    DcMotor rDrive2;
    DcMotor lDrive1;
    DcMotor lDrive2;
    DcMotor catapult;
    DcMotor sweeper;
    DcMotor lift1;
    DcMotor lift2;
    Servo hopper;
    Servo button;
    Servo belt;
    TouchSensor touch;
    int drive = 0;

    ElapsedTime timer = new ElapsedTime();

    enum LaunchState {
        Idle, Pressed, NotPressed, Load, Open
    }

    public void init() {
        rDrive1 = hardwareMap.dcMotor.get("rDrive1");
        rDrive2 = hardwareMap.dcMotor.get("rDrive2");
        lDrive1 = hardwareMap.dcMotor.get("lDrive1");
        lDrive2 = hardwareMap.dcMotor.get("lDrive2");
        sweeper = hardwareMap.dcMotor.get("sweeper");
        catapult = hardwareMap.dcMotor.get("catapult");
        lift1 = hardwareMap.dcMotor.get("lift1");
        lift2 = hardwareMap.dcMotor.get("lift2");
        hopper = hardwareMap.servo.get("hopper");
        touch = hardwareMap.touchSensor.get("t");
        color = hardwareMap.colorSensor.get("color");
        button = hardwareMap.servo.get("button");
        belt = hardwareMap.servo.get("belt");
        rDrive1.setDirection(DcMotor.Direction.REVERSE);
        rDrive2.setDirection(DcMotor.Direction.REVERSE);

        belt.setPosition(.5);
        button.setPosition(.5);
        hopper.setPosition(.8);


    }

    private LaunchState launchState = LaunchState.Idle;

    public void loop() {
        belt.setPosition(.5);
        float rStick1 = gamepad1.left_stick_y;
        float lStick1 = gamepad1.right_stick_y;
        float lStick2 = gamepad2.left_stick_y;
        float rStick2 = gamepad2.right_stick_y;
        boolean up = gamepad2.dpad_up;
        boolean left = gamepad2.dpad_left;
        boolean down = gamepad2.dpad_down;
        boolean y = gamepad2.y;
        boolean x = gamepad2.x;
        boolean a = gamepad2.a;
        boolean b = gamepad2.b;
        boolean rBumper1 = gamepad1.right_bumper;
        boolean lBumper1 = gamepad1.left_bumper;
        boolean rBumper2 = gamepad2.right_bumper;
        boolean lBumper2 = gamepad2.left_bumper;
        float leftPower;
        float rightPower;
        float lTrigger2 = gamepad2.left_trigger;
        float speed = 1;

        ///OPERATOR CODE\\\
        double noPower = 0.0;
        double firingPower = 1;
        /*if ( gamepad2.left_bumper ) {
            if ( touch.isPressed() ) {
                //catapult.setPower(noPower);
                hopper.setPosition(.5);

            }
            else {
                catapult.setPower(firingPower);
            }
        }*/

        if(!rBumper2) {
            launchState = LaunchState.Idle;
            if (y) {
                catapult.setPower(1);
            } else if (a) {
                catapult.setPower(-1);
            } else {
                catapult.setPower(0);
            }
            if (b) {
                hopper.setPosition(.5);
            } else {
                hopper.setPosition(.8);
            }
        }

        switch(launchState) {
            case Idle:
                if(rBumper2) {
                    launchState = LaunchState.Pressed;
                    catapult.setPower(1);
                }
                break;

            case Pressed:
                if(touch.isPressed()) {
                    launchState = LaunchState.NotPressed;
                }
                break;

            case NotPressed:
                if(!touch.isPressed()) {
                    timer.reset();
                    launchState = LaunchState.Load;
                }
                break;

            case Load:
                if(timer.milliseconds() > 500) {
                    hopper.setPosition(0.5);
                    timer.reset();
                    launchState = LaunchState.Open;
                }
                break;

            case Open:
                if(timer.milliseconds() > 500) {
                    hopper.setPosition(0.8);
                    launchState = LaunchState.Idle;
                }
        }

        if (up) {
            sweeper.setPower(-1);
        } else if (down) {
            sweeper.setPower(1);
        } else if (left) {
            sweeper.setPower(0);
        }
        /*if (y) {
            catapult.setPower(1);
        } else if (a) {
            catapult.setPower(-1);
        } else {
            catapult.setPower(0);
        }
        if (b) {
            hopper.setPosition(.5);
        } else {
            hopper.setPosition(.8);
        }*/

        lift1.setPower(gamepad2.right_stick_y);
        lift2.setPower(gamepad2.right_stick_y);

        if (lStick2 > .5)
            belt.setPosition(0);
        else if (lStick2 < -.5)
            belt.setPosition(1);
        else
            belt.setPosition(.5);


        ///DRIVER CODE\\\


        //This code controls the side button pusher
        if (gamepad1.b)
            button.setPosition(0);
        else if (gamepad1.x)
            button.setPosition(1);
        else
            button.setPosition(0.5);

        if (gamepad1.dpad_left) {
            sweeper.setPower(-1);
        } else if (gamepad1.dpad_right) {
            sweeper.setPower(0);
        }

        // This sets the power of the drive motors to based on the joystick position using an Exponential Scale Algorithm
        if (gamepad1.a) { // Backward 100%
            drive = 1;
        }
        if (gamepad1.y) { //Forward 100%
            drive = 2;
        }
        if (gamepad1.right_bumper) { //1/2 speed
            speed = 2;
        }
        if (gamepad1.left_bumper) { //full speed
            speed = 1;
        }
        if (gamepad1.dpad_up) { //Forward in a straight line
            drive = 3;
        }
        if (gamepad1.dpad_down) { // Backward in a straight line
            drive = 4;
        }

        switch (drive) {
            case 1:
                // forward default
                rightPower = ((-gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y))/speed);
                leftPower = ((-gamepad1.right_stick_y * Math.abs(gamepad1.right_stick_y))/speed);
                drive = 1;
                break;
            case 2:
                // backward
                rightPower = ((gamepad1.right_stick_y * Math.abs(gamepad1.right_stick_y))/speed);
                leftPower = ((gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y))/speed);
                drive = 2;
                break;
            case 3:
                rightPower = Math.abs(gamepad1.right_stick_y) * -1;
                leftPower = Math.abs(gamepad1.right_stick_y) * -1;
                drive = 1;
                break;
            case 4:
                rightPower = Math.abs(gamepad1.right_stick_y);
                leftPower = Math.abs(gamepad1.right_stick_y);
                drive = 1;
                break;

            default:
                leftPower = ((gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y))/speed);
                rightPower = ((gamepad1.right_stick_y * Math.abs(gamepad1.right_stick_y))/speed);
        }

        lDrive1.setPower(leftPower);
        lDrive2.setPower(leftPower);
        rDrive1.setPower(rightPower);
        rDrive2.setPower(rightPower);

    }

//Jims says: Gyro + rive function needs to be super exact
}