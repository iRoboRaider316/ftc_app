package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name="CrossFire TeleOp", group="TeleOp")

public class crossFireTeleOp extends OpMode{
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
    int direction = 0;
    int power = 1;


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
        float speed;

        ///OPERATOR CODE\\\

        if (up) {
            sweeper.setPower(-1);
        } else if (down) {
            sweeper.setPower(1);
        } else if (left) {
            sweeper.setPower(0);
        }
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

        // Universal drive train power switch case
        if (gamepad1.right_bumper && power == 1)
            // cut to half speed
            power = 2;
        else if (gamepad1.right_bumper && power == 2)
            // set to full speed
            power = 1;

        switch (power){
            case 1:
                // divide all powers by 1 (full speed)
                speed = 1;
                power = 1;
                break;
            case 2:
                // divide all powers by 2 (half speed
                speed = 2;
                power = 2;
                break;
            default:
                speed = 1;
                break;
        }

        if (gamepad1.y)
            // robot sweeper is forward
            direction = 1;
        else if (gamepad1.a)
            // robot cap ball lift is forward
            direction = 2;

        switch (direction) {
            case 1:
                // drive straight forward
                if (gamepad1.dpad_up){
                    rightPower = -1/speed;
                    leftPower = -1/speed;
                }
                // drive straight backward
                else if (gamepad1.dpad_down){
                    rightPower = 1/speed;
                    leftPower = 1/speed;
                }
                // standard driving
                else {
                    leftPower = ((gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y))/speed);
                    rightPower = ((gamepad1.right_stick_y * Math.abs(gamepad1.right_stick_y))/speed);
                }
                direction = 1;
                break;
            case 2:
                // drive straight backward
                if (gamepad1.dpad_up){
                    rightPower = 1/speed;
                    leftPower = 1/speed;
                }
                // drive straight forward
                else if (gamepad1.dpad_down){
                    rightPower = -1/speed;
                    leftPower = -1/speed;
                }
                // backward driving
                else {
                    leftPower = ((-gamepad1.right_stick_y * Math.abs(gamepad1.right_stick_y))/speed);
                    rightPower = ((-gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y))/speed);
                }
                direction = 2;
                break;
            default:
                // standard driving
                leftPower = ((gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y))/speed);
                rightPower = ((gamepad1.right_stick_y * Math.abs(gamepad1.right_stick_y))/speed);
                break;
        }

        lDrive1.setPower(leftPower);
        lDrive2.setPower(leftPower);
        rDrive1.setPower(rightPower);
        rDrive2.setPower(rightPower);

    }
}