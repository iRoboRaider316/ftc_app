package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;


@TeleOp(name="Cross Fire TeleOp", group="TeleOp")

public class CrossfireTeleop extends OpMode{
    ColorSensor color;
    DcMotor rDrive1;
    DcMotor rDrive2;
    DcMotor lDrive1;
    DcMotor lDrive2;
    DcMotor catapult;
    DcMotor sweeper;
    DcMotor lift1;
    DcMotor lift2;
    Servo lButton;
    Servo rButton;
    Servo hopper;
    Servo button;
    Servo belt;
    TouchSensor touch;
    int drive = 0;


    public void init() {
        rDrive1 = hardwareMap.dcMotor.get("rDrive1");
        rDrive2 = hardwareMap.dcMotor.get("rDrive2");
        lDrive1 = hardwareMap.dcMotor.get("lDrive1");
        lDrive2 = hardwareMap.dcMotor.get("lDrive2");
        sweeper = hardwareMap.dcMotor.get("sweeper");
        catapult = hardwareMap.dcMotor.get("catapult");
        lift1 = hardwareMap.dcMotor.get("lift1");
        lift2 = hardwareMap.dcMotor.get("lift2");
        lButton = hardwareMap.servo.get("lButton");
        rButton = hardwareMap.servo.get("rButton");
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
        float rTrigger2 = gamepad2.right_trigger;

        ///OPERATOR CODE\\\

        rButton.setPosition(rTrigger2 * -1);
        lButton.setPosition(lTrigger2);

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

        // This sets the power of the drive motors to based on the joystick position using an Exponential Scale Algorithm
        if (gamepad1.a) { // Backward 100%
            drive = 1;
        }
        if (gamepad1.y) { //Forward 100%
            drive = 2;
        }
        if (gamepad1.right_bumper && drive == 1) { //Backward 1/2 speed
            drive = 3;
        }
        if (gamepad1.right_bumper && drive == 2) { //Forward 1/2 speed
            drive = 4;
        }
        if (gamepad1.dpad_up) { //Forward in a straight line 100% power
            drive = 5;
        }
        if (gamepad1.dpad_down) { // Backward in a straight line 100% power
            drive = 6;
        }
        if (gamepad1.dpad_up && drive == 1) { // Forward in a straight line at 100%
            drive = 7;
        }
        if (gamepad1.dpad_down && drive == 2) { // Backward in a straight line 100%
            drive = 8;
        }
        if (gamepad1.dpad_up && drive == 4) {//Forward 1/2 speed in a straight line
            drive = 9;
        }
        if (gamepad1.dpad_down && drive == 3) { //Backward 1/2 speed in a straight line
            drive = 10;
        }


        switch (drive) {
            case 1:
                rightPower = ((-gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y)));
                leftPower = ((-gamepad1.right_stick_y * Math.abs(gamepad1.right_stick_y)));
                drive = 1;
                break;
            case 2:
                rightPower = (gamepad1.right_stick_y * Math.abs(gamepad1.right_stick_y));
                leftPower = (gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y));
                drive = 2;
                break;
            case 3:
                rightPower = ((-gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y)) / 2);
                leftPower = ((-gamepad1.right_stick_y * Math.abs(gamepad1.right_stick_y)) / 2);
                drive = 3;
                break;
            case 4:
                rightPower = (gamepad1.right_stick_y * Math.abs(gamepad1.right_stick_y)) / 2;
                leftPower = (gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y)) / 2;
                drive = 4;
                break;
            case 5:
                rightPower = 1;
                leftPower = 1;
                drive = 1;
                break;
            case 6:
                rightPower = -1;
                leftPower = -1;
                drive = 1;
                break;
            case 7:
                rightPower = -1;
                leftPower = -1;
                drive = 2;
                break;
            case 8:
                rightPower = 1;
                leftPower = 1;
                drive = 2;
                break;
            case 9:
                rightPower = -1/2;
                leftPower = -1/2;
                drive = 4;
                break;
            case 10:
                    rightPower = -1/2;
                    leftPower = -1/2;
                drive = 3;
                break;
            default:
                leftPower = (gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y));
                rightPower = (gamepad1.right_stick_y * Math.abs(gamepad1.right_stick_y));
        }

        lDrive1.setPower(leftPower);
        lDrive2.setPower(leftPower);
        rDrive1.setPower(rightPower);
        rDrive2.setPower(rightPower);

    }

//Jims says: Gyro + rive function needs to be super exact
}
