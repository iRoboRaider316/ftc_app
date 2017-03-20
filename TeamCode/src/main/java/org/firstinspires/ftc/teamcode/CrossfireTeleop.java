package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="CrossFire TeleOp", group="TeleOp")

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
    Servo hopper;
    Servo button;
    Servo belt;
    Servo wheels;
    TouchSensor touch;
    int direction = 0;
    int power = 1;
    int sideWheels = 1;
    int drive = 0;
    boolean wheelsDown = false;
    boolean wheelsUp = false;
    boolean halfSpeed = false;
    boolean fullSpeed = false;

    public double getDirection(double inputPower) {
        if(inputPower == 0) {
            return 1;
        } else {
            return inputPower * Math.abs(inputPower);
        }
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
        wheels = hardwareMap.servo.get("wheels");
        rDrive1.setDirection(DcMotor.Direction.REVERSE);
        rDrive2.setDirection(DcMotor.Direction.REVERSE);

        belt.setPosition(.5);
        button.setPosition(.5);
        hopper.setPosition(.8);
        wheels.setPosition(.2);

    }


    public void loop() {
        belt.setPosition(.5);
        float lStick1 = gamepad1.left_stick_y;
        float rStick1 = gamepad1.right_stick_y;
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
        double leftPower;
        double rightPower;
        float lTrigger2 = gamepad2.left_trigger;
        float speed;
        double floorLeft;
        double floorRight;
        double lDriveDirection;
        double rDriveDirection;

        ///OPERATOR CODE\\\
//================================================
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
//================================================
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

        // Switch case to control wheel position
        if (gamepad1.left_bumper && sideWheels == 1){
            wheelsDown = true;
        }
        else if (wheelsDown && !gamepad1.left_bumper){
            sideWheels = 2;
            wheelsDown = false;
        }
        else if (gamepad1.left_bumper && sideWheels == 2){
            wheelsUp = true;
        }
        else if (wheelsUp && !gamepad1.left_bumper){
            sideWheels = 1;
            wheelsUp = false;
        }

        switch (sideWheels){
            case 1:
                wheels.setPosition(.2);
                sideWheels = 1;
                break;
            case 2:
                wheels.setPosition(.85);
                sideWheels = 2;
                break;
            default:
                wheels.setPosition(.2);
                break;
        }

        // Universal drive train power switch case
        if (gamepad1.right_bumper && power == 1){
            halfSpeed = true;
        }
        else if (halfSpeed && !gamepad1.right_bumper){
            power = 2;
            halfSpeed = false;
        }
        else if (gamepad1.right_bumper && power == 2){
            fullSpeed = true;
        }
        else if (fullSpeed && !gamepad1.right_bumper){
            power = 1;
            fullSpeed = false;
        }

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

        // This is the floor-ceiling algorithm we use to keep Crossfire moving without wearing too
        // much battery power
        floorLeft = lStick1 <= 0.01 && lStick1 >= -0.01 ? 0 : 0.13 * getDirection(lStick1);
        floorRight = rStick1 <= 0.01 && rStick1 >= -0.01 ? 0 : 0.13 * getDirection(rStick1);

        // This is the switch case that sets the motor powers of Crossfire. Floor-Ceiling algorithms
        // have been included for greater control
        switch (drive) {
            case 1:
                // forward default
                rightPower = ((-lStick1 * Math.abs(lStick1))/speed) + floorLeft;
                leftPower = ((-rStick1 * Math.abs(rStick1))/speed) + floorRight;
                drive = 1;
                break;
            case 2:
                // backward
                rightPower = ((rStick1 * Math.abs(rStick1))/speed) + floorRight;
                leftPower = ((lStick1 * Math.abs(lStick1))/speed) + floorLeft;
                drive = 2;
                break;
            case 3:
                rightPower = (Math.abs(rStick1) * -1) + floorRight;
                leftPower = (Math.abs(rStick1) * -1) + floorRight;
                drive = 1;
                break;
            case 4:
                rightPower = Math.abs(rStick1) + floorRight;
                leftPower = Math.abs(rStick1) + floorRight;
                drive = 1;
                break;

            default:
                leftPower = ((lStick1 * Math.abs(lStick1))/speed) + floorLeft;
                rightPower = ((rStick1 * Math.abs(rStick1))/speed) + floorRight;
        }

        // Clips off the power in case it's over 1 or under -1 from the floor-ceiling function.
        leftPower = Range.clip(leftPower, -1, 1);
        rightPower = Range.clip(rightPower, -1, 1);

        lDrive1.setPower(leftPower);
        lDrive2.setPower(leftPower);
        rDrive1.setPower(rightPower);
        rDrive2.setPower(rightPower);

    }
}