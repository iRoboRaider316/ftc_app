package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Cross Fire TeleOp", group="TeleOp")

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
    Servo wheels;
    int direction = 0;
    int power = 1;
    int sideWheels = 1;
    boolean wheelsDown = false;
    boolean wheelsUp = false;
    boolean halfSpeed = false;
    boolean fullSpeed = false;
    double floorLeft;
    double floorRight;

    public double getDirection(double inputPower) {
        if(inputPower == 0) {
            return 1;
        } else {
            return inputPower * Math.abs(inputPower);
        }
    }
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
        wheels = hardwareMap.servo.get("wheels");
        rDrive1.setDirection(DcMotor.Direction.REVERSE);
        rDrive2.setDirection(DcMotor.Direction.REVERSE);

        belt.setPosition(.5);
        button.setPosition(.5);
        hopper.setPosition(.8);


    }

    private LaunchState launchState = LaunchState.Idle;

    public void loop() {
        belt.setPosition(.5);
        float rStick1 = gamepad1.right_stick_y;
        float lStick1 = gamepad1.left_stick_y;
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
        double lTrigger2 = gamepad2.left_trigger;
        double speed = 1;

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
                if(timer.milliseconds() > 100) {
                    hopper.setPosition(0.5);
                    timer.reset();
                    launchState = LaunchState.Open;
                }
                break;

            case Open:
                if(timer.milliseconds() > 200) {
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

        if (lBumper1 && sideWheels == 1){
            wheelsDown = true;
        }
        else if (wheelsDown && !lBumper1){
            sideWheels = 2;
            wheelsDown = false;
        }
        else if (lBumper1 && sideWheels == 2){
            wheelsUp = true;
        }
        else if (wheelsUp && !lBumper1){
            sideWheels = 1;
            wheelsUp = false;
        }

        switch (sideWheels){
            case 1:
                wheels.setPosition(.62);
                sideWheels = 1;
                break;
            case 2:
                wheels.setPosition(1);
                sideWheels = 2;
                break;
            default:
                wheels.setPosition(.62);
                break;
        }
        // Universal drive train power switch case
        if (rBumper1 && power == 1){
            halfSpeed = true;
        }
        else if (halfSpeed && !rBumper1){
            power = 2;
            halfSpeed = false;
        }
        else if (rBumper1 && power == 2){
            fullSpeed = true;
        }
        else if (fullSpeed && !rBumper1){
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

        // This is the floor-ceiling algorithm we use to keep Crossfire moving without wearing too
        // much battery power
        floorLeft = lStick1 <= 0.01 && lStick1 >= -0.01 ? 0 : 0.13 * getDirection(lStick1);
        floorRight = rStick1 <= 0.01 && rStick1 >= -0.01 ? 0 : 0.13 * getDirection(rStick1);

        // This is the switch case that sets the motor powers of Crossfire. Floor-Ceiling algorithms
        // have been included for greater control

        switch (drive) {
            case 1:
                // forward default
                rightPower = ((-lStick1 * Math.abs(lStick1))/speed) - floorLeft;
                leftPower = ((-rStick1 * Math.abs(rStick1))/speed) - floorRight;
                drive = 1;
                break;
            case 2:
                // backward
                rightPower = ((rStick1 * Math.abs(rStick1))/speed) + floorRight;
                leftPower = ((lStick1 * Math.abs(lStick1))/speed) + floorLeft;
                drive = 2;
                break;
            case 3:
                rightPower = (Math.abs(rStick1) * -1) - floorRight;
                leftPower = (Math.abs(rStick1) * -1) - floorRight;
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