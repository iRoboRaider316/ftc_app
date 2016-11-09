package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="vr_teleop", group="Opmode")
//@Disabled

public class vr_teleop extends OpMode {
    DcMotor lDrive1;
    DcMotor lDrive2;
    DcMotor rDrive1;
    DcMotor rDrive2;
    DcMotor catapult;
    DcMotor paddle;
    Servo feeder;
    int drive=0;
    float leftPower;
    float rightPower;
    public void init(){
        lDrive1 = hardwareMap.dcMotor.get("lDrive1");
        lDrive2 = hardwareMap.dcMotor.get("lDrive2");
        rDrive1 = hardwareMap.dcMotor.get("rDrive1");
        rDrive2 = hardwareMap.dcMotor.get("rDrive2");
        lDrive2.setDirection(DcMotor.Direction.REVERSE);
        rDrive1.setDirection(DcMotor.Direction.REVERSE);
        catapult = hardwareMap.dcMotor.get("catapult");
        paddle = hardwareMap.dcMotor.get("paddle");
        feeder = hardwareMap.servo.get("feeder");
    }
    public void loop(){
        // This sets the power of the drive motors to based on the joystick position using an Exponential Scale Algorithm
        if (gamepad1.y) {
            drive = 1;
        }
        if (gamepad1.a) {
            drive = 2;
        }
        lDrive1.setPower(leftPower);
        lDrive2.setPower(leftPower);
        rDrive1.setPower(rightPower);
        rDrive2.setPower(rightPower);
        switch (drive){
            case 1:
                leftPower = (-gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y));
                rightPower = (-gamepad1.right_stick_y * Math.abs(gamepad1.right_stick_y));
                drive = 1;
                break;
            case 2:
                leftPower = (gamepad1.right_stick_y * Math.abs(gamepad1.right_stick_y));
                rightPower = (gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y));
                drive = 2;
                break;
            default:
                leftPower = (-gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y));
                rightPower = (-gamepad1.right_stick_y * Math.abs(gamepad1.right_stick_y));
        }
        // Fires or reverses catapult based on a and y buttons
        if (gamepad2.y) {
            catapult.setPower(.5);
        }
        else if (gamepad2.a) {
            catapult.setPower(-.5);
        }
        else {
            catapult.setPower(0);
        }
        // Turns the paddle on, off, or on in reverse - Operator Gamepad
        if (gamepad2.dpad_up){
            paddle.setPower(-1);
        }
        else if (gamepad2.dpad_down) {
            paddle.setPower(1);
        }
        else if (gamepad2.dpad_left) {
            paddle.setPower(0);
        }
        // Turns the paddle on, off, or on in reverse - Driver gamepad
        if (gamepad1.dpad_up){
            paddle.setPower(-1);
        }
        else if (gamepad1.dpad_down) {
            paddle.setPower(1);
        }
        else if (gamepad1.dpad_left) {
            paddle.setPower(0);
        }
        // Sets the feeder servo position to allow balls to enter the catapult when b is pressed
        if (gamepad2.b) {
            feeder.setPosition(.3);
        }
        else {
            feeder.setPosition(0);
        }
    }
}
