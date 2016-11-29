package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Autonomous(name="cf_auto_red", group="LinearOpMode")


public class cf_auto_red extends LinearOpMode {
    ColorSensor color;
    Servo hopper;
    DcMotor catapult;
    DcMotor sweeper;
    DcMotor lDrive1;
    DcMotor lDrive2;
    DcMotor rDrive1;
    DcMotor rDrive2;
    Servo lButton;
    Servo rButton;
    TouchSensor touch;

    public void launchPosition() throws InterruptedException{

       while (!touch.isPressed()){
         catapult.setPower(0.5);
        }
        catapult.setPower(0);

        }

    public void fire() throws InterruptedException {
        launch();
        launchPosition();
        handleBall();
        launch();
        launchPosition();
        handleBall();
        launch();

    }

    public void handleBall() throws InterruptedException {
        hopper.setPosition(1);
        sleep(1000);
        hopper.setPosition(1000);

    }

    public void launch() throws InterruptedException {
        catapult.setPower(1);
        sleep(800);
        catapult.setPower(0);

    }


    public void runOpMode() throws InterruptedException {
        //##############Init##############
        rDrive1 = hardwareMap.dcMotor.get("rDrive1");
        rDrive2 = hardwareMap.dcMotor.get("rDrive2");
        lDrive1 = hardwareMap.dcMotor.get("lDrive1");
        lDrive2 = hardwareMap.dcMotor.get("lDrive2");
        sweeper = hardwareMap.dcMotor.get("sweeper");
        catapult = hardwareMap.dcMotor.get("catapult");
        lButton = hardwareMap.servo.get("lButton");
        rButton = hardwareMap.servo.get("rButton");
        hopper = hardwareMap.servo.get("hopper");
        touch = hardwareMap.touchSensor.get("t");
        color = hardwareMap.colorSensor.get("color");


        waitForStart();
        fire();
    }
}
