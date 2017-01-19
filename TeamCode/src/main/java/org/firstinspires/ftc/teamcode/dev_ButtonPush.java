package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;


@Autonomous(name="ButtonPush",group="LinearOpMode")

public class dev_ButtonPush extends LinearOpMode {

    DcMotor sweeper;
    Servo Hand;
    Servo hopper;
    DcMotor catapult;
    DcMotor lDrive1;
    DcMotor lDrive2;
    DcMotor rDrive1;
    DcMotor rDrive2;
    ColorSensor color;
    GyroSensor gyroSensor;
    TouchSensor touch;
    OpticalDistanceSensor rODSensor;
    OpticalDistanceSensor lODSensor;
    ModernRoboticsI2cGyro gyro;

    public void moveMotors(double left, double right, long time) throws InterruptedException {
        rDrive1.setPower(right);        // moves forward at certain power...
        rDrive2.setPower(right);
        lDrive1.setPower(left);
        lDrive2.setPower(left);
        sleep(time);               // ...until it's been running for a certain time.
        rDrive1.setPower(0);            // at that point, the robot stops...
        rDrive2.setPower(0);
        lDrive1.setPower(0);
        lDrive2.setPower(0);
        sleep(500);                     // ...and waits a half second.
    }

    public void ButtonPush() throws InterruptedException {
        color.enableLed(false);
        if(color.red() > color.blue()) {
            Hand.setPosition(1);
            sleep(2000);
            Hand.setPosition(0);
            sleep(2000);
        } else if(color.blue() > color.red()) {
            moveMotors(0.4, 0.4, 500);
            Hand.setPosition(1);
            sleep(2000);
            Hand.setPosition(0);
            sleep(2000);
        }
        telemetry.addData("RedValue:", color.red());
        telemetry.addData("GreenValue:", color.green());
        telemetry.addData("BlueValue:", color.blue());
        telemetry.update();
        sleep(3000);

    }

    public void runOpMode() throws InterruptedException {
        rDrive1 = hardwareMap.dcMotor.get("rDrive1");
        rDrive2 = hardwareMap.dcMotor.get("rDrive2");
        lDrive1 = hardwareMap.dcMotor.get("lDrive1");
        lDrive2 = hardwareMap.dcMotor.get("lDrive2");
        sweeper = hardwareMap.dcMotor.get("sweeper");
        catapult = hardwareMap.dcMotor.get("catapult");
        Hand = hardwareMap.servo.get("button");
        hopper = hardwareMap.servo.get("hopper");
        touch = hardwareMap.touchSensor.get("t");
        color = hardwareMap.colorSensor.get("color");
        rODSensor = hardwareMap.opticalDistanceSensor.get("rOD");
        lODSensor = hardwareMap.opticalDistanceSensor.get("lOD");
        lDrive2.setDirection(DcMotor.Direction.REVERSE);
        lDrive1.setDirection(DcMotor.Direction.REVERSE);
        lDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();
        lDrive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rDrive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        ButtonPush();
    }
}

