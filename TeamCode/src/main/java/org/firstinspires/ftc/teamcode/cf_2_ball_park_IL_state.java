package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Autonomous(name="cf_2_ball_park", group="LinearOPMode")
@Disabled
public class cf_2_ball_park_IL_state extends LinearOpMode {
    private DcMotor catapult;
    //private DcMotor sweeper;
    private DcMotor lDrive1;
    private DcMotor lDrive2;
    private DcMotor rDrive1;
    private DcMotor rDrive2;
    private DcMotor sweeper;
    private Servo belt;
    private Servo button;
    private Servo hopper;
    private TouchSensor touch;


    // This is the Drive Method
    // It will take in two static values: distance and maxSpeed.
    // It will then calculate the encoder counts to drive and drive the distance at the specified power,
    // accelerating to max speed for the first third of the distance, maintaining that speed for the second third,
    // and decelerating to a minimum speed for the last third.
    // If the robot deviates from the initial gyro heading, it will correct itself proportionally to the error.
    private void encoderDrive(double distance, double maxSpeed, int direction) throws InterruptedException {
        int ENCODER_CPR = 1120; // Encoder counts per Rev
        double gearRatio = 1.75; // [Gear Ratio]:1
        double circumference = 13.10; // Wheel circumference
        double ROTATIONS = distance / (circumference * gearRatio); // Number of rotations to drive
        double COUNTS = ENCODER_CPR * ROTATIONS; // Number of encoder counts to drive

        rDrive1.setTargetPosition(rDrive1.getCurrentPosition() + (int) COUNTS);

        if (direction == 1) {
            while (rDrive1.getCurrentPosition() < rDrive1.getTargetPosition() - 5 && opModeIsActive()) {
                drive(maxSpeed, maxSpeed);
                telemetry.addData("1. speed", maxSpeed);
                updateTelemetry(telemetry);
            }
            driveStop();
        }
        else if (direction == -1) {
            while (Math.abs(rDrive1.getCurrentPosition()) < Math.abs(rDrive1.getTargetPosition() - 5) && opModeIsActive()) {
                drive(-maxSpeed, -maxSpeed);
                telemetry.addData("1. speed", maxSpeed);
                updateTelemetry(telemetry);
            }
            driveStop();
        }
        else {
            telemetry.addLine("Invalid direction");
            telemetry.update();
            sleep(10000);
        }
    }

    private void drive(double leftSpeed, double rightSpeed){
        lDrive1.setPower(leftSpeed);
        lDrive2.setPower(leftSpeed);
        rDrive1.setPower(rightSpeed);
        rDrive2.setPower(rightSpeed);
    }

    private void driveStop(){
        lDrive1.setPower(0);
        lDrive2.setPower(0);
        rDrive1.setPower(0);
        rDrive2.setPower(0);
    }

    // Function that utlizes the launchPosition, handleBall, and launch functions to fire and reload the catapult
    private void fire() throws InterruptedException {
        launchPosition();
        launchBall();
        launchPosition();
        sleep(1000);
        loadBall();
        launchBall();
        launchPosition();
    }
    // Resets catapult to the launch position
    private void launchPosition() throws InterruptedException{
        while (!touch.isPressed()){
            catapult.setPower(0.5);
        }
        catapult.setPower(0);
    }
    // Function to load the catapult
    private void loadBall() throws InterruptedException {
        hopper.setPosition(.5);
        sleep(1000);
        hopper.setPosition(.8);
    }
    // Fires the ball
    private void launchBall() throws InterruptedException {
        catapult.setPower(1);
        sleep(800);
        catapult.setPower(0);
    }

    public void runOpMode() throws InterruptedException {
        rDrive1 = hardwareMap.dcMotor.get("rDrive1");
        rDrive2 = hardwareMap.dcMotor.get("rDrive2");
        lDrive1 = hardwareMap.dcMotor.get("lDrive1");
        lDrive2 = hardwareMap.dcMotor.get("lDrive2");
        belt = hardwareMap.servo.get("belt");
        lDrive1.setDirection(DcMotor.Direction.REVERSE);
        lDrive2.setDirection(DcMotor.Direction.REVERSE);
        sweeper = hardwareMap.dcMotor.get("sweeper");
        catapult = hardwareMap.dcMotor.get("catapult");
        hopper = hardwareMap.servo.get("hopper");
        touch = hardwareMap.touchSensor.get("t");
        button = hardwareMap.servo.get("button");
        hopper.setPosition(0.8);
        button.setPosition(0.5);
        belt.setPosition(.5);

        double distance;
        double maxSpeed;
        int direction;

        waitForStart();

        //sleep(10000);
        distance = 25;
        maxSpeed = .5;
        direction = 1;
        encoderDrive(distance, maxSpeed, direction);
        sleep(1000);
        sweeper.setPower(1);
        sleep(500);
        sweeper.setPower(-1);
        fire();
        sleep(5000);
        loadBall();
        fire();
        sweeper.setPower(0);
        distance = 40;
        maxSpeed = 1;
        direction = 1;
        encoderDrive(distance, maxSpeed, direction);
        sleep(5000);

    }
}