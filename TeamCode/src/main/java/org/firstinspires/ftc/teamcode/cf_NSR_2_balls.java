package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="cf_NSR_2_balls", group="LinearOPMode")

public class cf_NSR_2_balls extends LinearOpMode {
    ElapsedTime timer = new ElapsedTime();
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
    private Servo wheels;
    private TouchSensor touch;
    GyroSensor gyroSensor;
    ModernRoboticsI2cGyro gyro;

    private void useEncoders() {
        rDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (lDrive1.isBusy()) {
            sleep(10);
        }
    }

    private void resetEncoders() {
        lDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while (lDrive1.isBusy()) {
            sleep(10);
        }
    }

    private void encoderDrive(double distance, double leftSpeed, double rightSpeed, int direction) throws InterruptedException {
        int ENCODER_CPR = 1120; // Encoder counts per Rev
        double gearRatio = 1.75; // [Gear Ratio]:1
        double circumference = 13.10; // Wheel circumference
        double ROTATIONS = distance / (circumference * gearRatio); // Number of rotations to drive
        double COUNTS = ENCODER_CPR * ROTATIONS; // Number of encoder counts to drive
        if (direction == 1)
            rDrive1.setTargetPosition(rDrive1.getCurrentPosition() + (int) COUNTS);
        else if (direction == -1)
            rDrive1.setTargetPosition(rDrive1.getCurrentPosition() - (int) COUNTS);
        if (direction == 1) {
            while (rDrive1.getCurrentPosition() < rDrive1.getTargetPosition() - 5 && opModeIsActive()) {
                drive(leftSpeed, rightSpeed);
                telemetry.addData("1. left speed", leftSpeed);
                telemetry.addData("2. right speed", rightSpeed);
                updateTelemetry(telemetry);
            }
            driveStop();
        } else if (direction == -1) {
            while (rDrive1.getCurrentPosition() > rDrive1.getTargetPosition() + 5 && opModeIsActive()) {
                drive(-leftSpeed, -rightSpeed);
                telemetry.addData("1. left speed", leftSpeed);
                telemetry.addData("2. right speed", rightSpeed);
                updateTelemetry(telemetry);
            }
            driveStop();
        } else {
            telemetry.addLine("Invalid direction");
            telemetry.update();
            sleep(10000);
        }
    }

    private void drive(double leftSpeed, double rightSpeed) {
        lDrive1.setPower(leftSpeed);
        lDrive2.setPower(leftSpeed);
        rDrive1.setPower(rightSpeed);
        rDrive2.setPower(rightSpeed);
    }

    private void driveStop() {
        drive(0,0);
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
    private void launchPosition() throws InterruptedException {
        while (!touch.isPressed()) {
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

    private void gyroTurn(int targetHeading) {
        boolean done = false;
        double error;
        double currentHeading;
        double kp = .0035;
        double power;
        gyro.resetZAxisIntegrator();
        sleep(250);

        while (!done && opModeIsActive()) {
            currentHeading = -gyro.getIntegratedZValue();

            error = (targetHeading - currentHeading);
            power = error * kp;

            error = Range.clip(error, -.3, .3);
            if (power > 0 && power < .18)
                power = .18;
            else if (power < 0 && power > -.18)
                power = -.18;

            telemetry.addData("error", error);
            telemetry.addData("power", power);
            telemetry.addData("currentHeading", currentHeading);
            telemetry.addData("targetHeading", targetHeading);
            telemetry.update();

            if (currentHeading <= targetHeading + 1 && currentHeading >= targetHeading - 1) {
                done = true;
                driveStop();
            } else {
                done = false;
                drive(0 + power, 0 - power);
            }

        }
        driveStop();
    }

    private void setUpGyro() throws InterruptedException {

        // setup the Gyro
        // write some device information (connection info, name and type)
        // to the log file.
        hardwareMap.logDevices();
        // get a reference to our GyroSensor object.
        gyroSensor = hardwareMap.gyroSensor.get("gyro");
        gyro = (ModernRoboticsI2cGyro) gyroSensor;
        // calibrate the gyro.
        gyroSensor.calibrate();
        while (gyroSensor.isCalibrating()) {
            sleep(50);
        }
        // End of setting up Gyro
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
        wheels = hardwareMap.servo.get("wheels");
        hopper.setPosition(0.8);
        button.setPosition(0.5);
        belt.setPosition(.5);
        wheels.setPosition(.2);
        double distance;
        double leftSpeed;
        double rightSpeed;
        int direction;
        boolean far = false;
        boolean farCenter = false;
        boolean nearCenter = false;
        boolean near = false;
        boolean nearCorner = false;
        boolean farCorner = false;
        boolean red = false;
        boolean blue = false;
        boolean nothing = false;
        int wait = 0;

        setUpGyro();
        resetEncoders();
        idle();
        useEncoders();
        idle();

        while (!red && !blue){
            telemetry.addLine("Press B for red alliance");
            telemetry.addLine("Press X for blue alliance");
            telemetry.addLine("Press dpad_right to turn vortex");
            telemetry.update();
            if (gamepad1.x)
                blue = true;
            else if (gamepad1.b)
                red = true;
        }
        sleep(1000);

        while (!near && !far){
            telemetry.addLine("Press dpad_up to start far corner");
            telemetry.addLine("Press dpad_down to start near ramp");
            telemetry.update();
            if (gamepad1.dpad_up)
                far = true;
            else if (gamepad1.dpad_down)
                near = true;
        }
        sleep(1000);
        if (near) {
            while (!nearCenter && !nearCorner && !nothing) {
                telemetry.addLine("Press dpad_up to park center");
                telemetry.addLine("Press dpad_down to park ramp");
                telemetry.update();

                telemetry.update();
                if (gamepad1.dpad_up)
                    nearCenter = true;
                else if (gamepad1.dpad_down)
                    nearCorner = true;
                else if (gamepad1.dpad_left);
                nothing = true;
            }
        }
        else if (far){
            while (!farCenter && !farCorner && !nothing) {
                telemetry.addLine("Press dpad_up to park center");
                telemetry.addLine("Press dpad_down to park ramp");
                telemetry.update();

                telemetry.update();
                if (gamepad1.dpad_up)
                    farCenter = true;
                else if (gamepad1.dpad_down)
                    farCorner = true;
                else if (gamepad1.dpad_left);
                nothing = true;


            }
        }

        while (!isStarted()){
            if (blue)
                telemetry.addLine("Blue alliance");
            if (red)
                telemetry.addLine("Red alliance");
            if (near)
                telemetry.addLine("Near start");
            if (far)
                telemetry.addLine("far start");
            if (nearCenter)
                telemetry.addLine("Center End");
            if (nearCorner)
                telemetry.addLine("Corner End");
            if (farCenter)
                telemetry.addLine("Center End");
            if (farCorner)
                telemetry.addLine("Corner End");
            if (nothing)
                telemetry.addLine("No End");
            if (gamepad1.y)
                wait = wait + 1;
            sleep(100);
            telemetry.update();
        }

        waitForStart();
        wait = wait * 1000;
        timer.reset();

        if (red) {
            // Drive forward from wall
            if (far) {

                encoderDrive(/*distance*/25, /*leftSpeed*/0.5, /*rightSpeed*/0.5, /*direction*/1);
                sleep(1000);
            }
            if (near) {
                encoderDrive(/*distance*/4, /*leftSpeed*/0.5, /*rightSpeed*/0.5, /*direction*/1);
                sleep(1000);
            }
//            // deploy sweeper
//            sweeper.setPower(1);
//            sleep(500);
//            sweeper.setPower(-1);
//            // fire balls
//            fire();
//            sleep(5000);
//            // load any balls that have been picked up
//            loadBall();
//            // fire balls
            fire();
            // stop sweeper
//            sweeper.setPower(0);
            // drive forward to knock off cap ball
            sleep(wait);

                if (farCenter) {

                    encoderDrive(/*distance*/40, /*leftSpeed*/1, /*rightSpeed*/1, /*direction*/1);
                }
            if (farCorner) {
                gyroTurn(-45);

                encoderDrive(/*distance*/50, /*leftSpeed*/1, /*rightSpeed*/1, /*direction*/1);
                gyroTurn(-45);
                encoderDrive(/*distance*/20, /*leftSpeed*/1, /*rightSpeed*/1, /*direction*/1);
            }

            if (nearCenter) {

                encoderDrive(/*distance*/40, /*leftSpeed*/0.8, /*rightSpeed*/0.8, /*direction*/1);
            }
            if (nearCorner) {
                gyroTurn(-70);

                encoderDrive(/*distance*/20, /*leftSpeed*/0.8, /*rightSpeed*/1, /*direction*/1);
            }

        }

        if (blue) {
            // Drive forward from wall
            if (far) {

                encoderDrive(/*distance*/25, /*leftSpeed*/0.5, /*rightSpeed*/0.5, /*direction*/1);
                sleep(1000);
            }
            if (near) {
                encoderDrive(/*distance*/4, /*leftSpeed*/0.5, /*rightSpeed*/0.5, /*direction*/1);
                sleep(1000);
            }
            // deploy sweeper
//            sweeper.setPower(1);
//            sleep(500);
//            sweeper.setPower(-1);
//            // fire balls
            fire();
//            sleep(5000);
//            // load any balls that have been picked up
//            loadBall();
//            // fire balls
//            fire();
            // stop sweeper
            //sweeper.setPower(0);
            // drive forward to knock off cap ball
            sleep(wait);

                if (farCenter) {

                    encoderDrive(/*distance*/40, /*leftSpeed*/1, /*rightSpeed*/1, /*direction*/1);
                }
            if (farCorner) {
                gyroTurn(45);

                encoderDrive(/*distance*/50, /*leftSpeed*/1, /*rightSpeed*/1, /*direction*/1);
                gyroTurn(45);
                encoderDrive(/*distance*/20, /*leftSpeed*/1, /*rightSpeed*/1, /*direction*/1);
            }

            if (nearCenter) {

                encoderDrive(/*distance*/40, /*leftSpeed*/0.8, /*rightSpeed*/0.8, /*direction*/1);
            }
            if (nearCorner) {
                gyroTurn(70);

                encoderDrive(/*distance*/20, /*leftSpeed*/1, /*rightSpeed*/0.8, /*direction*/1);
            }

        }
    }
}