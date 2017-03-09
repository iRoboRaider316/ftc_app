package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="refactoring_red", group="LinearOpMode")
//@Disabled

public class refactoring_red extends LinearOpMode {

    private DcMotor catapult;
    private DcMotor lDrive1;
    private DcMotor lDrive2;
    private DcMotor rDrive1;
    private DcMotor rDrive2;
    private Servo button;
    private Servo hopper;
    private Servo belt;
    private ModernRoboticsI2cRangeSensor fRangeSensor;
    private ModernRoboticsI2cRangeSensor bRangeSensor;

    private TouchSensor touch;
    private ColorSensor color;
    private OpticalDistanceSensor fODSensor;
    private OpticalDistanceSensor bODSensor;



    // Function that utlizes the launchPosition, handleBall, and launch functions to fire and reload the catapult
    private void fire() throws InterruptedException {
        sleep(1000);
        launchPosition();
        launchBall();
        launchPosition();
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

    private void driveForwardToWall() throws InterruptedException {
        double fdist = fRangeSensor.getDistance(DistanceUnit.INCH);
        lDrive1.setPower(0.4);
        rDrive1.setPower(0.4);
        lDrive2.setPower(0.4);
        rDrive2.setPower(0.4);
        //sleep(2000);
        while (fdist >= 12 && opModeIsActive()) {
//            range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
            fdist = fRangeSensor.getDistance(DistanceUnit.INCH);
//            telemetry.addData("Range value:", (range1Cache[0] & 0xFF));
            telemetry.addData("RangeF value:", fdist);
        }
        lDrive1.setPower(0);
        rDrive1.setPower(0);
        lDrive2.setPower(0);
        rDrive2.setPower(0);
    }
    //    public void driveForwardAlongWall() throws InterruptedException {
//        // Tolerance/Gain for Wall Tracking
//        double tolerance = 0.25;
//        // start small and increase slowly to eliminate oscillation
//        double gain = 0.025;
//
//        while (bODSensor.getRawLightDetected() < 0.75 && opModeIsActive()) {
//            double error = fRangeSensor.getDistance(DistanceUnit.INCH) - 4;
//            double leftSpeed = 0.5;
//            double rightSpeed = 0.5;
//            if (error < tolerance) { // too close
//                leftSpeed = gain*error;
//                rightSpeed = -gain*error;
//            }
//            else if (error > tolerance) { // too far
//                leftSpeed = -gain*error;
//                rightSpeed = gain*error;
//            }
//            leftSpeed = Range.clip(leftSpeed, -0.6, 0.6);
//            rightSpeed = Range.clip(rightSpeed, -0.6, 0.6);
//            driveTurn(leftSpeed, rightSpeed);
//        }
//    }
    private void driveBackwardToWall() throws InterruptedException {
        double bdist = bRangeSensor.getDistance(DistanceUnit.INCH);
        lDrive1.setPower(-0.4);
        rDrive1.setPower(-0.4);
        lDrive2.setPower(-0.4);
        rDrive2.setPower(-0.4);
        //sleep(2000);

        while (bdist >= 4.8 && opModeIsActive()) {
           bdist = bRangeSensor.getDistance(DistanceUnit.INCH);
            telemetry.addData("RangeF value:", bdist);
            telemetry.update();
        }
        lDrive1.setPower(0);
        rDrive1.setPower(0);
        lDrive2.setPower(0);
        rDrive2.setPower(0);
    }

        public void driveForwardAlongWall() throws InterruptedException {
        // Tolerance/Gain for Wall Tracking
        double tolerance = 0.25;
        // start small and increase slowly to eliminate oscillation
        double gain = 0.025;

        while (bODSensor.getRawLightDetected() < 0.75 && opModeIsActive()) {
            double error = fRangeSensor.getDistance(DistanceUnit.INCH) - 4;
            double leftSpeed = 0.5;
            double rightSpeed = 0.5;
            if (error < tolerance) { // too close
                leftSpeed = gain*error;
                rightSpeed = -gain*error;
            }
            else if (error > tolerance) { // too far
                leftSpeed = -gain*error;
                rightSpeed = gain*error;
            }
            leftSpeed = Range.clip(leftSpeed, -0.6, 0.6);
            rightSpeed = Range.clip(rightSpeed, -0.6, 0.6);
            //driveTurn(leftSpeed, rightSpeed);
            lDrive1.setPower((leftSpeed));
            rDrive1.setPower((rightSpeed));
            lDrive2.setPower((leftSpeed));
            rDrive2.setPower((rightSpeed));
        }
    }
    public void driveBackwardAlongWall() throws InterruptedException {
        // Tolerance/Gain for Wall Tracking
        double tolerance = 0.25;
        // start small and increase slowly to eliminate oscillation
        double gain = 0.025;

        while (bODSensor.getRawLightDetected() < 0.75 && opModeIsActive()) {
            double error = bRangeSensor.getDistance(DistanceUnit.INCH) - 4;
            double leftSpeed = 0.5;
            double rightSpeed = 0.5;
            if (error < tolerance) { // too close
                leftSpeed = gain*error;
                rightSpeed = -gain*error;
            }
            else if (error > tolerance) { // too far
                leftSpeed = -gain*error;
                rightSpeed = gain*error;
            }
            leftSpeed = Range.clip(leftSpeed, -0.6, 0.6);
            rightSpeed = Range.clip(rightSpeed, -0.6, 0.6);
            //driveTurn((leftSpeed) *-1 , (rightSpeed) * -1);
            lDrive1.setPower((leftSpeed) *-1);
            rDrive1.setPower((rightSpeed) * -1);
            lDrive2.setPower((leftSpeed) *-1);
            rDrive2.setPower((rightSpeed) * -1);
        }
    }
    private void lineUp() throws InterruptedException {

        double error = 0;
        boolean done = false;
        double rangeb;
        double rangef;
        while (!done && opModeIsActive()) {

             rangeb = bRangeSensor.getDistance(DistanceUnit.INCH);
             rangef = fRangeSensor.getDistance(DistanceUnit.INCH);
            telemetry.addData("Range value:", rangef);
            telemetry.addData("Range2 value:", rangeb);

            error = (rangef - rangeb)/40;
            telemetry.addData("error", error);

            lDrive1.setPower(0+error);
            lDrive2.setPower(0+error);
            rDrive1.setPower(0-error);
            rDrive2.setPower(0-error);
            telemetry.update();
            if (rangef > 100 || rangeb > 100) //If the range sensors see a 255
                done = false;
            else {
                if (rangef >= rangeb-1 && rangef <= rangeb+1)
                    done = true;
                else
                    done = false;
            }
        }
        lDrive1.setPower(0);
        rDrive1.setPower(0);
        lDrive2.setPower(0);
        rDrive2.setPower(0);
    }
    // Fully working as of 2/8/17
    private void recognizeColorRed(int direction) throws InterruptedException {
        color.enableLed(false);
        while (color.red() < (color.blue())+1) {
            rDrive1.setPower(.2*direction);
            rDrive2.setPower(.2*direction);
            lDrive1.setPower(.2*direction);
            lDrive2.setPower(.2*direction);
            telemetry.addData("Blue", color.blue());
            telemetry.addData("Red", color.red());
            telemetry.update();
        }
        rDrive1.setPower(0);
        rDrive2.setPower(0);
        lDrive1.setPower(0);
        lDrive2.setPower(0);
        button.setPosition(0);
        sleep(2000);
        button.setPosition(1);
        sleep(2000);
        button.setPosition(0.5);
        telemetry.addData("Blue", color.blue());
        telemetry.addData("Red", color.red());
        telemetry.update();


    }

    // Fully working as of 2/8/17
    private void recognizeColorBlue(int direction) throws InterruptedException {
        color.enableLed(false);
        while (color.blue() < (color.red()+1)) {
            rDrive1.setPower(.2*direction);
            rDrive2.setPower(.2*direction);
            lDrive1.setPower(.2*direction);
            lDrive2.setPower(.2*direction);
            telemetry.addData("Blue", color.blue());
            telemetry.addData("Red", color.red());
            telemetry.update();
        }
        rDrive1.setPower(0);
        rDrive2.setPower(0);
        lDrive1.setPower(0);
        lDrive2.setPower(0);
        button.setPosition(0);
        sleep(2000);
        button.setPosition(1);
        sleep(2000);
        button.setPosition(0.5);
        telemetry.addData("Blue", color.blue());
        telemetry.addData("Red", color.red());
        telemetry.update();
    }

    private void wrongBlue() {
        sleep(500);

        if ((color.blue() < (color.red()+1))) {
            sleep(5000);
            button.setPosition(0);
            sleep(2000);
            button.setPosition(1);
            sleep(2000);
            button.setPosition(0.5);
        }

    }

    private void wrongRed() {
        sleep(500);

        if ((color.blue() > (color.red()+1))) {
            sleep(5000);
            button.setPosition(0);
            sleep(2000);
            button.setPosition(1);
            sleep(2000);
            button.setPosition(0.5);
        }
    }

    // This is the Drive Method
    // It will take in two static values: distance and maxSpeed.
    // It will then calculate the encoder counts to drive and drive the distance at the specified power,
    // accelerating to max speed for the first third of the distance, maintaining that speed for the second third,
    // and decelerating to a minimum speed for the last third.
    // If the robot deviates from the initial gyro heading, it will correct itself proportionally to the error.
    private void drive(double distance, double maxSpeed, int direction) throws InterruptedException {
        int ENCODER_CPR = 1120; // Encoder counts per Rev
        double gearRatio = 1.75; // [Gear Ratio]:1
        double circumference = 13.10; // Wheel circumference
        double ROTATIONS = distance / (circumference * gearRatio); // Number of rotations to drive
        double COUNTS = ENCODER_CPR * ROTATIONS; // Number of encoder counts to drive
        double speed = 0;
        double leftSpeed;
        double rightSpeed;

        rDrive1.setTargetPosition(rDrive1.getCurrentPosition() + (int) COUNTS);

        if (direction == 1) {
            while (rDrive1.getCurrentPosition() < (rDrive1.getTargetPosition() - 5) && opModeIsActive()) {

                leftSpeed = maxSpeed;
                rightSpeed = maxSpeed;

                leftSpeed = Range.clip(leftSpeed, -1, 1);
                rightSpeed = Range.clip(rightSpeed, -1, 1);

                lDrive1.setPower(leftSpeed);
                rDrive1.setPower(rightSpeed);
                lDrive2.setPower(leftSpeed);
                rDrive2.setPower(rightSpeed);

                telemetry.addData("1. speed", speed);
                telemetry.addData("2. leftSpeed", leftSpeed);
                telemetry.addData("3. rightSpeed", rightSpeed);
                updateTelemetry(telemetry);
            }

            lDrive1.setPower(0);
            rDrive1.setPower(0);
            lDrive2.setPower(0);
            rDrive2.setPower(0);
        }
        else if (direction == -1) {
            while (Math.abs(rDrive1.getCurrentPosition()) < Math.abs(rDrive1.getTargetPosition() - 5) && opModeIsActive()) {

                leftSpeed = maxSpeed;
                rightSpeed = maxSpeed;

                leftSpeed = Range.clip(leftSpeed, -1, 1);
                rightSpeed = Range.clip(rightSpeed, -1, 1);

                lDrive1.setPower(-leftSpeed);
                rDrive1.setPower(-rightSpeed);
                lDrive2.setPower(-leftSpeed);
                rDrive2.setPower(-rightSpeed);

                telemetry.addData("1. speed", speed);
                telemetry.addData("2. leftSpeed", leftSpeed);
                telemetry.addData("3. rightSpeed", rightSpeed);
                updateTelemetry(telemetry);
            }

            lDrive1.setPower(0);
            rDrive1.setPower(0);
            lDrive2.setPower(0);
            rDrive2.setPower(0);
        }
        else {
            telemetry.addLine("Invalid direction");
            telemetry.update();
            sleep(10000);
        }
    }

    private void encoderTurn(double power, double distance) {
        int ENCODER_CPR = 1120; // Encoder counts per Rev
        double gearRatio = 1.75; // [Gear Ratio]:1
        double circumference = 13.10; // Wheel circumference
        double ROTATIONS = distance / (circumference * gearRatio); // Number of rotations to drive
        double COUNTS = ENCODER_CPR * ROTATIONS; // Number of encoder counts to drive
        int lTarget1 = lDrive1.getCurrentPosition() + (int) COUNTS;
        int lTarget2 = lDrive2.getCurrentPosition() + (int) COUNTS;
        int rTarget1 = rDrive1.getCurrentPosition() - (int) COUNTS;
        int rTarget2 = rDrive2.getCurrentPosition() - (int) COUNTS;
        lDrive1.setTargetPosition(lTarget1);
        lDrive2.setTargetPosition(lTarget2);
        rDrive1.setTargetPosition(rTarget1);
        rDrive2.setTargetPosition(rTarget2);
        lDrive1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rDrive1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rDrive2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lDrive1.setPower(power);
        lDrive2.setPower(power);
        rDrive1.setPower(-power);
        rDrive2.setPower(-power);
        while(lDrive1.isBusy()) {
            telemetry.addData("lDrive1 Position", lDrive1.getCurrentPosition());
            telemetry.addData("lDrive2 Position", lDrive2.getCurrentPosition());
            telemetry.addData("rDrive1 Position", rDrive1.getCurrentPosition());
            telemetry.addData("rDrive2 Position", rDrive2.getCurrentPosition());
            telemetry.addData("Target Position", lTarget1);
            telemetry.update();
        }
        lDrive1.setPower(0);
        lDrive2.setPower(0);
        rDrive1.setPower(0);
        rDrive2.setPower(0);

        lDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void runOpMode() throws InterruptedException {
        //##############Init##############
        rDrive1 = hardwareMap.dcMotor.get("rDrive1");
        rDrive2 = hardwareMap.dcMotor.get("rDrive2");
        lDrive1 = hardwareMap.dcMotor.get("lDrive1");
        lDrive2 = hardwareMap.dcMotor.get("lDrive2");
        lDrive1.setDirection(DcMotor.Direction.REVERSE);
        lDrive2.setDirection(DcMotor.Direction.REVERSE);

        //sweeper = hardwareMap.dcMotor.get("sweeper");
        catapult = hardwareMap.dcMotor.get("catapult");
        button = hardwareMap.servo.get("button");
        hopper = hardwareMap.servo.get("hopper");
        belt = hardwareMap.servo.get("belt");
        touch = hardwareMap.touchSensor.get("t");
        color = hardwareMap.colorSensor.get("color");


        fODSensor = hardwareMap.opticalDistanceSensor.get("fOD");
        bODSensor = hardwareMap.opticalDistanceSensor.get("bOD");
        fRangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "fRS");
        bRangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "bRS");
        hopper.setPosition(0.8);
        button.setPosition(0.5);
        belt.setPosition(.5);
        double distance;
        double maxSpeed;
        int direction;



        lDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        idle();

        waitForStart();
        driveBackwardToWall();
        lineUp();
        driveBackwardAlongWall();
        recognizeColorRed(-1);
        wrongRed();
        lDrive1.setPower(.4);
        rDrive1.setPower(.4);
        lDrive2.setPower(.4);
        rDrive2.setPower(.4);
        sleep(2000);
        lDrive1.setPower(0);
        rDrive1.setPower(0);
        lDrive2.setPower(0);
        rDrive2.setPower(0);
        driveForwardAlongWall();
        recognizeColorRed(1);
        wrongRed();
        lineUp();
        encoderTurn(-0.3, -11);
        distance = 12;
        maxSpeed = .5;
        direction = 1;
        drive(distance, maxSpeed, direction);
        fire();

    }
}