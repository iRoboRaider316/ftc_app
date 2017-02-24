package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="cf_blue_beacons_IL_State", group="LinearOpMode")
//@Disabled

public class cf_blue_beacons_IL_State extends LinearOpMode {

    private DcMotor catapult;
    //private DcMotor sweeper;
    private DcMotor lDrive1;
    private DcMotor lDrive2;
    private DcMotor rDrive1;
    private DcMotor rDrive2;
    private Servo button;
    private Servo hopper;
    private Servo belt;

    private TouchSensor touch;
    private ColorSensor color;
    private OpticalDistanceSensor fODSensor;
    private OpticalDistanceSensor bODSensor;

    I2cAddr RANGE1ADDRESS = new I2cAddr(0x14); //Default I2C address for MR Range (7-bit)
    I2cAddr RANGE2ADDRESS = new I2cAddr(0x18);

    //private I2cAddr RANGEfADDRESS = new I2cAddr(0x28); //Default I2C address for MR Range (7-bit)
    private static final int RANGE1_REG_START = 0x04; //Register to start reading
    private static final int RANGE1_READ_LENGTH = 2; //Number of byte to read
    //2nd range
    private static final int RANGE2_REG_START = 0x04; //Register to start reading
    private static final int RANGE2_READ_LENGTH = 2; //Number of byte to read

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
//        I2cDevice RANGE1 = hardwareMap.i2cDevice.get("range");
//        I2cDeviceSynch RANGE1Reader = new I2cDeviceSynchImpl(RANGE1, RANGE1ADDRESS, false);
//        RANGE1Reader.engage();
//        byte[] range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
//        RANGE1Reader.engage();
        //prepare second range sensor
        I2cDevice RANGE2 = hardwareMap.i2cDevice.get("range2");
        I2cDeviceSynch RANGE2Reader = new I2cDeviceSynchImpl(RANGE2, RANGE2ADDRESS, false);
        byte[] range2Cache = RANGE2Reader.read(RANGE2_REG_START, RANGE2_READ_LENGTH);
        RANGE2Reader.engage();
        telemetry.addData("Status", "Initialized");
//        range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
//        range2Cache = RANGE2Reader.read(RANGE2_REG_START, RANGE2_READ_LENGTH);
//        telemetry.addData("Range value:", (range1Cache[0] & 0xFF));
        telemetry.addData("Range2 value:", (range2Cache[0] & 0xFF));
        telemetry.update();
        lDrive1.setPower(0.4);
        rDrive1.setPower(0.4);
        lDrive2.setPower(0.4);
        rDrive2.setPower(0.4);
        //sleep(2000);
        while (range2Cache[0] >= 12 && opModeIsActive()) {
//            range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
            range2Cache = RANGE2Reader.read(RANGE2_REG_START, RANGE2_READ_LENGTH);
//            telemetry.addData("Range value:", (range1Cache[0] & 0xFF));
            telemetry.addData("Range2 value:", (range2Cache[0] & 0xFF));
        }
        lDrive1.setPower(0);
        rDrive1.setPower(0);
        lDrive2.setPower(0);
        rDrive2.setPower(0);
    }

    private void driveBackwardToWall() throws InterruptedException {
        I2cDevice RANGE1 = hardwareMap.i2cDevice.get("range");
        I2cDeviceSynch RANGE1Reader = new I2cDeviceSynchImpl(RANGE1, RANGE1ADDRESS, false);
        RANGE1Reader.engage();
        byte[] range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
        RANGE1Reader.engage();
//        //prepare second range sensor
//        I2cDevice RANGE2 = hardwareMap.i2cDevice.get("range2");
//        I2cDeviceSynch RANGE2Reader = new I2cDeviceSynchImpl(RANGE2, RANGE2ADDRESS, false);
//        byte[] range2Cache = RANGE2Reader.read(RANGE2_REG_START, RANGE2_READ_LENGTH);
//        RANGE2Reader.engage();
        telemetry.addData("Status", "Initialized");
//        range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
//        range2Cache = RANGE2Reader.read(RANGE2_REG_START, RANGE2_READ_LENGTH);
        telemetry.addData("Range value:", (range1Cache[0] & 0xFF));
//        telemetry.addData("Range2 value:", (range2Cache[0] & 0xFF));
        telemetry.update();
        lDrive1.setPower(-0.4);
        rDrive1.setPower(-0.4);
        lDrive2.setPower(-0.4);
        rDrive2.setPower(-0.4);
        //sleep(2000);

        while (range1Cache[0] >= 12 && opModeIsActive()) {
            range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
//            range2Cache = RANGE2Reader.read(RANGE2_REG_START, RANGE2_READ_LENGTH);
            telemetry.addData("Range value:", (range1Cache[0] & 0xFF));
//            telemetry.addData("Range2 value:", (range2Cache[0] & 0xFF));
            telemetry.update();
        }
        lDrive1.setPower(0);
        rDrive1.setPower(0);
        lDrive2.setPower(0);
        rDrive2.setPower(0);
    }

    private void wallTrackBackward() throws InterruptedException {
        //I2cDevice RANGE1 = hardwareMap.i2cDevice.get("range");
        //I2cDeviceSynch RANGE1Reader = new I2cDeviceSynchImpl(RANGE1, RANGEfADDRESS, false);
        //RANGE1Reader.engage();
        //byte[] range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
        //RANGE1Reader.engage();
        //prepare second range sensor
        I2cDevice RANGE1 = hardwareMap.i2cDevice.get("range");
        I2cDeviceSynch RANGE1Reader = new I2cDeviceSynchImpl(RANGE1, RANGE1ADDRESS, false);
//        byte[] range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
        RANGE1Reader.engage();
        telemetry.addData("Status", "Initialized");
        //range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
//        range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
        //telemetry.addData("Range value:", (range1Cache[0] & 0xFF));
//        telemetry.addData("Range2 value:", (range1Cache[0] & 0xFF));
        telemetry.update();

        while (bODSensor.getRawLightDetected() < .075 && opModeIsActive()) {
            //range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
            byte[] range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
            double range1 = range1Cache[0];

            //telemetry.addData("Range value:", (range1Cache[0] & 0xFF));
            telemetry.addData("Range1 value:", (range1));
            telemetry.addData("bOD light", (bODSensor.getRawLightDetected()));

            double error = (range1-12)/70; //55
            if (range1 > 100 || range1 < 1)
                error = 0;
            telemetry.addData("Error", error);
            if (error > 0.1){
                error = 0.1;
            }
            double leftSpeed = -.2-error;
            double rightSpeed = -.2+error;

            leftSpeed = Range.clip(leftSpeed, -1, 1);
            rightSpeed = Range.clip(rightSpeed, -1, 1);

            lDrive1.setPower(leftSpeed);
            lDrive2.setPower(leftSpeed);
            rDrive1.setPower(rightSpeed);
            rDrive2.setPower(rightSpeed);

            telemetry.addData("leftSpeed",(leftSpeed));
            telemetry.addData("rightSpeed",(rightSpeed));
            telemetry.update();
        }
        lDrive1.setPower(0);
        lDrive2.setPower(0);
        rDrive1.setPower(0);
        rDrive2.setPower(0);
    }

    private void wallTrackForward() throws InterruptedException {
//        I2cDevice RANGE1 = hardwareMap.i2cDevice.get("range");
//        I2cDeviceSynch RANGE1Reader = new I2cDeviceSynchImpl(RANGE1, RANGE1ADDRESS, false);
//        RANGE1Reader.engage();
//        byte[] range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
//        RANGE1Reader.engage();
        //prepare second range sensor
        I2cDevice RANGE2 = hardwareMap.i2cDevice.get("range2");
        I2cDeviceSynch RANGE2Reader = new I2cDeviceSynchImpl(RANGE2, RANGE2ADDRESS, false);
//        byte[] range2Cache = RANGE2Reader.read(RANGE2_REG_START, RANGE2_READ_LENGTH);
        RANGE2Reader.engage();
        telemetry.addData("Status", "Initialized");
        //range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
//        range2Cache = RANGE2Reader.read(RANGE2_REG_START, RANGE2_READ_LENGTH);
        //telemetry.addData("Range value:", (range1Cache[0] & 0xFF));
//        telemetry.addData("Range2 value:", (range2Cache[0] & 0xFF));
        telemetry.update();
        while (fODSensor.getRawLightDetected() < .1 && opModeIsActive()) {
//            range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
            byte[] range2Cache = RANGE2Reader.read(RANGE2_REG_START, RANGE2_READ_LENGTH);
            double range2 = range2Cache[0];
//            double range1 = range1Cache[0];
            //telemetry.addData("Range value:", (range1Cache[0] & 0xFF));
            telemetry.addData("Range2 value:", (range2));
            telemetry.addData("bOD light", (bODSensor.getRawLightDetected()));

            double error = ((range2-12)/70); //60
            if (range2 > 100 || range2 < 1)
                error = 0;
            telemetry.addData("Error", error);
            if (error > 0.1){
                error = 0.1;
            }
            double leftSpeed = .2+error;
            double rightSpeed = .2-error;


            leftSpeed = Range.clip(leftSpeed, -1, 1);
            rightSpeed = Range.clip(rightSpeed, -1, 1);

            lDrive1.setPower(leftSpeed);
            lDrive2.setPower(leftSpeed);
            rDrive1.setPower(rightSpeed);
            rDrive2.setPower(rightSpeed);
            telemetry.addData("leftSpeed",(leftSpeed));
            telemetry.addData("rightSpeed",(rightSpeed));
            telemetry.update();
        }
        lDrive1.setPower(0);
        lDrive2.setPower(0);
        rDrive1.setPower(0);
        rDrive2.setPower(0);
    }

    private void lineUp() throws InterruptedException {
        I2cDevice RANGE1 = hardwareMap.i2cDevice.get("range");
        I2cDeviceSynch RANGE1Reader = new I2cDeviceSynchImpl(RANGE1, RANGE1ADDRESS, false);
        RANGE1Reader.engage();
//        byte[] rangefCache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
//        RANGE1Reader.engage();
        //prepare second range sensor
        I2cDevice RANGE2 = hardwareMap.i2cDevice.get("range2");
        I2cDeviceSynch RANGE2Reader = new I2cDeviceSynchImpl(RANGE2, RANGE2ADDRESS, false);
//        byte[] rangebCache = RANGE2Reader.read(RANGE2_REG_START, RANGE2_READ_LENGTH);
        RANGE2Reader.engage();
//        rangebCache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
//        rangefCache = RANGE2Reader.read(RANGE2_REG_START, RANGE2_READ_LENGTH);

        double error = 0;
        boolean done = false;

        while (!done && opModeIsActive()) {
            byte[] rangebCache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
            byte[] rangefCache = RANGE2Reader.read(RANGE2_REG_START, RANGE2_READ_LENGTH);
            double rangeb = rangebCache[0];
            double rangef = rangefCache[0];
            telemetry.addData("Range value:", rangef);
            telemetry.addData("Range2 value:", rangeb);

            error = (rangef - rangeb)/100;
            telemetry.addData("error", error);

            lDrive1.setPower(0+error);
            lDrive2.setPower(0+error);
            rDrive1.setPower(0-error);
            rDrive2.setPower(0-error);
            telemetry.update();
            if (rangef > 200 || rangeb > 200)
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
        hopper.setPosition(0.8);
        button.setPosition(0.5);
        belt.setPosition(.5);

        double distance;
        double maxSpeed;
        int direction;
        int degrees;

        lDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        lDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        idle();

        waitForStart();

        distance = 70;
        maxSpeed = .7;
        direction = 1;
        drive(distance, maxSpeed, direction);
        // Drive forward until the range sensor detects the wall
        driveForwardToWall();
        // Turn until parallel with the wall using both range sensors
        lineUp();
        // Track forward along the wall until the white line
        wallTrackForward();
        // Push the button for red
        recognizeColorBlue(1);
        wrongBlue();
        // Drive backward past the line
        rDrive1.setPower(-0.6);
        rDrive2.setPower(-0.6);
        lDrive1.setPower(-0.6);
        lDrive2.setPower(-0.6);
        sleep(800);
        rDrive1.setPower(0);
        rDrive2.setPower(0);
        lDrive1.setPower(0);
        lDrive2.setPower(0);
        // Track backward along the wall until the next white line
        wallTrackBackward();
        // Push the button for red
        recognizeColorBlue(-1);
        wrongBlue();
//        lineUp();
//        encoderTurn(-0.3, -11);
//        distance = 12;
//        maxSpeed = .5;
//        direction = 1;
//        drive(distance, maxSpeed, direction);
//        fire();

    }
}