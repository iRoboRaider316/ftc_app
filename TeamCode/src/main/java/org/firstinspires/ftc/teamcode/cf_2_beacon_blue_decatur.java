package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="cf_2_beacon_blue_decatur", group="LinearOpMode")
//@Disabled

public class cf_2_beacon_blue_decatur extends LinearOpMode {

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
    private GyroSensor gyroSensor;
    private ModernRoboticsI2cGyro gyro;
    private ColorSensor color;
    private OpticalDistanceSensor fODSensor;
    private OpticalDistanceSensor bODSensor;

    I2cAddr RANGEfADDRESS = new I2cAddr(0x14); //Default I2C address for MR Range (7-bit)
    I2cAddr RANGEbADDRESS = new I2cAddr(0x18);

    //private I2cAddr RANGEfADDRESS = new I2cAddr(0x28); //Default I2C address for MR Range (7-bit)
    private static final int RANGE1_REG_START = 0x04; //Register to start reading
    private static final int RANGE1_READ_LENGTH = 2; //Number of byte to read
    private ModernRoboticsI2cRangeSensor rangeSensor;
    //2nd range
    private static final int RANGE2_REG_START = 0x04; //Register to start reading
    private static final int RANGE2_READ_LENGTH = 2; //Number of byte to read
    private ModernRoboticsI2cRangeSensor rangeSensor2;
    private void checkColor() throws InterruptedException {

    }

    public void driveForwardToWall() throws InterruptedException {
        I2cDevice RANGE1 = hardwareMap.i2cDevice.get("range");
        I2cDeviceSynch RANGE1Reader = new I2cDeviceSynchImpl(RANGE1, RANGEfADDRESS, false);
        RANGE1Reader.engage();
        byte[] rangefCache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
        RANGE1Reader.engage();
        //prepare second range sensor
        I2cDevice RANGE2 = hardwareMap.i2cDevice.get("range2");
        I2cDeviceSynch RANGE2Reader = new I2cDeviceSynchImpl(RANGE2, RANGEbADDRESS, false);
        byte[] rangebCache = RANGE2Reader.read(RANGE2_REG_START, RANGE2_READ_LENGTH);
        RANGE2Reader.engage();
        telemetry.addData("Status", "Initialized");
        rangefCache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
        rangebCache = RANGE2Reader.read(RANGE2_REG_START, RANGE2_READ_LENGTH);
        telemetry.addData("Range value:", (rangefCache[0] & 0xFF));
        telemetry.addData("Range2 value:", (rangebCache[0] & 0xFF));
        telemetry.update();
        // avoid the walltop

        lDrive1.setPower(0.3);
        rDrive1.setPower(0.3);
        lDrive2.setPower(0.3);
        rDrive2.setPower(0.3);
        sleep(2000);
        while (rangefCache[0] >= 17 && opModeIsActive()) {
            rangefCache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
            rangebCache = RANGE2Reader.read(RANGE2_REG_START, RANGE2_READ_LENGTH);
            telemetry.addData("Range value:", (rangefCache[0] & 0xFF));
            telemetry.addData("Range2 value:", (rangebCache[0] & 0xFF));
            if (rangefCache[0] <= 200) {
                sleep(1);
            }
        }
        lDrive1.setPower(0);
        rDrive1.setPower(0);
        lDrive2.setPower(0);
        rDrive2.setPower(0);
    }

    public void driveBackwardToWall() throws InterruptedException {
        I2cDevice RANGE1 = hardwareMap.i2cDevice.get("range");
        I2cDeviceSynch RANGE1Reader = new I2cDeviceSynchImpl(RANGE1, RANGEfADDRESS, false);
        RANGE1Reader.engage();
        byte[] rangefCache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
        RANGE1Reader.engage();
        //prepare second range sensor
        I2cDevice RANGE2 = hardwareMap.i2cDevice.get("range2");
        I2cDeviceSynch RANGE2Reader = new I2cDeviceSynchImpl(RANGE2, RANGEbADDRESS, false);
        byte[] rangebCache = RANGE2Reader.read(RANGE2_REG_START, RANGE2_READ_LENGTH);
        RANGE2Reader.engage();
        telemetry.addData("Status", "Initialized");
        rangefCache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
        rangebCache = RANGE2Reader.read(RANGE2_REG_START, RANGE2_READ_LENGTH);
        telemetry.addData("Range value:", (rangefCache[0] & 0xFF));
        telemetry.addData("Range2 value:", (rangebCache[0] & 0xFF));
        telemetry.update();
        lDrive1.setPower(-0.4);
        rDrive1.setPower(-0.4);
        lDrive2.setPower(-0.4);
        rDrive2.setPower(-0.4);
        sleep(2000);
        while (rangebCache[0] >= 17 && opModeIsActive()) {
            rangefCache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
            rangebCache = RANGE2Reader.read(RANGE2_REG_START, RANGE2_READ_LENGTH);
            telemetry.addData("Range value:", (rangefCache[0] & 0xFF));
            telemetry.addData("Range2 value:", (rangebCache[0] & 0xFF));
            if (rangebCache[0] <= 200) {
                sleep(1);
            }
        }
        lDrive1.setPower(0);
        rDrive1.setPower(0);
        lDrive2.setPower(0);
        rDrive2.setPower(0);
    }

    public void wallTrack() throws InterruptedException {
        //I2cDevice RANGE1 = hardwareMap.i2cDevice.get("range");
        //I2cDeviceSynch RANGE1Reader = new I2cDeviceSynchImpl(RANGE1, RANGEfADDRESS, false);
        //RANGE1Reader.engage();
        //byte[] range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
        //RANGE1Reader.engage();
        //prepare second range sensor
        I2cDevice RANGE2 = hardwareMap.i2cDevice.get("range2");
        I2cDeviceSynch RANGE2Reader = new I2cDeviceSynchImpl(RANGE2, RANGEbADDRESS, false);
        byte[] rangebCache = RANGE2Reader.read(RANGE2_REG_START, RANGE2_READ_LENGTH);
        RANGE2Reader.engage();
        telemetry.addData("Status", "Initialized");
        //range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
        rangebCache = RANGE2Reader.read(RANGE2_REG_START, RANGE2_READ_LENGTH);
        //telemetry.addData("Range value:", (range1Cache[0] & 0xFF));
        telemetry.addData("Range2 value:", (rangebCache[0] & 0xFF));
        telemetry.update();

        while (bODSensor.getRawLightDetected() < .075 && opModeIsActive()) {
            //range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
            rangebCache = RANGE2Reader.read(RANGE2_REG_START, RANGE2_READ_LENGTH);
            double range2 = rangebCache[0];

            //telemetry.addData("Range value:", (range1Cache[0] & 0xFF));
            telemetry.addData("Range2 value:", (range2));
            telemetry.addData("bOD light", (bODSensor.getRawLightDetected()));

            double error = (range2-12)/55;
            telemetry.addData("Error", error);

            double leftSpeed = -.2-error;
            double rightSpeed = -.2+error;
            leftSpeed = Range.clip(leftSpeed, -1, 1);
            rightSpeed = Range.clip(rightSpeed, -1, 1);

            lDrive1.setPower(leftSpeed);
            lDrive2.setPower(leftSpeed);
            rDrive1.setPower(rightSpeed);
            rDrive2.setPower(rightSpeed);

//            if (range2Cache[0] < 15) {
//                // Adjust right
//                leftSpeed = -.3-error;
//                rightSpeed = -.3+error;
//                leftSpeed = Range.clip(leftSpeed, -1, 1);
//                rightSpeed = Range.clip(rightSpeed, -1, 1);
//                lDrive1.setPower(leftSpeed);
//                lDrive2.setPower(leftSpeed);
//                rDrive1.setPower(rightSpeed);
//                rDrive2.setPower(rightSpeed);
//                telemetry.addLine("Adjusting Right");
//            }
//            else if (range2Cache[0] > 15){
//                // Adjust left
//                leftSpeed = -.3+error;
//                rightSpeed = -.3-error;
//                leftSpeed = Range.clip(leftSpeed, -1, 1);
//                rightSpeed = Range.clip(rightSpeed, -1, 1);
//                lDrive1.setPower(leftSpeed);
//                lDrive2.setPower(leftSpeed);
//                rDrive1.setPower(rightSpeed);
//                rDrive2.setPower(rightSpeed);
//                telemetry.addLine("Adjusting Left");
//            }
//            else {
//                // Drive straight backward
//                leftSpeed = -.3;
//                rightSpeed = -.3;
//                lDrive1.setPower(leftSpeed);
//                lDrive2.setPower(leftSpeed);
//                rDrive1.setPower(rightSpeed);
//                rDrive2.setPower(rightSpeed);
//                telemetry.addLine("Driving Straight");
//            }
            telemetry.addData("leftSpeed",(leftSpeed));
            telemetry.addData("rightSpeed",(rightSpeed));
            telemetry.update();
        }
        lDrive1.setPower(0);
        lDrive2.setPower(0);
        rDrive1.setPower(0);
        rDrive2.setPower(0);
    }

    public void wallTrackBack() throws InterruptedException {
        I2cDevice RANGE1 = hardwareMap.i2cDevice.get("range");
        I2cDeviceSynch RANGE1Reader = new I2cDeviceSynchImpl(RANGE1, RANGEfADDRESS, false);
        RANGE1Reader.engage();
        byte[] rangefCache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
        RANGE1Reader.engage();
        //prepare second range sensor
        I2cDevice RANGE2 = hardwareMap.i2cDevice.get("range2");
        I2cDeviceSynch RANGE2Reader = new I2cDeviceSynchImpl(RANGE2, RANGEbADDRESS, false);
        byte[] rangebCache = RANGE2Reader.read(RANGE2_REG_START, RANGE2_READ_LENGTH);
        RANGE2Reader.engage();
        telemetry.addData("Status", "Initialized");
        //range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
        rangebCache = RANGE2Reader.read(RANGE2_REG_START, RANGE2_READ_LENGTH);
        //telemetry.addData("Range value:", (range1Cache[0] & 0xFF));
        telemetry.addData("Range2 value:", (rangebCache[0] & 0xFF));
        telemetry.update();
        while (fODSensor.getRawLightDetected() < .1 && opModeIsActive()) {
            rangefCache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
            rangebCache = RANGE2Reader.read(RANGE2_REG_START, RANGE2_READ_LENGTH);
            double range2 = rangebCache[0];
            double range1 = rangefCache[0];
            //telemetry.addData("Range value:", (range1Cache[0] & 0xFF));
            telemetry.addData("Range2 value:", (range2));
            telemetry.addData("bOD light", (bODSensor.getRawLightDetected()));
            double error = ((range1-12)/60);
            telemetry.addData("Error", error);
            double leftSpeed = .15+error;
            double rightSpeed = .15-error;
            leftSpeed = Range.clip(leftSpeed, -1, 1);
            rightSpeed = Range.clip(rightSpeed, -1, 1);
            lDrive1.setPower(leftSpeed);
            lDrive2.setPower(leftSpeed);
            rDrive1.setPower(rightSpeed);
            rDrive2.setPower(rightSpeed);
//            if (range2Cache[0] < 15) {
//                // Adjust right
//                leftSpeed = -.3-error;
//                rightSpeed = -.3+error;
//                leftSpeed = Range.clip(leftSpeed, -1, 1);
//                rightSpeed = Range.clip(rightSpeed, -1, 1);
//                lDrive1.setPower(leftSpeed);
//                lDrive2.setPower(leftSpeed);
//                rDrive1.setPower(rightSpeed);
//                rDrive2.setPower(rightSpeed);
//                telemetry.addLine("Adjusting Right");
//            }
//            else if (range2Cache[0] > 15){
//                // Adjust left
//                leftSpeed = -.3+error;
//                rightSpeed = -.3-error;
//                leftSpeed = Range.clip(leftSpeed, -1, 1);
//                rightSpeed = Range.clip(rightSpeed, -1, 1);
//                lDrive1.setPower(leftSpeed);
//                lDrive2.setPower(leftSpeed);
//                rDrive1.setPower(rightSpeed);
//                rDrive2.setPower(rightSpeed);
//                telemetry.addLine("Adjusting Left");
//            }
//            else {
//                // Drive straight backward
//                leftSpeed = -.3;
//                rightSpeed = -.3;
//                lDrive1.setPower(leftSpeed);
//                lDrive2.setPower(leftSpeed);
//                rDrive1.setPower(rightSpeed);
//                rDrive2.setPower(rightSpeed);
//                telemetry.addLine("Driving Straight");
//            }
            telemetry.addData("leftSpeed",(leftSpeed));
            telemetry.addData("rightSpeed",(rightSpeed));
            telemetry.update();
        }
        lDrive1.setPower(0);
        lDrive2.setPower(0);
        rDrive1.setPower(0);
        rDrive2.setPower(0);
    }

    // This is the Drive Method
    // It will take in two static values: distance and maxSpeed.
    // It will then calculate the encoder counts to drive and drive the distance at the specified power,
    // accelerating to max speed for the first third of the distance, maintaining that speed for the second third,
    // and decelerating to a minimum speed for the last third.
    // If the robot deviates from the initial gyro heading, it will correct itself proportionally to the error.
    public void drive(double distance, double maxSpeed) {
        int ENCODER_CPR = 1120; // Encoder counts per Rev
        double gearRatio = 1.75; // [Gear Ratio]:1
        double circumference = 13.10; // Wheel circumference
        double ROTATIONS = distance / (circumference * gearRatio); // Number of rotations to drive
        double COUNTS = ENCODER_CPR * ROTATIONS; // Number of encoder counts to drive
        //double startPosition = rDrive1.getCurrentPosition();

        //double oldSpeed;
        double speed = 0;
        //double minSpeed = 0.3;
        //double acceleration = 0.01;
        double leftSpeed;
        double rightSpeed;

        rDrive1.setTargetPosition(rDrive1.getCurrentPosition() + (int) COUNTS);

        rDrive1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lDrive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double heading = gyroSensor.getHeading();

        while (rDrive1.getCurrentPosition()<(rDrive1.getTargetPosition()-5)){

            leftSpeed = maxSpeed-((gyroSensor.getHeading()-heading)/10);
            rightSpeed = maxSpeed+((gyroSensor.getHeading()-heading)/10);

            leftSpeed = Range.clip(leftSpeed, -1, 1);
            rightSpeed = Range.clip(rightSpeed, -1, 1);

            lDrive1.setPower(leftSpeed);
            rDrive1.setPower(rightSpeed);
            lDrive2.setPower(leftSpeed);
            rDrive2.setPower(rightSpeed);

            telemetry.addData("1. speed", speed);
            telemetry.addData("2. leftSpeed", leftSpeed);
            telemetry.addData("3. rightSpeed", rightSpeed);
            telemetry.addData("4. IntegratedZValue", gyro.getIntegratedZValue());
            updateTelemetry(telemetry);
        }

        lDrive1.setPower(0);
        rDrive1.setPower(0);
        lDrive2.setPower(0);
        rDrive2.setPower(0);

        lDrive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rDrive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void lineUp() throws InterruptedException {
        I2cDevice RANGE1 = hardwareMap.i2cDevice.get("range");
        I2cDeviceSynch RANGE1Reader = new I2cDeviceSynchImpl(RANGE1, RANGEfADDRESS, false);
        RANGE1Reader.engage();
        byte[] rangefCache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
        RANGE1Reader.engage();
        //prepare second range sensor
        I2cDevice RANGE2 = hardwareMap.i2cDevice.get("range2");
        I2cDeviceSynch RANGE2Reader = new I2cDeviceSynchImpl(RANGE2, RANGEbADDRESS, false);
        byte[] rangebCache = RANGE2Reader.read(RANGE2_REG_START, RANGE2_READ_LENGTH);
        RANGE2Reader.engage();
        telemetry.addData("Status", "Initialized");
        rangefCache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
        rangebCache = RANGE2Reader.read(RANGE2_REG_START, RANGE2_READ_LENGTH);
        telemetry.addData("Range value:", (rangefCache[0] & 0xFF));
        telemetry.addData("Range2 value:", (rangebCache[0] & 0xFF));
        telemetry.update();
        telemetry.update();
        if (rangefCache[0] > rangebCache[0]) {
            lDrive1.setPower(0.2);
            rDrive1.setPower(-0.2);
            lDrive2.setPower(0.2);
            rDrive2.setPower(-0.2);
            while (rangefCache[0] > rangebCache[0]  && opModeIsActive()) {
                rangefCache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
                rangebCache = RANGE2Reader.read(RANGE2_REG_START, RANGE2_READ_LENGTH);
                telemetry.addData("Range value:", (rangefCache[0] & 0xFF));
                telemetry.addData("Range2 value:", (rangebCache[0] & 0xFF));
                if (rangefCache[0] < 200) {
                    lDrive1.setPower(0);
                    rDrive1.setPower(0);
                    lDrive2.setPower(0);
                    rDrive2.setPower(0);
                    sleep(10);
                    lDrive1.setPower(0.2);
                    rDrive1.setPower(-0.15);
                    lDrive2.setPower(0.2);
                    rDrive2.setPower(-0.15);
                }
                else if (rangebCache[0] < 200) {
                    lDrive1.setPower(0);
                    rDrive1.setPower(0);
                    lDrive2.setPower(0);
                    rDrive2.setPower(0);
                    lDrive1.setPower(0.2);
                    rDrive1.setPower(-0.15);
                    lDrive2.setPower(0.2);
                    rDrive2.setPower(-0.15);
                }
            }
            lDrive1.setPower(0);
            rDrive1.setPower(0);
            lDrive2.setPower(0);
            rDrive2.setPower(0);
        }
        else if (rangefCache[0] < rangebCache[0]) {
            lDrive1.setPower(-0.2);
            rDrive1.setPower(0.2);
            lDrive2.setPower(-0.2);
            rDrive2.setPower(0.2);
            while (rangefCache[0] < rangebCache[0]&& opModeIsActive()) {
                rangefCache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
                rangebCache = RANGE2Reader.read(RANGE2_REG_START, RANGE2_READ_LENGTH);
                telemetry.addData("Range value:", (rangefCache[0] & 0xFF));
                telemetry.addData("Range2 value:", (rangebCache[0] & 0xFF));
                if (rangefCache[0] < 200) {
                    lDrive1.setPower(0);
                    rDrive1.setPower(0);
                    lDrive2.setPower(0);
                    rDrive2.setPower(0);
                    sleep(100);
                    lDrive1.setPower(-0.2);
                    rDrive1.setPower(0.2);
                    lDrive2.setPower(-0.2);
                    rDrive2.setPower(0.2);
                }
                else if (rangebCache[0] < 200) {
                    lDrive1.setPower(0);
                    rDrive1.setPower(0);
                    lDrive2.setPower(0);
                    rDrive2.setPower(0);
                    sleep(100);
                    lDrive1.setPower(-0.2);
                    rDrive1.setPower(0.2);
                    lDrive2.setPower(-0.2);
                    rDrive2.setPower(0.2);
                }
            }
            lDrive1.setPower(0);
            rDrive1.setPower(0);
            lDrive2.setPower(0);
            rDrive2.setPower(0);
        }
        else {
            lDrive1.setPower(0);
            rDrive1.setPower(0);
            lDrive2.setPower(0);
            rDrive2.setPower(0);
        }
    }

    // Fully working as of 2/8/17
    private void recognizeColorRed(int direction) throws InterruptedException {
        color.enableLed(false);
        while (color.red() < (color.blue())) {
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
        touch = hardwareMap.touchSensor.get("touch");
        color = hardwareMap.colorSensor.get("color");


        fODSensor = hardwareMap.opticalDistanceSensor.get("fOD");
        bODSensor = hardwareMap.opticalDistanceSensor.get("bOD");
        hopper.setPosition(0.8);
        button.setPosition(0.5);
        belt.setPosition(.5);

        double distance;
        double maxSpeed;
        int targetHeading;
        int direction;
        long time;
        double parallel;

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

        driveForwardToWall();
        lineUp();
        wallTrackBack();
        recognizeColorBlue(1);
        rDrive1.setPower(-0.4);
        rDrive2.setPower(-0.4);
        lDrive1.setPower(-0.4);
        lDrive2.setPower(-0.4);
        sleep(5000);
        rDrive1.setPower(0);
        rDrive2.setPower(0);
        lDrive1.setPower(0);
        lDrive2.setPower(0);
        wallTrack();
        recognizeColorBlue(-1);

    }
}