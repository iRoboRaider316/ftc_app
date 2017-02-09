package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="cf_wall_track", group="LinearOpMode")
//@Disabled

public class cf_wall_track extends LinearOpMode {

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

    I2cAddr RANGE1ADDRESS = new I2cAddr(0x14); //Default I2C address for MR Range (7-bit)
    I2cAddr RANGE2ADDRESS = new I2cAddr(0x18);


    //private I2cAddr RANGE1ADDRESS = new I2cAddr(0x28); //Default I2C address for MR Range (7-bit)
    private static final int RANGE1_REG_START = 0x04; //Register to start reading
    private static final int RANGE1_READ_LENGTH = 2; //Number of byte to read
    private ModernRoboticsI2cRangeSensor rangeSensor;
    //2nd range
    private static final int RANGE2_REG_START = 0x04; //Register to start reading
    private static final int RANGE2_READ_LENGTH = 2; //Number of byte to read
    private ModernRoboticsI2cRangeSensor rangeSensor2;


    public void driveForwardToWall() throws InterruptedException {
        I2cDevice RANGE1 = hardwareMap.i2cDevice.get("range");
        I2cDeviceSynch RANGE1Reader = new I2cDeviceSynchImpl(RANGE1, RANGE1ADDRESS, false);
        RANGE1Reader.engage();
        byte[] range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
        RANGE1Reader.engage();
        //prepare second range sensor
        I2cDevice RANGE2 = hardwareMap.i2cDevice.get("range2");
        I2cDeviceSynch RANGE2Reader = new I2cDeviceSynchImpl(RANGE2, RANGE2ADDRESS, false);
        byte[] range2Cache = RANGE2Reader.read(RANGE2_REG_START, RANGE2_READ_LENGTH);
        RANGE2Reader.engage();
        telemetry.addData("Status", "Initialized");
        range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
        range2Cache = RANGE2Reader.read(RANGE2_REG_START, RANGE2_READ_LENGTH);
        telemetry.addData("Range value:", (range1Cache[0] & 0xFF));
        telemetry.addData("Range2 value:", (range2Cache[0] & 0xFF));
        telemetry.update();
        lDrive1.setPower(0.4);
        rDrive1.setPower(0.4);
        lDrive2.setPower(0.4);
        rDrive2.setPower(0.4);
        sleep(2000);
        while (range1Cache[0] >= 15 && opModeIsActive()) {
            range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
            range2Cache = RANGE2Reader.read(RANGE2_REG_START, RANGE2_READ_LENGTH);
            telemetry.addData("Range value:", (range1Cache[0] & 0xFF));
            telemetry.addData("Range2 value:", (range2Cache[0] & 0xFF));
            if (range1Cache[0] <= 200) {
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
        I2cDeviceSynch RANGE1Reader = new I2cDeviceSynchImpl(RANGE1, RANGE1ADDRESS, false);
        RANGE1Reader.engage();
        byte[] range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
        RANGE1Reader.engage();
        //prepare second range sensor
        I2cDevice RANGE2 = hardwareMap.i2cDevice.get("range2");
        I2cDeviceSynch RANGE2Reader = new I2cDeviceSynchImpl(RANGE2, RANGE2ADDRESS, false);
        byte[] range2Cache = RANGE2Reader.read(RANGE2_REG_START, RANGE2_READ_LENGTH);
        RANGE2Reader.engage();
        telemetry.addData("Status", "Initialized");
        range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
        range2Cache = RANGE2Reader.read(RANGE2_REG_START, RANGE2_READ_LENGTH);
        telemetry.addData("Range value:", (range1Cache[0] & 0xFF));
        telemetry.addData("Range2 value:", (range2Cache[0] & 0xFF));
        telemetry.update();
        lDrive1.setPower(-0.35);
        rDrive1.setPower(-0.3);
        lDrive2.setPower(-0.35);
        rDrive2.setPower(-0.3);
        sleep(2000);
        while (range2Cache[0] >= 15 && opModeIsActive()) {
            range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
            range2Cache = RANGE2Reader.read(RANGE2_REG_START, RANGE2_READ_LENGTH);
            telemetry.addData("Range value:", (range1Cache[0] & 0xFF));
            telemetry.addData("Range2 value:", (range2Cache[0] & 0xFF));
            if (range2Cache[0] <= 200) {
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
        //I2cDeviceSynch RANGE1Reader = new I2cDeviceSynchImpl(RANGE1, RANGE1ADDRESS, false);
        //RANGE1Reader.engage();
        //byte[] range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
        //RANGE1Reader.engage();
        //prepare second range sensor
        I2cDevice RANGE2 = hardwareMap.i2cDevice.get("range2");
        I2cDeviceSynch RANGE2Reader = new I2cDeviceSynchImpl(RANGE2, RANGE2ADDRESS, false);
        byte[] range2Cache = RANGE2Reader.read(RANGE2_REG_START, RANGE2_READ_LENGTH);
        RANGE2Reader.engage();
        telemetry.addData("Status", "Initialized");
        //range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
        range2Cache = RANGE2Reader.read(RANGE2_REG_START, RANGE2_READ_LENGTH);
        //telemetry.addData("Range value:", (range1Cache[0] & 0xFF));
        telemetry.addData("Range2 value:", (range2Cache[0] & 0xFF));
        telemetry.update();

        while (bODSensor.getRawLightDetected() < .1 && opModeIsActive()) {
            //range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
            range2Cache = RANGE2Reader.read(RANGE2_REG_START, RANGE2_READ_LENGTH);
            double range2 = range2Cache[0];

            //telemetry.addData("Range value:", (range1Cache[0] & 0xFF));
            telemetry.addData("Range2 value:", (range2));
            telemetry.addData("bOD light", (bODSensor.getRawLightDetected()));

            double error = (range2-15)/60;
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
        int targetHeading;
        int direction;
        long time;
        double parallel;

        lDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        lDrive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rDrive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        idle();

        waitForStart();

        //driveForwardToWall();
        wallTrack();
        sleep(10000);
    }
}