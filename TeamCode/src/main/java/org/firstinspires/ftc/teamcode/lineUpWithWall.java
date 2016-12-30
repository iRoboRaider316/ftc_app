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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name="lineUpWithWall", group="LinearOpMode")

public class lineUpWithWall extends LinearOpMode {

    private DcMotor catapult;
    //private DcMotor sweeper;
    private DcMotor lDrive1;
    private DcMotor lDrive2;
    private DcMotor rDrive1;
    private DcMotor rDrive2;
    private Servo lButton;
    private Servo rButton;
    private Servo hopper;

    private TouchSensor touch;
    private GyroSensor gyroSensor;
    private ModernRoboticsI2cGyro gyro;
    private ColorSensor color;
    private OpticalDistanceSensor rODSensor;
    private OpticalDistanceSensor lODSensor;
    I2cAddr RANGE1ADDRESS = new I2cAddr(0x14); //Default I2C address for MR Range (7-bit)
    I2cAddr RANGE2ADDRESS = new I2cAddr(0x13);


    //private I2cAddr RANGE1ADDRESS = new I2cAddr(0x28); //Default I2C address for MR Range (7-bit)
    private static final int RANGE1_REG_START = 0x04; //Register to start reading
    private static final int RANGE1_READ_LENGTH = 2; //Number of byte to read
    private ModernRoboticsI2cRangeSensor rangeSensor;
    //2nd range
    private static final int RANGE2_REG_START = 0x04; //Register to start reading
    private static final int RANGE2_READ_LENGTH = 2; //Number of byte to read
    private ModernRoboticsI2cRangeSensor rangeSensor2;
    private void lineUpWithWall() throws InterruptedException {
        //prepare first range sensor
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




        //drive to wall
        lDrive1.setPower(0.3);
        rDrive1.setPower(0.3);
        lDrive2.setPower(0.3);
        rDrive2.setPower(0.3);
        while((range1Cache[0] & 0xFF) > 20) {
            range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
            range2Cache = RANGE2Reader.read(RANGE2_REG_START, RANGE2_READ_LENGTH);
            telemetry.addData("Range value:", (range1Cache[0] & 0xFF) );
            telemetry.addData("Range2 value:", (range2Cache[0] & 0xFF) );
            telemetry.update();
        }
        //stop at wall
        lDrive1.setPower(0);
        rDrive1.setPower(0);
        lDrive2.setPower(0);
        rDrive2.setPower(0);
        sleep(300);
        lDrive1.setPower(-0.3);
        rDrive1.setPower(0.3);
        lDrive2.setPower(-0.3);
        rDrive2.setPower(0.3);
        while (!(range2Cache [0]  == range1Cache [0]) ){
            range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
            range2Cache = RANGE2Reader.read(RANGE2_REG_START, RANGE2_READ_LENGTH);
            telemetry.addData("Range value:", (range1Cache[0] & 0xFF) );
            telemetry.addData("Range2 value:", (range2Cache[0] & 0xFF) );
            telemetry.update();
        }
        lDrive1.setPower(0);
        rDrive1.setPower(0);
        lDrive2.setPower(0);
        rDrive2.setPower(0);
        sleep(100);
        if ((range2Cache [0] >= range1Cache [0])) {
            lDrive1.setPower(-0.3);
            rDrive1.setPower(0.3);
            lDrive2.setPower(-0.3);
            rDrive2.setPower(0.3);
            while (!(range2Cache [0] == range1Cache [0]) ){
                range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
                range2Cache = RANGE2Reader.read(RANGE2_REG_START, RANGE2_READ_LENGTH);
                telemetry.addData("Range value:", (range1Cache[0] & 0xFF) );
                telemetry.addData("Range2 value:", (range2Cache[0] & 0xFF) );
                telemetry.update();
            }
            lDrive1.setPower(0);
            rDrive1.setPower(0);
            lDrive2.setPower(0);
            rDrive2.setPower(0);

        }
       else if ((range2Cache [0] <= range1Cache [0])) {
            lDrive1.setPower(0.3);
            rDrive1.setPower(-0.3);
            lDrive2.setPower(0.3);
            rDrive2.setPower(-0.3);
            while (!(range2Cache [0] == range1Cache [0]) ){
                range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
                range2Cache = RANGE2Reader.read(RANGE2_REG_START, RANGE2_READ_LENGTH);
                telemetry.addData("Range value:", (range1Cache[0] & 0xFF) );
                telemetry.addData("Range2 value:", (range2Cache[0] & 0xFF) );
                telemetry.update();
            }
            lDrive1.setPower(0);
            rDrive1.setPower(0);
            lDrive2.setPower(0);
            rDrive2.setPower(0);
        }


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
        lButton = hardwareMap.servo.get("lButton");
        rButton = hardwareMap.servo.get("rButton");
        hopper = hardwareMap.servo.get("hopper");
        touch = hardwareMap.touchSensor.get("t");
        color = hardwareMap.colorSensor.get("color");
        waitForStart();
        lineUpWithWall();


    }
}