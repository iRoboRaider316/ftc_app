package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

@Autonomous(name="cf_linup", group="LinearOpMode")
@Disabled

public class cf_lineup extends LinearOpMode {

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
    private ModernRoboticsI2cRangeSensor rangeSensor;
    //2nd range
    private static final int RANGE2_REG_START = 0x04; //Register to start reading
    private static final int RANGE2_READ_LENGTH = 2; //Number of byte to read
    private ModernRoboticsI2cRangeSensor rangeSensor2;

    private void lineUp() throws InterruptedException {
        I2cDevice RANGE1 = hardwareMap.i2cDevice.get("bRS");
        I2cDeviceSynch RANGE1Reader = new I2cDeviceSynchImpl(RANGE1, RANGE1ADDRESS, false);
        RANGE1Reader.engage();
        I2cDevice RANGE2 = hardwareMap.i2cDevice.get("fRS");
        I2cDeviceSynch RANGE2Reader = new I2cDeviceSynchImpl(RANGE2, RANGE2ADDRESS, false);
        RANGE2Reader.engage();

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
            if (error > .15)
                error = .15;
            else if (error < .05 && error > 0)
                error = .05;
            else if (error < -.15)
                error = -.15;
            else if (error < 0 && error > -.05)
                error = -.05;
            telemetry.addData("error", error);

            lDrive1.setPower(0+error);
            lDrive2.setPower(0+error);
            rDrive1.setPower(0-error);
            rDrive2.setPower(0-error);
            telemetry.update();
            if (rangef > 200 || rangeb > 200 || rangeb == -1 || rangef == -1)
                done = false;
            else {
                if (rangef >= rangeb+1 && rangeb >= rangef-1)
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

    private void redLineUp() throws InterruptedException {
        I2cDevice RANGE1 = hardwareMap.i2cDevice.get("bRS");
        I2cDeviceSynch RANGE1Reader = new I2cDeviceSynchImpl(RANGE1, RANGE1ADDRESS, false);
        RANGE1Reader.engage();
        I2cDevice RANGE2 = hardwareMap.i2cDevice.get("fRS");
        I2cDeviceSynch RANGE2Reader = new I2cDeviceSynchImpl(RANGE2, RANGE2ADDRESS, false);
        RANGE2Reader.engage();

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
            if (error > .15)
                error = .15;
            else if (error < .05 && error > 0)
                error = .05;
            else if (error < -.15)
                error = -.15;
            else if (error < 0 && error > -.05)
                error = -.05;
            telemetry.addData("error", error);

            lDrive1.setPower(0+error);
            lDrive2.setPower(0+error);
            rDrive1.setPower(0-error);
            rDrive2.setPower(0-error);
            telemetry.update();
            if (rangef > 200 || rangeb > 200 || rangeb == -1 || rangef == -1)
                done = false;
            else {
                if (rangef >= rangeb+1)
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

    private void driveTurn(double speed){
        lDrive1.setPower(-speed);
        lDrive2.setPower(-speed);
        rDrive1.setPower(speed);
        rDrive2.setPower(speed);
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

        lineUp();
    }
}