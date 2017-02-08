package org.firstinspires.ftc.teamcode;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "Telemetry", group = "Telemetry")

public class telemetryData extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    byte[] range1Cache; //The read will return an array of bytes. They are stored in this variable
    byte[] range2Cache;


    I2cAddr RANGE1ADDRESS = new I2cAddr(0x14); //Default I2C address for MR Range (7-bit)
    I2cAddr RANGE2ADDRESS = new I2cAddr(0x18);


    //private I2cAddr RANGE1ADDRESS = new I2cAddr(0x28); //Default I2C address for MR Range (7-bit)
    private static final int RANGE1_REG_START = 0x04; //Register to start reading
    private static final int RANGE1_READ_LENGTH = 2; //Number of byte to read
    //2nd range
    private static final int RANGE2_REG_START = 0x04; //Register to start reading
    private static final int RANGE2_READ_LENGTH = 2; //Number of byte to read

    public I2cDevice RANGE1;
    public I2cDeviceSynch RANGE1Reader;


    public void runOpMode() throws InterruptedException {
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
        telemetry.update();

        RANGE1 = hardwareMap.i2cDevice.get("range");
        RANGE1Reader = new I2cDeviceSynchImpl(RANGE1, RANGE1ADDRESS, false);
        RANGE1Reader.engage();

        waitForStart();
        runtime.reset();



           do {
               range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
               range2Cache = RANGE2Reader.read(RANGE2_REG_START, RANGE2_READ_LENGTH);
               telemetry.addData("Ultra Sonic", range1Cache[0] & 0xFF);
               telemetry.addData("ODS", range1Cache[1] & 0xFF);
               telemetry.addData("Status", "Run Time: " + runtime.toString());
               telemetry.addData("Ultra Sonic2", range2Cache[0] & 0xFF);
               telemetry.addData("ODS2", range2Cache[1] & 0xFF);
               telemetry.addData("ODS2", "ODS2");
               telemetry.addData("ODS2", "ODS2");


               telemetry.update();

               idle();
           } while (range1Cache[0] < 200 || range2Cache[0] < 200 && opModeIsActive());

            idle();

        }
    }
