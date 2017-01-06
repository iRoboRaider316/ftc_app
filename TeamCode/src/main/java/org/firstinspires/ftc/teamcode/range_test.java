/*
Modern Robotics Range Sensor Example
Created 9/8/2016 by Colton Mehlhoff of Modern Robotics using FTC SDK 2.x Beta
Reuse permitted with credit where credit is due

Configuration:
I2cDevice on an Interface Module named "range" at the default address of 0x28 (0x14 7-bit)

This program can be run without a battery and Power Destitution Module.

For more information, visit modernroboticsedu.com.
Support is available by emailing support@modernroboticsinc.com.
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Range2 Linear", group = "MRI")

public class range_test extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    byte[] range1Cache; //The read will return an array of bytes. They are stored in this variable

    I2cAddr RANGE1ADDRESS = new I2cAddr(0x28); //Default I2C address for MR Range (7-bit)
    I2cAddr RANGE2ADDRESS = new I2cAddr(0x18);


    //private I2cAddr RANGE1ADDRESS = new I2cAddr(0x28); //Default I2C address for MR Range (7-bit)
    private static final int RANGE1_REG_START = 0x28; //Register to start reading
    private static final int RANGE1_READ_LENGTH = 2; //Number of byte to read
    private ModernRoboticsI2cRangeSensor rangeSensor;
    //2nd range
    private static final int RANGE2_REG_START = 0x18; //Register to start reading
    private static final int RANGE2_READ_LENGTH = 2; //Number of byte to read
    private ModernRoboticsI2cRangeSensor rangeSensor2;

    public I2cDevice RANGE1;
    public I2cDeviceSynch RANGE1Reader;
    public I2cDevice RANGE2;
    public I2cDeviceSynch RANGE2Reader;
    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        RANGE1 = hardwareMap.i2cDevice.get("range");
        RANGE1Reader = new I2cDeviceSynchImpl(RANGE1, RANGE1ADDRESS, false);
        RANGE1Reader.engage();
        I2cDevice RANGE1 = hardwareMap.i2cDevice.get("range");
        I2cDeviceSynch RANGE1Reader = new I2cDeviceSynchImpl(RANGE1, RANGE1ADDRESS, false);


        RANGE2 = hardwareMap.i2cDevice.get("range");
        RANGE2Reader = new I2cDeviceSynchImpl(RANGE2, RANGE2ADDRESS, false);
        RANGE2Reader.engage();
        I2cDevice RANGE2 = hardwareMap.i2cDevice.get("range2");
        I2cDeviceSynch RANGE2Reader = new I2cDeviceSynchImpl(RANGE2, RANGE2ADDRESS, false);
        RANGE1Reader.engage();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            byte[] range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
            byte[] range2Cache = RANGE2Reader.read(RANGE2_REG_START, RANGE2_READ_LENGTH);

            telemetry.addData("Ultra Sonic", range1Cache[0] & 0xFF);
            telemetry.addData("ODS", range1Cache[1] & 0xFF);
            telemetry.addData("Ultra Sonic2", range2Cache[0] & 0xFF);
            telemetry.addData("ODS2", range2Cache[1] & 0xFF);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();

            idle();
        }
    }
}