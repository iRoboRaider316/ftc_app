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

@Autonomous(name="range_telemetry", group="LinearOpMode")
@Disabled

public class range_test extends LinearOpMode {


    I2cAddr RANGE1ADDRESS = new I2cAddr(0x14); //Default I2C address for MR Range (7-bit)
    I2cAddr RANGE2ADDRESS = new I2cAddr(0x18);
    private static final int RANGE1_REG_START = 0x04; //Register to start reading
    private static final int RANGE1_READ_LENGTH = 2; //Number of byte to read
    //2nd range
    private static final int RANGE2_REG_START = 0x04; //Register to start reading
    private static final int RANGE2_READ_LENGTH = 2; //Number of byte to read


    private void range() throws InterruptedException {
        I2cDevice RANGE1 = hardwareMap.i2cDevice.get("bRS");
        I2cDeviceSynch RANGE1Reader = new I2cDeviceSynchImpl(RANGE1, RANGE1ADDRESS, false);
        RANGE1Reader.engage();
        //I2cDevice RANGE2 = hardwareMap.i2cDevice.get("fRS");
        //I2cDeviceSynch RANGE2Reader = new I2cDeviceSynchImpl(RANGE2, RANGE2ADDRESS, false);
        //RANGE2Reader.engage();

        while (opModeIsActive()) {
            byte[] rangebCache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
            //byte[] rangefCache = RANGE2Reader.read(RANGE2_REG_START, RANGE2_READ_LENGTH);
            double rangeb = rangebCache[0];
            //double rangef = rangefCache[0];
            //telemetry.addData("Range value:", rangef);
            telemetry.addData("Range2 value:", rangeb);
            telemetry.update();
            }
        }


    public void runOpMode() throws InterruptedException {
        //##############Init##############

        waitForStart();

        range();

    }
}