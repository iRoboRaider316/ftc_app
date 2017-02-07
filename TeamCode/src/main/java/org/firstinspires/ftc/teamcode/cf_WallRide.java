package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;

/**
 * Created by James on 1/31/2017.
 */
@Autonomous(name="WallRide", group="LinearOpMode")
public class cf_WallRide extends LinearOpMode {
    private DcMotor lDrive1;
    private DcMotor lDrive2;
    private DcMotor rDrive1;
    private DcMotor rDrive2;
    OpticalDistanceSensor rODSensor;
    OpticalDistanceSensor lODSensor;
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


        sleep(300);

        //if the robot is too far from the wall
        if ((range2Cache [0] >= range1Cache [0])) {
            //turn while the robot is too far from the wall (+-1 error)
            lDrive1.setPower(-0.30);
            rDrive1.setPower(0.30);
            lDrive2.setPower(-0.20);
            rDrive2.setPower(0.2);
            while (!((range2Cache)[0] - (range1Cache[0]) == 0) && !((range2Cache)[0] - (range1Cache[0]) == 0) ) {
                range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
                range2Cache = RANGE2Reader.read(RANGE2_REG_START, RANGE2_READ_LENGTH);
                telemetry.addData("Range value:", (range1Cache[0] & 0xFF));
                telemetry.addData("Range2 value:", (range2Cache[0] & 0xFF));
                telemetry.update();
            }
            //Stop driving robot once the sensors are close enough
            lDrive1.setPower(0);
            rDrive1.setPower(0);
            lDrive2.setPower(0);
            rDrive2.setPower(0);



        }
        else if ((range2Cache[0] <= range1Cache[0])) {
            //turn while the robot is too far from the wall (+-1 error)
            lDrive1.setPower(0.30);
            rDrive1.setPower(-0.30);
            lDrive2.setPower(0.20);
            rDrive2.setPower(-0.2);
            while (!((range2Cache)[0] - (range1Cache[0]) == 0) && !((range2Cache)[0] - (range1Cache[0]) == 0) ) {
                range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
                range2Cache = RANGE2Reader.read(RANGE2_REG_START, RANGE2_READ_LENGTH);
                telemetry.addData("Range value:", (range1Cache[0] & 0xFF));
                telemetry.addData("Range2 value:", (range2Cache[0] & 0xFF));
                telemetry.update();
            }
            //Stop driving robot once the sensors are close enough
            lDrive1.setPower(0);
            rDrive1.setPower(0);
            lDrive2.setPower(0);
            rDrive2.setPower(0);

        }
    }





    private void wallRideSingle(double speed) throws InterruptedException {
        while (opModeIsActive()) {
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
            telemetry.update();
            double range = range1Cache[0];

            do {
                range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
                range2Cache = RANGE2Reader.read(RANGE2_REG_START, RANGE2_READ_LENGTH);

                do {
                    range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
                    lDrive1.setPower(speed);
                    rDrive1.setPower(speed);
                    lDrive2.setPower(speed);
                    rDrive2.setPower(speed);
                    telemetry.addData("range", range);
                    telemetry.addData("Range value:", (range1Cache[0] & 0xFF));
                    telemetry.update();
                }while (range == range1Cache[0] && range1Cache[0] < 200 && opModeIsActive());

                do {
                    range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
                    //turn away from wall
                    double resolveLT = (range - range1Cache[0]) / 30;
                    if (resolveLT > 0.15)
                        resolveLT = 0.15;
                    lDrive1.setPower(speed - resolveLT);
                    rDrive1.setPower(speed + resolveLT);
                    lDrive2.setPower(speed - resolveLT);
                    rDrive2.setPower(speed + resolveLT);
                    telemetry.addData("range",range);
                    telemetry.addData("Range value:", (range1Cache[0] & 0xFF));
                    telemetry.update();

                }while (range > range1Cache[0] && range1Cache[0] < 200 && opModeIsActive());

                do {
                    range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
                    //turn toward wall
                    double resolveRT = (range - range1Cache[0]) / -30;
                    if (resolveRT > 0.15)
                        resolveRT = 0.15;
                    lDrive1.setPower(speed + resolveRT);
                    rDrive1.setPower(speed - resolveRT);
                    lDrive2.setPower(speed + resolveRT);
                    rDrive2.setPower(speed - resolveRT);
                    telemetry.addData("range", range);
                    telemetry.addData("Range value:", (range1Cache[0] & 0xFF));
                    telemetry.update();
                }while (range < range1Cache[0] && range1Cache[0] < 200 && opModeIsActive());
                telemetry.addData("range",range);
                telemetry.addData("Range value:", (range1Cache[0] & 0xFF));
                telemetry.update();
            }  while (range1Cache[0] < 200 && opModeIsActive());

        }
    }


    private void wallRideSingleBack(double speed) throws InterruptedException {
        rODSensor.enableLed(true);
        lODSensor.enableLed(true);
        while (opModeIsActive() && rODSensor.getRawLightDetected()<0.12 && lODSensor.getRawLightDetected()<0.12) {
            telemetry.addData("rLight", rODSensor.getRawLightDetected());
            telemetry.addData("lLight", lODSensor.getRawLightDetected());
            telemetry.update();
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
            telemetry.update();
            double range = range2Cache[0];

            do {
                range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
                range2Cache = RANGE2Reader.read(RANGE2_REG_START, RANGE2_READ_LENGTH);
                telemetry.addData("rLight", rODSensor.getRawLightDetected());
                telemetry.addData("lLight", lODSensor.getRawLightDetected());
                telemetry.update();
                do {
                    range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
                    range2Cache = RANGE2Reader.read(RANGE2_REG_START, RANGE2_READ_LENGTH);
                    lDrive1.setPower((speed) * -1);
                    rDrive1.setPower((speed) * -1);
                    lDrive2.setPower((speed) * -1);
                    rDrive2.setPower((speed) * -1);
                    telemetry.addData("range", range);
                    telemetry.addData("Range value:", (range2Cache[0] & 0xFF));
                    telemetry.addData("rLight", rODSensor.getRawLightDetected());
                    telemetry.addData("lLight", lODSensor.getRawLightDetected());
                    telemetry.update();
                    telemetry.update();
                }while (range == range2Cache[0] && range2Cache[0] < 200 && opModeIsActive());

                do {
                    range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
                    range2Cache = RANGE2Reader.read(RANGE2_REG_START, RANGE2_READ_LENGTH);
                    //turn away from wall
                    double resolveLT = (range - range2Cache[0]) / 30;
                    if (resolveLT > 0.15)
                        resolveLT = 0.15;
                    lDrive1.setPower((speed - resolveLT) * -1);
                    rDrive1.setPower((speed + resolveLT) * -1);
                    lDrive2.setPower((speed - resolveLT) * -1);
                    rDrive2.setPower((speed + resolveLT) * -1);
                    telemetry.addData("range", range);
                    telemetry.addData("Range value:", (range1Cache[0] & 0xFF));
                    telemetry.addData("rLight", rODSensor.getRawLightDetected());
                    telemetry.addData("lLight", lODSensor.getRawLightDetected());
                    telemetry.update();
                    telemetry.update();

                }while (range > range2Cache[0] && range2Cache[0] < 200 && opModeIsActive());

                do {
                    range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
                    range2Cache = RANGE2Reader.read(RANGE2_REG_START, RANGE2_READ_LENGTH);
                    //turn toward wall
                    double resolveRT = (range - range2Cache[0]) / -30;
                    if (resolveRT > 0.15)
                        resolveRT = 0.15;
                    lDrive1.setPower((speed + resolveRT) * -1);
                    rDrive1.setPower((speed - resolveRT) * -1);
                    lDrive2.setPower((speed + resolveRT) * -1);
                    rDrive2.setPower((speed - resolveRT) * -1);
                    telemetry.addData("range", range);
                    telemetry.addData("Range value:", (range2Cache[0] & 0xFF));
                    telemetry.addData("rLight", rODSensor.getRawLightDetected());
                    telemetry.addData("lLight", lODSensor.getRawLightDetected());
                    telemetry.update();
                    telemetry.update();
                }while (range < range2Cache[0] && range2Cache[0] < 200 && opModeIsActive());
                telemetry.addData("range",range);
                telemetry.addData("Range value:", (range1Cache[0] & 0xFF));
                telemetry.update();
            }  while (range2Cache[0] < 200 && opModeIsActive());

        }
    }
    public void lineUp() throws InterruptedException {



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
        telemetry.update();

   if (range1Cache[0] > range2Cache[0]) {
       lDrive1.setPower(0.25);
       rDrive1.setPower(-0.25);
       lDrive2.setPower(0.25);
       rDrive2.setPower(-0.25);
       while (range1Cache[0] > range2Cache[0] && opModeIsActive()) {
           range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
           range2Cache = RANGE2Reader.read(RANGE2_REG_START, RANGE2_READ_LENGTH);
           telemetry.addData("Range value:", (range1Cache[0] & 0xFF));
           telemetry.addData("Range2 value:", (range2Cache[0] & 0xFF));
           if (range1Cache[0] < 200) {
               lDrive1.setPower(0);
               rDrive1.setPower(0);
               lDrive2.setPower(0);
               rDrive2.setPower(0);
               sleep(100);
               lDrive1.setPower(0.25);
               rDrive1.setPower(-0.25);
               lDrive2.setPower(0.25);
               rDrive2.setPower(-0.25);
           }
          else if (range2Cache[0] < 200) {
               lDrive1.setPower(0);
               rDrive1.setPower(0);
               lDrive2.setPower(0);
               rDrive2.setPower(0);
               sleep(100);
               lDrive1.setPower(0.25);
               rDrive1.setPower(-0.25);
               lDrive2.setPower(0.25);
               rDrive2.setPower(-0.25);
           }
       }
       lDrive1.setPower(0);
       rDrive1.setPower(0);
       lDrive2.setPower(0);
       rDrive2.setPower(0);
   }
        else if (range1Cache[0] < range2Cache[0]) {
            lDrive1.setPower(-0.25);
            rDrive1.setPower(0.25);
            lDrive2.setPower(-0.25);
            rDrive2.setPower(0.25);
       while (range1Cache[0] < range2Cache[0] && opModeIsActive()) {
           range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
           range2Cache = RANGE2Reader.read(RANGE2_REG_START, RANGE2_READ_LENGTH);
           telemetry.addData("Range value:", (range1Cache[0] & 0xFF));
           telemetry.addData("Range2 value:", (range2Cache[0] & 0xFF));
           if (range1Cache[0] < 200) {
               lDrive1.setPower(0);
               rDrive1.setPower(0);
               lDrive2.setPower(0);
               rDrive2.setPower(0);
               sleep(100);
               lDrive1.setPower(-0.25);
               rDrive1.setPower(0.25);
               lDrive2.setPower(-0.25);
               rDrive2.setPower(0.25);
           }
           else if (range2Cache[0] < 200) {
               lDrive1.setPower(0);
               rDrive1.setPower(0);
               lDrive2.setPower(0);
               rDrive2.setPower(0);
               sleep(100);
               lDrive1.setPower(-0.25);
               rDrive1.setPower(0.25);
               lDrive2.setPower(-0.25);
               rDrive2.setPower(0.25);
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
    public void driveToWall() throws InterruptedException {

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
        telemetry.update();
        lDrive1.setPower(-0.3);
        rDrive1.setPower(-0.3);
        lDrive2.setPower(-0.3);
        rDrive2.setPower(-0.3);
        sleep(1000);
        while (range2Cache[0] >= 15 && opModeIsActive()) {
            range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
            range2Cache = RANGE2Reader.read(RANGE2_REG_START, RANGE2_READ_LENGTH);
            telemetry.addData("Range value:", (range1Cache[0] & 0xFF));
            telemetry.addData("Range2 value:", (range2Cache[0] & 0xFF));

            if (range2Cache[0] <= 200) {
//                lDrive1.setPower(0);
//                rDrive1.setPower(0);
//                lDrive2.setPower(0);
//                rDrive2.setPower(0);
                sleep(1);
//                lDrive1.setPower(-0.3);
//                rDrive1.setPower(-0.3);
//                lDrive2.setPower(-0.3);
//                rDrive2.setPower(-0.3);
            }

        }
        lDrive1.setPower(0);
        rDrive1.setPower(0);
        lDrive2.setPower(0);
        rDrive2.setPower(0);


    }

    public void runOpMode() throws InterruptedException {
waitForStart();
        rDrive1 = hardwareMap.dcMotor.get("rDrive1");
        rDrive2 = hardwareMap.dcMotor.get("rDrive2");
        lDrive1 = hardwareMap.dcMotor.get("lDrive1");
        lDrive2 = hardwareMap.dcMotor.get("lDrive2");
        rODSensor = hardwareMap.opticalDistanceSensor.get("fOD");
        lODSensor = hardwareMap.opticalDistanceSensor.get("bOD");
        lDrive1.setDirection(DcMotor.Direction.REVERSE);
        lDrive2.setDirection(DcMotor.Direction.REVERSE);

        sleep(1000);



        //driveToWall();
        //lineUp();

      wallRideSingleBack(0.2);

      // lineUp();
    }
}
