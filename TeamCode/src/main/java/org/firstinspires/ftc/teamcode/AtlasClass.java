package org.firstinspires.ftc.teamcode;

//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Arrays;

//import org.firstinspires.ftc.robotcore.external.Telemetry;


public class AtlasClass {
    DcMotor flipM, lbDrive, rfDrive, lfDrive, rbDrive, liftM, extendM, collectM;
    Servo deliveryS;

    BNO055IMU imu;               // IMU Gyro sensor inside of REV Hub
    Orientation angles;          // variables of the IMU that get the rotation of the robot


    AtlasClass(HardwareMap hardwareMap) throws InterruptedException {
        rbDrive = hardwareMap.dcMotor.get("rbDriveM");
        rbDrive.setPower(0);
        rbDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lbDrive = hardwareMap.dcMotor.get("lbDriveM");
        lbDrive.setPower(0);
        lbDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rfDrive = hardwareMap.dcMotor.get("rfDriveM");
        rfDrive.setPower(0);
        rfDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lfDrive = hardwareMap.dcMotor.get("lfDriveM");
        lfDrive.setPower(0);
        lfDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        flipM = hardwareMap.dcMotor.get("flipM");
        flipM.setPower(0);
        flipM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftM = hardwareMap.dcMotor.get("liftM");
        liftM.setPower(0);
        liftM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        extendM = hardwareMap.dcMotor.get("extendM");
        extendM.setPower(0);
        extendM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        collectM = hardwareMap.dcMotor.get("collectM");
        collectM.setPower(0);
        collectM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        deliveryS = hardwareMap.servo.get("deliveryS");
        deliveryS.setPosition(0);

        BNO055IMU.Parameters parameters_IMU = new BNO055IMU.Parameters();
        parameters_IMU.angleUnit = BNO055IMU.AngleUnit.DEGREES;             // Angle Unit? Degrees!
        parameters_IMU.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;// Advanced stuff that we don't worry about
        parameters_IMU.calibrationDataFile = "BNO055IMUCalibration.json";   // see the calibration sample opmode
        parameters_IMU.loggingEnabled = true;                               // Log the data? Yes
        parameters_IMU.loggingTag = "IMU";                                  // Pretty straightforward
        parameters_IMU.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator(); // Pretty straightforward
        imu = hardwareMap.get(BNO055IMU.class, "imu");           // Now we init the IMU
        imu.initialize(parameters_IMU);

    }

    // This method checks for
    public boolean shouldKeepTurning2(int[] listOfHeadings) {
        int[] headings = Arrays.copyOfRange(listOfHeadings, 18, 22);
        return Arrays.asList(headings).contains((int)-angles.firstAngle);
    }

    /**
     * Perform a turn with the IMU inside our REV Hub. For degreesToTurn, positive is clockwise,
     * and negative is counterclockwise. angles.firstAngle is used for currentRotation.
     * NOTE: angles.firstAngle is flipped on the number line because the REV Hub is upside-down
     */

    public void imuTurn2(double degreesToTurn, String direction) throws InterruptedException {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        /* For us, the IMU has had us turn just a bit more than what we intend. The operation
         * below accounts for this by dividing the current degreesToTurn value by 8/9.
         */
        degreesToTurn = (degreesToTurn * 8) / 9;

        // Now we define our variables
        int targetHeading = (int)degreesToTurn - (int)angles.firstAngle;

        /* These operations account for when the robot would cross the IMU rotation line, which
         * separates -180 from 180. Adding or subtracting the degreesToTurn by 360 here isn't
         * always necessary, however, so we skip this operation in those cases */
        targetHeading += targetHeading > 180 ? -360 :
                targetHeading < -180 ? 360 : 0;

        /* In case you don't know what this is (it isn't widely used in FTC Robot programs, I
         * don't think), it's called an Array, and they're used to store a list of variables in
         * Java programs.
         * This array stores 5 heading variables that are very close or equal to our target heading.
         * We will use them to determine if we are in range of where we want Legacy to turn
         */
        int[] headingList = new int[40];
        for(int i = -(headingList.length / 2); i < headingList.length / 2; i++) {
            headingList[i + 20] = targetHeading + i;
        }

        /* As you can probably see here, the values in an array are mutable, which works very well
         * in cases like this, where are target heading values might be above or below where our
         * IMU can read.
         */
        for(int i = 0; i < headingList.length; i++) {
            headingList[i] += headingList[i] < -180 ? 360 : headingList[i] > 180 ? -360 : 0;
        }

        // RIGHT TURN
        if (direction == "RIGHT") {
            while (shouldKeepTurning2(headingList)) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                if(Arrays.asList(headingList).contains((int)-angles.firstAngle)) {
                    rfDrive.setPower(-0.25);
                    rbDrive.setPower(-0.25);
                    lfDrive.setPower(0.25);
                    lbDrive.setPower(0.25);
                } else {
                    rfDrive.setPower(-0.4);
                    rbDrive.setPower(-0.4);
                    lfDrive.setPower(0.4);
                    lbDrive.setPower(0.4);
                }
            }
            // LEFT TURN
        } else {
            while (shouldKeepTurning2(headingList)) {
                angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
                if(Arrays.asList(headingList).contains((int)-angles.firstAngle)) {
                    rfDrive.setPower(0.25);
                    rbDrive.setPower(0.25);
                    lfDrive.setPower(-0.25);
                    lbDrive.setPower(-0.25);
                } else {
                    rfDrive.setPower(0.4);
                    rbDrive.setPower(0.4);
                    lfDrive.setPower(-0.4);
                    lbDrive.setPower(-0.4);
                }
            }
        }
        rfDrive.setPower(0);
        rbDrive.setPower(0);
        lfDrive.setPower(0);
        lbDrive.setPower(0);
    }


}