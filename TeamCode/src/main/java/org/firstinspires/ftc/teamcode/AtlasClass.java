package org.firstinspires.ftc.teamcode;

//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

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
        imu = hardwareMap.get(BNO055IMU.class, "imu3");           // Now we init the IMU
        imu.initialize(parameters_IMU);

    }

    /**
     * Perform a turn with the IMU inside our REV Hub. For degreesToTurn, positive is clockwise,
     * and negative is counterclockwise. angles.firstAngle is used for currentRotation.
     * NOTE: angles.firstAngle is flipped on the number line because the REV Hub is upside-down
     */

    public void imuTurn(double degreesToTurn, LinearOpMode op) throws InterruptedException {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currentHeading = -angles.firstAngle + 180;
        double targetHeading = degreesToTurn + currentHeading;

        targetHeading += targetHeading > 360 ? -360 :
                         targetHeading <   0 ?  360 : 0;

        while (Math.abs(degreesToTurn) > 2) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currentHeading = -angles.firstAngle + 180;
            degreesToTurn = targetHeading - currentHeading;

            double power = Range.clip(Math.signum(degreesToTurn) * (0.25 + (Math.abs(degreesToTurn) / 360)), -1, 1);
            op.telemetry.addData("DegreesToTurn", degreesToTurn);
            op.telemetry.addData("currentHeading", currentHeading);
            op.telemetry.addData("targetHeading", targetHeading);
            op.telemetry.addData("POWA", power);
            System.out.println("DegreesToTurn: " + degreesToTurn + " -- currentHeading: " + currentHeading + " -- POWA: " + power);
            op.telemetry.update();
            rfDrive.setPower(power);
            rbDrive.setPower(power);
            lfDrive.setPower(power);
            lbDrive.setPower(power);
            // LEFT TURN
        }
        rfDrive.setPower(0);
        rbDrive.setPower(0);
        lfDrive.setPower(0);
        lbDrive.setPower(0);
    }


}