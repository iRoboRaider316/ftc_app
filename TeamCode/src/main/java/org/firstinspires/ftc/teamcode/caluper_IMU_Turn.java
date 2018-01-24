package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by Ian on 11/2/2017.
 */

@Autonomous(name="caluper_auto_IMU", group="LinearOpMode")
//@Disabled
public class caluper_IMU_Turn extends LinearOpMode {

    public DcMotor lfDriveM,     // Left Front Drive
                   rfDriveM,     // Left Back Drive
                   lbDriveM,     // Right Front Drive
                   rbDriveM;     // Right Back Drive

    boolean blue = false;
    boolean red  = false;
    boolean rightStone = false;
    boolean leftStone  = false;

    BNO055IMU imu;
    Orientation angles;
    Acceleration gravity;
    ElapsedTime gyroTimer = new ElapsedTime();

    public void drive(double leftPower, double rightPower) {    // Turns on motors. The reason
        lfDriveM.setPower(leftPower);                            // there are two powers is so the
        lbDriveM.setPower(leftPower);                            // robot may be allowed to curve.
        rfDriveM.setPower(rightPower);
        rbDriveM.setPower(rightPower);
    }

    public void driveStop() {                                   // Stop robot motors
        lfDriveM.setPower(0);
        lbDriveM.setPower(0);
        rfDriveM.setPower(0);
        rbDriveM.setPower(0);
    }

    /*
     * Update the IMU so the robot can turn with the gyro
     */

    public void updateIMU() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity = imu.getGravity();
    }

    public boolean shouldKeepTurning(float desiredHeading, float afterHeading, float currentHeading) {
        if(desiredHeading > 160) {
            if(currentHeading > afterHeading + 100) {
                return false;
            }
        } else if(desiredHeading < -160){
            if(afterHeading + 100 < currentHeading) {
                return false;
            }
        }
        return true;             // If we made it this far, yes we should keep turning
    }

    /*
     * Perform a turn with the gyro sensor. For degreesToTurn, positive is clockwise, and negative
     * is counterclockwise.
     * angles.firstAngle is used for currentRotation.
     */

    public void imuTurn(float degreesToTurn) {
        updateIMU();                                    // Update the IMU to see where we are,
        // rotation-wise.
        /*
         * These operations account for when the robot would cross the IMU rotation line, which
         * separates -180 from 180. Adding or subtracting the degreesToTurn by 360 here isn't
         * always necessary, however, so we skip this operation in those cases*/
        if(degreesToTurn + angles.firstAngle > 180) {
            degreesToTurn -= 360;
        }

        if(degreesToTurn - angles.firstAngle < -180) {
            degreesToTurn += 360;
        }

        /*
         * For us, the IMU has had us turn just a bit more than what we intend. The operation
         * below accounts for this by dividing the current degreesToTurn value by 8/9.
         */
        degreesToTurn *= (8.0F / 9.0F);

        float targetHeading = degreesToTurn + angles.firstAngle;
        float foreHeading = angles.firstAngle;
        int currentMotorPosition = rfDriveM.getCurrentPosition();
        int previousMotorPosition;

        gyroTimer.reset();

        if (targetHeading > angles.firstAngle) {
            while (targetHeading > angles.firstAngle && shouldKeepTurning(targetHeading, foreHeading, angles.firstAngle)) {
                updateIMU();
                drive(0.2, -0.2);
                telemetry.addData("Gyro", angles.firstAngle);
                telemetry.addData("Target", targetHeading);
                telemetry.update();
                foreHeading = angles.firstAngle;
                if(gyroTimer.milliseconds() % 500 == 0) {       // Every 1/2 second that passes...
                    previousMotorPosition = currentMotorPosition;
                    currentMotorPosition = rfDriveM.getCurrentPosition();
                    if(Math.abs(currentMotorPosition) + Math.abs(previousMotorPosition) < 20 ||
                       Math.abs(currentMotorPosition) + Math.abs(previousMotorPosition) > -20) {
                        break;
                    }
                }
                if(isStopRequested()) {
                    break;
                }

            }
        } else {
            while (targetHeading < angles.firstAngle && shouldKeepTurning(targetHeading, foreHeading, angles.firstAngle)) {
                updateIMU();
                drive(-0.2, 0.2);
                telemetry.addData("Gyro", angles.firstAngle);
                telemetry.addData("Target", targetHeading);
                telemetry.update();
                foreHeading = angles.firstAngle;
                if(gyroTimer.milliseconds() % 500 == 0) {       // Every 1/2 second that passes...
                    previousMotorPosition = currentMotorPosition;
                    currentMotorPosition = rfDriveM.getCurrentPosition();
                    if(Math.abs(currentMotorPosition) + Math.abs(previousMotorPosition) < 20 ||
                       Math.abs(currentMotorPosition) + Math.abs(previousMotorPosition) > -20) {
                        break;
                    }
                }
                if(isStopRequested()) {             // Found this one boolean in LinearOpMode
                    break;                          // that checks if STOP is hit.
                }                                   // Could help with the OpModeStuckInStop issues.
            }
        }
        driveStop();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // ===================================INIT==================================================
        lfDriveM = hardwareMap.dcMotor.get("lfDriveM"); //Left front drive, Hub 1, port 2
        lbDriveM = hardwareMap.dcMotor.get("lbDriveM"); //Left back drive, Hub 1, port 3
        rfDriveM = hardwareMap.dcMotor.get("rfDriveM"); //Right front drive, Hub 1, port 1
        rbDriveM = hardwareMap.dcMotor.get("rbDriveM"); //Right back drive, Hub 1, port 0

        driveStop();
        lfDriveM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lbDriveM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rfDriveM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rbDriveM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lfDriveM.setDirection(DcMotor.Direction.REVERSE);
        lbDriveM.setDirection(DcMotor.Direction.REVERSE);
        lfDriveM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lbDriveM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rfDriveM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rbDriveM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled      = true;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        waitForStart();

        // Start the logging of measured acceleration
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        // =======================================AUTONOMOUS========================================
        imuTurn(90);
    }
}
// Please let Ian H. know if there is anything that needs to be fixed!