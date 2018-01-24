package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by Ian on 1/24/2018.
 */

public class DriveToLocation extends LinearOpMode {

    public DcMotor lfDriveM, lbDriveM, rfDriveM, rbDriveM;

    BNO055IMU imu;               // IMU Gyro sensor inside of REV Hub
    Orientation angles;          // variables of the IMU that get the rotation of the robot
    Acceleration gravity;

    ElapsedTime gyroTimer = new ElapsedTime();
    ElapsedTime gyroTimer2 = new ElapsedTime();

    int locationX;
    int locationY;
    int angularOffset = 0;

    private void useEncoders(){
        rfDriveM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rbDriveM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lfDriveM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lbDriveM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void resetEncoders(){
        lfDriveM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lbDriveM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rfDriveM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rbDriveM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    private void drive(double leftSpeed, double rightSpeed){
        lfDriveM.setPower(leftSpeed);
        lbDriveM.setPower(leftSpeed);
        rfDriveM.setPower(rightSpeed);
        rbDriveM.setPower(rightSpeed);
    }

    private void driveStop(){
        drive(0, 0);
    }

    private void encoderDrive(double distance, double speed, int direction) throws InterruptedException {
        int ENCODER_CPR = 1120; // Encoder counts per Revolution
        double gearRatio = 1.75; // [Gear Ratio]:1
        double circumference = 13.10; // Wheel circumference (in inches)
        double distanceToDrive = distance / (circumference * gearRatio); // Number of rotations to drive
        double COUNTS = ENCODER_CPR * distanceToDrive; // Number of encoder counts to drive

        rfDriveM.setTargetPosition(rfDriveM.getCurrentPosition() + (int) COUNTS);
        lfDriveM.setTargetPosition(lfDriveM.getCurrentPosition() + (int) COUNTS);

        if (direction == 1) {
            while ((rfDriveM.getCurrentPosition() < rfDriveM.getTargetPosition() - 5 ||
                    lfDriveM.getCurrentPosition() < lfDriveM.getTargetPosition() - 5) && opModeIsActive()) {
                drive(speed, speed);
                telemetry.addData("1. left speed", speed);
                telemetry.addData("2. right speed", speed);
                telemetry.addData("3. position", rfDriveM.getCurrentPosition());
                telemetry.addData("4. target", rfDriveM.getTargetPosition());
                updateTelemetry(telemetry);
            }
            driveStop();
        }
        else if (direction == -1) {
            while ((Math.abs(rfDriveM.getCurrentPosition()) < Math.abs(rfDriveM.getTargetPosition() - 5) ||
                    Math.abs(lfDriveM.getCurrentPosition()) < Math.abs(lfDriveM.getTargetPosition() - 5)) && opModeIsActive()) {
                drive(-speed, -speed);
                telemetry.addData("1. left speed", speed);
                telemetry.addData("2. right speed", speed);
                telemetry.addData("3. position", rfDriveM.getCurrentPosition());
                telemetry.addData("4. target", rfDriveM.getTargetPosition());
                updateTelemetry(telemetry);
            }
            driveStop();
        }
        else {
            telemetry.addLine("Invalid direction");
            telemetry.update();
            sleep(10000);
        }
    }

    /*
     * Update the IMU so the robot can turn with the gyro
     */

    public void updateIMU() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity = imu.getGravity();
    }

    public boolean shouldKeepTurning(double desiredHeading, double afterHeading, double currentHeading) {
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

    public void imuTurn(double degreesToTurn) {
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
        degreesToTurn = (degreesToTurn * 8.0F) / 9.0F;

        double targetHeading = -degreesToTurn - angles.firstAngle;
        double foreHeading = -angles.firstAngle;
        int currentMotorPosition = rfDriveM.getCurrentPosition();
        int previousMotorPosition;

        gyroTimer.reset();
        gyroTimer2.reset();

        if (targetHeading < angles.firstAngle) {
            while (targetHeading < angles.firstAngle && shouldKeepTurning(targetHeading, foreHeading, angles.firstAngle)) {
                updateIMU();
                drive(0.2, -0.2);
                telemetry.addData("Gyro", -angles.firstAngle);
                telemetry.addData("Target", targetHeading);
                telemetry.addData("Time to update", gyroTimer2.milliseconds());
                telemetry.update();
                foreHeading = -angles.firstAngle;
                if(gyroTimer.milliseconds() % 500 == 0) {       // Every 1/2 second that passes...
                    previousMotorPosition = currentMotorPosition;
                    currentMotorPosition = rfDriveM.getCurrentPosition();
                    if(Math.abs(currentMotorPosition) - Math.abs(previousMotorPosition) < 20 ||
                            Math.abs(currentMotorPosition) - Math.abs(previousMotorPosition) > -20) {
                        break;
                    }
                }
                if(isStopRequested()) {
                    break;
                }
                gyroTimer2.reset();
            }
        } else {
            while (targetHeading > angles.firstAngle && shouldKeepTurning(targetHeading, foreHeading, angles.firstAngle)) {
                updateIMU();
                drive(-0.2, 0.2);
                telemetry.addData("Gyro", angles.firstAngle);
                telemetry.addData("Target", targetHeading);
                telemetry.addData("Time to update", gyroTimer2.milliseconds());
                telemetry.update();
                foreHeading = angles.firstAngle;
                if(gyroTimer.milliseconds() % 500 == 0) {       // Every 1/2 second that passes...
                    previousMotorPosition = currentMotorPosition;
                    currentMotorPosition = rfDriveM.getCurrentPosition();
                    if(Math.abs(currentMotorPosition) - Math.abs(previousMotorPosition) < 20 ||
                            Math.abs(currentMotorPosition) - Math.abs(previousMotorPosition) > -20) {
                        break;
                    }
                }
                if(isStopRequested()) {
                    break;
                }
                gyroTimer2.reset();
            }
        }
        driveStop();
    }

    private void driveToLocation(int X, int Y, boolean resetHeading) throws InterruptedException {
        int differenceX = X - locationX;
        int differenceY = Y - locationY;
        double trigHeading;
        if(differenceX > 0 && differenceY > 0) {                        // If desired point is in the robot's 1st quadrant...
            trigHeading = Math.atan(Math.abs(differenceX) / Math.abs(differenceY)) + 90 + angularOffset;
        } else if(differenceX < 0 && differenceY > 0) {                 // 2nd quadrant...
            trigHeading = Math.atan(Math.abs(differenceX) / Math.abs(differenceY)) + angularOffset;
        } else if(differenceX < 0 && differenceY < 0) {                 // 3rd quadrant...
            trigHeading = Math.atan(Math.abs(differenceX) / Math.abs(differenceY)) + 270 + angularOffset;
        } else if(differenceX > 0 && differenceY < 0) {                 // 4th quadrant...
            trigHeading = Math.atan(Math.abs(differenceX) / Math.abs(differenceY)) + 180 + angularOffset;
        } else if(differenceX == 0 && differenceY > 0) {                // between the 1st & 2nd quadrants...
            trigHeading = 90 + angularOffset;
        } else if(differenceX == 0 && differenceY < 0) {                // between the 3rd & 4th quadrants...
            trigHeading = -90 + angularOffset;
        } else if(differenceY == 0 && differenceX > 0) {                // between the 1st & 4th quadrants...
            trigHeading = 180 + angularOffset;
        } else if(differenceY == 0 && differenceX < 0) {                // between the 2nd & 3rd quadrants...
            trigHeading = angularOffset;
        } else {                                // Anywhere else is where the robot already is.
            return;
        }
        double dist = Math.hypot(differenceX, differenceY);

        imuTurn(trigHeading);
        encoderDrive(dist, 0.25, -1);
        if(resetHeading) {
            imuTurn(-trigHeading);
        }
        locationX = X;
        locationY = Y;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        lfDriveM = hardwareMap.dcMotor.get("lfDriveM"); //Left front drive, Hub 1, port 2
        lbDriveM = hardwareMap.dcMotor.get("lbDriveM"); //Left back drive, Hub 1, port 3
        rfDriveM = hardwareMap.dcMotor.get("rfDriveM"); //Right front drive, Hub 1, port 1
        rbDriveM = hardwareMap.dcMotor.get("rbDriveM"); //Right back drive, Hub 1, port 0
        lfDriveM.setDirection(DcMotor.Direction.REVERSE);
        lbDriveM.setDirection(DcMotor.Direction.REVERSE);
        lfDriveM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lbDriveM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rfDriveM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rbDriveM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        resetEncoders();
        useEncoders();

        locationX = 120;
        locationY = 24;
        driveToLocation(84, 24, false);

    }
}
