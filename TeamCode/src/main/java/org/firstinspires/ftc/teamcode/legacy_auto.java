package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by Ian on 1/23/2018.
 */
@Autonomous(name="legacy_auto", group="LinearOpMode")
//@Disabled
public class legacy_auto extends LinearOpMode {

    public DcMotor lfDriveM, lbDriveM, rfDriveM, rbDriveM, glyphLiftM;
    public Servo lGlyphS, rGlyphS, jewelExtendS, jewelKnockS;
    public CRServo glyphSlideS;

    private double lGlyphSInit = .39;               //Glyph arms will initialize in the open position.
    private double rGlyphSInit = .61;
    private double lGlyphSGrasp = 0;              //After testing, these positions were optimal for grasping the glyphs.
    private double rGlyphSGrasp = 1;

    DcMotorSimple.Direction FORWARD = DcMotorSimple.Direction.FORWARD;
    DcMotorSimple.Direction BACKWARD = DcMotorSimple.Direction.REVERSE;

    String stone = "";
    String stoneAndKey = "";

    int locationX = 0;
    int locationY = 0;
    int angularOffset = 0;

    ClosableVuforiaLocalizer vuforia;    // The Vuforia camera
    BNO055IMU imu;               // IMU Gyro sensor inside of REV Hub
    Orientation angles;          // variables of the IMU that get the rotation of the robot
    Acceleration gravity;

    ElapsedTime gyroTimer = new ElapsedTime();

    private void drive(double leftSpeed, double rightSpeed){
        lfDriveM.setPower(leftSpeed);
        lbDriveM.setPower(leftSpeed);
        rfDriveM.setPower(rightSpeed);
        rbDriveM.setPower(rightSpeed);
    }

    private void driveStop(){
        drive(0, 0);
    }

    public void grabbers(double lPos, double rPos) {
        lGlyphS.setPosition(lPos);
        rGlyphS.setPosition(rPos);
        sleep(200);
    }

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

    // This is the Drive Method
    private void encoderDrive(double distance, double speed, int direction) throws InterruptedException {
        int ENCODER_CPR = 1120; // Encoder counts per Revolution
        double gearRatio = 1.75; // [Gear Ratio]:1
        double circumference = 13.10; // Wheel circumference (in inches)
        double distanceToDrive = distance / (circumference * gearRatio); // Number of rotations to drive
        double COUNTS = ENCODER_CPR * distanceToDrive; // Number of encoder counts to drive

        rfDriveM.setTargetPosition(rfDriveM.getCurrentPosition() + (int) COUNTS);
        lfDriveM.setTargetPosition(lfDriveM.getCurrentPosition() + (int) COUNTS);

        if (direction == 1) {
            while (rfDriveM.getCurrentPosition() < rfDriveM.getTargetPosition() - 5 &&
                    lfDriveM.getCurrentPosition() < lfDriveM.getTargetPosition() - 5 && opModeIsActive()) {
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
            while (Math.abs(rfDriveM.getCurrentPosition()) < Math.abs(rfDriveM.getTargetPosition() - 5) &&
                    Math.abs(lfDriveM.getCurrentPosition()) < Math.abs(lfDriveM.getTargetPosition() - 5) && opModeIsActive()) {
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
         * always necessary, however, so we skip this operation in those cases */
        if(degreesToTurn - angles.firstAngle < -180) {
            degreesToTurn += 360;
        }

        if(degreesToTurn + angles.firstAngle > 180) {
            degreesToTurn -= 360;
        }

        /*
         * For us, the IMU has had us turn just a bit more than what we intend. The operation
         * below accounts for this by dividing the current degreesToTurn value by 8/9.
         */
        degreesToTurn *= (8.0F / 9.0F);

        double targetHeading = -degreesToTurn + -angles.firstAngle;
        double foreHeading = -angles.firstAngle;
        int currentMotorPosition = rfDriveM.getCurrentPosition();
        int previousMotorPosition;

        gyroTimer.reset();

        if (targetHeading > -angles.firstAngle) {
            while (targetHeading > -angles.firstAngle && shouldKeepTurning(targetHeading, foreHeading, -angles.firstAngle)) {
                updateIMU();
                drive(0.2, -0.2);
                telemetry.addData("Gyro", angles.thirdAngle);
                telemetry.addData("Target", targetHeading);
                telemetry.update();
                foreHeading = -angles.firstAngle;
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
            while (targetHeading < -angles.firstAngle && shouldKeepTurning(targetHeading, foreHeading, -angles.firstAngle)) {
                updateIMU();
                drive(-0.2, 0.2);
                telemetry.addData("Gyro", -angles.firstAngle);
                telemetry.addData("Target", targetHeading);
                telemetry.update();
                foreHeading = -angles.firstAngle;
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

    public void decryptKey(VuforiaTrackable cryptokeys) {
        RelicRecoveryVuMark vuMark;
        while (!isStarted()) {
            vuMark = RelicRecoveryVuMark.from(cryptokeys);

            if (vuMark == RelicRecoveryVuMark.LEFT) {
                stoneAndKey = stone + "KeyLeft";
                telemetry.addData("Spotted Key", "Left!");
                telemetry.update();
            } else if (vuMark == RelicRecoveryVuMark.CENTER) {
                stoneAndKey = stone + "KeyCenter";
                telemetry.addData("Spotted Key", "Center!");
                telemetry.update();
            } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
                stoneAndKey = stone + "KeyRight";
                telemetry.addData("Spotted Key", "Right!");
                telemetry.update();
            }

            if(isStopRequested()) {             // Found this one boolean in LinearOpMode
                break;                          // that checks if STOP is hit.
            }                                   // Could help with the OpModeStuckInStop issues.
        }
    }

    public void runOpMode() throws InterruptedException {
        lfDriveM = hardwareMap.dcMotor.get("lfDriveM");         // Drive Motors
        lbDriveM = hardwareMap.dcMotor.get("lbDriveM");
        rfDriveM = hardwareMap.dcMotor.get("rfDriveM");
        rbDriveM = hardwareMap.dcMotor.get("rbDriveM");
        glyphLiftM = hardwareMap.dcMotor.get("glyphLiftM");     // Glyph Lifter
        glyphSlideS = hardwareMap.crservo.get("glyphSlideS");   // Glyph Slide

        lGlyphS = hardwareMap.servo.get("lGlyphS");
        rGlyphS = hardwareMap.servo.get("rGlyphS");

        lGlyphS.scaleRange(0, 0.8);
        rGlyphS.scaleRange(0.2, 1);
        grabbers(lGlyphSGrasp, rGlyphSGrasp);

//        jewelExtendS = hardwareMap.servo.get("jewelExtendS");
//        jewelKnockS = hardwareMap.servo.get("jewelKnockS");

        resetEncoders();
        useEncoders();

        BNO055IMU.Parameters parameters_IMU = new BNO055IMU.Parameters();
        parameters_IMU.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters_IMU.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters_IMU.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters_IMU.loggingEnabled = true;
        parameters_IMU.loggingTag = "IMU";
        parameters_IMU.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters_IMU);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()); // Get the camera!
        VuforiaLocalizer.Parameters parameters_Vuf = new VuforiaLocalizer.Parameters(cameraMonitorViewId);    // Prepare the parameters
        parameters_Vuf.vuforiaLicenseKey = "Ae3H91v/////AAAAGT+4TPU5r02VnQxesioVLr0qQzNtgdYskxP7aL6/" +       // This is long...
                "yt9VozCBUcQrSjwec5opfpOWEuc55kDXNNSRJjLAnjGPeaku9j4nOfe7tWxio/xj/uNdPX7fEHD0j5b" +
                "5M1OgX/bkWoUV6pUTAsKj4GaaAKIf76vnX36boqJ7BaMJNuhkYhoQJWdVqwFOC4veNcABzJRw4mQmfO" +
                "3dfPvNVjxDl8kgdBEQOZRi9kFDy9w3cTLatSGZne3IvyaYYd8uckzPnQb5Mgel3ORjar/84qO+GBmG2" +
                "vDhmiv+vkY4gbCtS0em5LM+7CIMuZa5vO9GmtqXyNsoCp9zpPlgZHc1OJ7javiI5jAzWEKCPjZcmLAkSs7k+amw";
        parameters_Vuf.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;                               // Look through the camera you use for selfies
        this.vuforia = new ClosableVuforiaLocalizer(parameters_Vuf);                                   // Apply Parameters                               // Apply Parameters

        VuforiaTrackables Cryptokey = this.vuforia.loadTrackablesFromAsset("RelicVuMark");                    // Create VuMarks from Pictograph
        VuforiaTrackable Targets = Cryptokey.get(0);
        Targets.setName("Targets");

        //============================================== Selection =================================
        telemetry.addData("Selection", "X for Blue, B for Red");        // Which side are you on?
        telemetry.update();
        while (stone == "") {
            if (gamepad1.x) {
                stone = "blue";
            } else if (gamepad1.b) {
                stone = "red";
                angularOffset = 180;
            }
            if(isStopRequested()) {             // Found this one boolean in LinearOpMode
                break;                          // that checks if STOP is hit.
            }                                   // Could help with the OpModeStuckInStop issues.
        }
        sleep(500);
        telemetry.addData("Selection", "X for Left, B for Right");  // Which stone is the robot on?
        telemetry.update();
        while (stone == "blue" || stone == "red") {
            if (gamepad1.x) {
                stone += "Left";
            } else if (gamepad1.b) {
                stone += "Right";
            }
            if(isStopRequested()) {             // Found this one boolean in LinearOpMode
                break;                          // that checks if STOP is hit.
            }                                   // Could help with the OpModeStuckInStop issues.
        }

        switch(stone) {                                             // Display the input
            case "redLeft":
                telemetry.addData("Team", "Red");
                telemetry.addData("Stone", "Left");
                locationX = 120;
                locationY = 120;
                break;
            case "redRight":
                telemetry.addData("Team", "Red");
                telemetry.addData("Stone", "Right");
                locationX = 48;
                locationY = 120;
                break;
            case "blueLeft":
                telemetry.addData("Team", "Blue");
                telemetry.addData("Stone", "Left");
                locationX = 48;
                locationY = 24;
                break;
            case "blueRight":
                telemetry.addData("Team", "Blue");
                telemetry.addData("Stone", "Right");
                locationX = 120;
                locationY = 24;
                break;
        }

        Cryptokey.activate();
        decryptKey(Targets);
        waitForStart();


        locationX = 120;
        locationY = 24;
        driveToLocation(84, 24, false);

    }
}
