package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Created by Ian on 11/2/2017.
 */

@Autonomous(name = "caluper_auto", group = "LinearOpMode")
//@Disabled
public class caluper_auto extends LinearOpMode {

    //============================VARIABLES + CONSTANTS=============================================

    public DcMotor lfDriveM,     // Left Front Drive
            rfDriveM,            // Left Back Drive
            lbDriveM,            // Right Front Drive
            rbDriveM,            // Right Back Drive
            liftM;               // Lift Glyph Grabber

    public Servo lArmS,          // Left Glyph Grabber Arm
            rArmS;               // Right Glyph Grabber Arm

    public CRServo jewelArmS;    // Jewel Bumper

    ColorSensor sensorColorFwd;     // Front Color Sensor on Jewel Bumper
    ColorSensor sensorColorBck;     // Back Color Sensor on Jewel Bumper

    private double lArmSGrasp = 0;
    private double rArmSGrasp = 1;
    private double lArmSRelease = 1;
    private double rArmSRelease = 0;

    DcMotorSimple.Direction FORWARD = DcMotorSimple.Direction.FORWARD;
    DcMotorSimple.Direction BACKWARD = DcMotorSimple.Direction.REVERSE;

    String stone = "";
    String stoneAndKey = "";

    VuforiaLocalizer vuforia;    // The Vuforia camera
    BNO055IMU imu;               // IMU Gyro sensor inside of REV Hub
    Orientation angles;          // variables of the IMU that get the rotation of the robot
    Acceleration gravity;

    // =======================================METHODS===============================================
    public void drive(double leftPower, double rightPower) {    // Turns on motors. The reason
        lfDriveM.setPower(leftPower);                            // there are two powers is so the
        lbDriveM.setPower(leftPower);                            // robot may be allowed to curve.
        rfDriveM.setPower(rightPower);
        rbDriveM.setPower(rightPower);
    }

    public void driveStop() {                                   // Stop robot motors
        drive(0, 0);
    }

    public void grabbers(double lPos, double rPos) {
        lArmS.setPosition(lPos);
        rArmS.setPosition(rPos);
        sleep(200);
    }

    public void jewelBumper(DcMotorSimple.Direction direction, long time) throws InterruptedException { // Move Jewel Bumper
        jewelArmS.setDirection(direction);                 // when pos is 0, jewel bumper is lowered
        jewelArmS.setPower(1);
        sleep(time);                               // when pos is 1, jewel bumper is raised
        jewelArmS.setPower(0);
    }

    public void bumpRedJewel(int direction) throws InterruptedException {  // Knock the red jewel off. For blue side only.
        jewelBumper(BACKWARD, 2700);                              // Lower jewel bumper
        telemetry.addData("Red Fwd", sensorColorFwd.red());
        telemetry.addData("Blue Fwd", sensorColorFwd.blue());
        telemetry.addData("Red Bck", sensorColorBck.red());
        telemetry.addData("Blue Bck", sensorColorBck.blue());
        telemetry.update();
        if (sensorColorFwd.red() > sensorColorFwd.blue() || sensorColorBck.blue() > sensorColorBck.red()) {       // Is detected jewel red?
            drive(0.25 * direction, 0.25 * direction);       // Drive to knock it off.
            sleep(500);
            driveStop();
            jewelBumper(FORWARD, 2700);                          // Raise Jewel Bumper
            if(stone == "blueLeft") {
                drive(-0.25 * direction, -0.25 * direction);
                sleep(200);
            }
        } else if (sensorColorBck.red() > sensorColorBck.blue() || sensorColorFwd.blue() > sensorColorFwd.red()) {// Is detected jewel blue?
            drive(-0.25 * direction, -0.25 * direction);   // Drive to knock off red jewel
            sleep(400);                                    // it's called indirect proof
            driveStop();
            jewelBumper(FORWARD, 2700);                          // Raise Jewel Bumper
            drive(0.5 * direction, 0.5 * direction);       // Drive back on stone (We won't need it later on)
            sleep(350);
            driveStop();
        } else {
            jewelBumper(FORWARD, 2700);
            drive(0.25 * direction, 0.25 * direction);       // Drive to knock it off.
            sleep(300);
            driveStop();
            sleep(500);
        }
    }

    public void bumpBlueJewel(int direction) throws InterruptedException {              // Knock the blue jewel off. For red side only.
        jewelBumper(BACKWARD, 2700);                               // Lower Jewel Bumper
        if (sensorColorBck.red() > sensorColorBck.blue() || sensorColorFwd.blue() > sensorColorFwd.red()) {        // Is detected jewel blue?
            drive(-0.25 * direction, -0.25 * direction);    // Drive to knock off blue jewel
            sleep(350);                                     // it's called indirect proof
            driveStop();
            jewelBumper(FORWARD, 2700);                           // Raise Jewel Bumper
            drive(0.5 * direction, 0.5 * direction);        // Drive back on stone (We won't need it later on)
            sleep(800);
            driveStop();
            sleep(5000);
        } else if (sensorColorFwd.red() > sensorColorFwd.blue() || sensorColorBck.blue() > sensorColorBck.red()) { // Is detected jewel red?
            drive(0.25 * direction, 0.25 * direction);      // Drive to knock it off.
            sleep(900);
            driveStop();
            jewelBumper(FORWARD, 2700);                           // Raise the Jewel Bumper
            sleep(5000);
        } else {
            jewelBumper(FORWARD, 2700);
            drive(0.25 * direction, 0.25 * direction);       // Drive to knock it off.
            sleep(600);
            driveStop();
            sleep(500);
        }
    }

    /*public void bumpJewel(int direction, String color) throws InterruptedException {  // Knock the jewel off
        jewelBumper(0, 2700);                              // Lower jewel bumper
        if(color == "red") {
            if(sensorColor.red() > sensorColor.blue()) {       // Is detected jewel red?
                drive(0.25 * direction, 0.25 * direction);       // Drive to knock it off.
                sleep(300);
                driveStop();
                jewelBumper(1, 2300);                          // Raise Jewel Bumper
            } else if(sensorColor.blue() > sensorColor.red()) {// Is detected jewel blue?
                drive(-0.25 * direction, -0.25 * direction);   // Drive to knock off red jewel
                sleep(500);                                    // it's called indirect proof
                driveStop();
                jewelBumper(1, 2300);                          // Raise Jewel Bumper
                drive(0.5 * direction, 0.5 * direction);       // Drive back on stone (We won't need it later on)
                sleep(600);
                driveStop();
            }
        } else if(color == "blue") {
            if(sensorColor.blue() > sensorColor.red()) {        // Is detected jewel blue?
                drive(-0.25 * direction, -0.25 * direction);    // Drive to knock off blue jewel
                sleep(300);                                     // it's called indirect proof
                driveStop();
                jewelBumper(1, 2300);                           // Raise Jewel Bumper
                drive(0.5 * direction, 0.5 * direction);        // Drive back on stone (We won't need it later on)
                sleep(600);
                driveStop();
            } else if(sensorColor.red() > sensorColor.blue()) { // Is detected jewel red?
                drive(0.25 * direction, 0.25 * direction);      // Drive to knock it off.
                sleep(600);
                driveStop();
                jewelBumper(1, 2300);                           // Raise the Jewel Bumper
            }
        }
    }*/

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

    public void placeGlyph(String StoneWithKey) {
        switch (StoneWithKey) {
            case "redLeftKeyLeft":
                drive(0.25, -0.25);
                sleep(300);
                drive(-0.25, -0.25);
                sleep(2000);
                driveStop();
                grabbers(lArmSRelease, rArmSRelease);
                drive(0.2, 0.2);
                sleep(500);
                break;
            case "redLeftKeyCenter":
                drive(-0.25, -0.25);
                sleep(2000);
                driveStop();
                grabbers(lArmSRelease, rArmSRelease);
                drive(0.2, 0.2);
                sleep(500);
                break;
            case "redLeftKeyRight":
                drive(-0.25, 0.25);
                sleep(300);
                drive(-0.25, -0.25);
                sleep(2000);
                driveStop();
                grabbers(lArmSRelease, rArmSRelease);
                drive(0.2, 0.2);
                sleep(500);
                break;
            case "redRightKeyLeft":
                drive(0.25, -0.25);
                sleep(500);
                drive(-0.25, -0.25);
                sleep(2000);
                driveStop();
                grabbers(lArmSRelease, rArmSRelease);
                drive(0.2, 0.2);
                sleep(500);
                break;
            case "redRightKeyCenter":
                drive(-0.25, -0.25);
                sleep(2000);
                driveStop();
                grabbers(lArmSRelease, rArmSRelease);
                drive(0.2, 0.2);
                sleep(500);
                break;
            case "redRightKeyRight":
                drive(-0.25, 0.25);
                sleep(500);
                drive(-0.25, -0.25);
                sleep(2000);
                driveStop();
                grabbers(lArmSRelease, rArmSRelease);
                drive(0.2, 0.2);
                sleep(500);
                break;
            case "blueRightKeyLeft":
                drive(0.25, -0.25);
                sleep(400);
                drive(-0.25, -0.25);
                sleep(2000);
                driveStop();
                grabbers(lArmSRelease, rArmSRelease);
                drive(0.2, 0.2);
                sleep(500);
                break;
            case "blueRightKeyCenter":
                drive(-0.25, 0.25);
                sleep(90);
                drive(-0.25, -0.25);
                sleep(2000);
                driveStop();
                grabbers(lArmSRelease, rArmSRelease);
                drive(0.2, 0.2);
                sleep(500);
                break;
            case "blueRightKeyRight":
                drive(-0.25, 0.25);
                sleep(470);
                drive(-0.25, -0.25);
                sleep(2000);
                driveStop();
                grabbers(lArmSRelease, rArmSRelease);
                drive(0.2, 0.2);
                sleep(500);
                break;
            case "blueLeftKeyLeft":
                drive(0.25, -0.25);
                sleep(450);
                drive(-0.25, -0.25);
                sleep(2000);
                driveStop();
                grabbers(lArmSRelease, rArmSRelease);
                drive(0.2, 0.2);
                sleep(500);
                break;
            case "blueLeftKeyCenter":
                drive(-0.25, -0.25);
                sleep(2000);
                driveStop();
                grabbers(lArmSRelease, rArmSRelease);
                drive(0.2, 0.2);
                sleep(500);
                break;
            case "blueLeftKeyRight":
                drive(-0.25, 0.25);
                sleep(450);
                drive(-0.25, -0.25);
                sleep(2000);
                driveStop();
                grabbers(lArmSRelease, rArmSRelease);
                drive(0.2, 0.2);
                sleep(500);
                break;
        }
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

        if (targetHeading > angles.firstAngle) {
            while (targetHeading > angles.firstAngle && shouldKeepTurning(targetHeading, foreHeading, angles.firstAngle)) {
                updateIMU();
                drive(0.2, -0.2);
                telemetry.addData("Gyro", angles.firstAngle);
                telemetry.addData("Target", targetHeading);
                telemetry.update();
                foreHeading = angles.firstAngle;
                if(isStopRequested()) {             // Found this one boolean in LinearOpMode
                    break;                          // that checks if STOP is hit.
                }                                   // Could help with the OpModeStuckInStop issues.
            }
        } else {
            while (targetHeading < angles.firstAngle && shouldKeepTurning(targetHeading, foreHeading, angles.firstAngle)) {
                updateIMU();
                drive(-0.2, 0.2);
                telemetry.addData("Gyro", angles.firstAngle);
                telemetry.addData("Target", targetHeading);
                telemetry.update();
                foreHeading = angles.firstAngle;
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
        liftM = hardwareMap.dcMotor.get("liftM"); //Lift motor, Hub 2, port 3

        lArmS = hardwareMap.servo.get("lArmS"); //Left servo arm, Hub 1, port 2
        rArmS = hardwareMap.servo.get("rArmS"); //Right servo arm, Hub 2, port 1
        jewelArmS = hardwareMap.crservo.get("jewelArmS");

        sensorColorFwd = hardwareMap.get(ColorSensor.class, "sensorColorFwd");
        sensorColorBck = hardwareMap.get(ColorSensor.class, "sensorColorBck");

        driveStop();
        liftM.setPower(0);
        lfDriveM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lbDriveM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rfDriveM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rbDriveM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lfDriveM.setDirection(DcMotor.Direction.REVERSE);
        lbDriveM.setDirection(DcMotor.Direction.REVERSE);
        lfDriveM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lbDriveM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rfDriveM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rbDriveM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lArmS.scaleRange(0, 0.8);
        rArmS.scaleRange(0.2, 1);
        grabbers(lArmSGrasp, rArmSGrasp);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()); // Get the camera!
        VuforiaLocalizer.Parameters parameters_Vuf = new VuforiaLocalizer.Parameters(cameraMonitorViewId);    // Prepare the parameters
        parameters_Vuf.vuforiaLicenseKey = "Ae3H91v/////AAAAGT+4TPU5r02VnQxesioVLr0qQzNtgdYskxP7aL6/" +       // This is long...
                "yt9VozCBUcQrSjwec5opfpOWEuc55kDXNNSRJjLAnjGPeaku9j4nOfe7tWxio/xj/uNdPX7fEHD0j5b" +
                "5M1OgX/bkWoUV6pUTAsKj4GaaAKIf76vnX36boqJ7BaMJNuhkYhoQJWdVqwFOC4veNcABzJRw4mQmfO" +
                "3dfPvNVjxDl8kgdBEQOZRi9kFDy9w3cTLatSGZne3IvyaYYd8uckzPnQb5Mgel3ORjar/84qO+GBmG2" +
                "vDhmiv+vkY4gbCtS0em5LM+7CIMuZa5vO9GmtqXyNsoCp9zpPlgZHc1OJ7javiI5jAzWEKCPjZcmLAkSs7k+amw";
        parameters_Vuf.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;                               // Look through the camera you use for selfies
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters_Vuf);                                   // Apply Parameters

        VuforiaTrackables Cryptokey = this.vuforia.loadTrackablesFromAsset("RelicVuMark");                    // Create VuMarks from Pictograph
        VuforiaTrackable Targets = Cryptokey.get(0);
        Targets.setName("Targets");

        BNO055IMU.Parameters parameters_IMU = new BNO055IMU.Parameters();
        parameters_IMU.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters_IMU.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters_IMU.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters_IMU.loggingEnabled = true;
        parameters_IMU.loggingTag = "IMU";
        parameters_IMU.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters_IMU);

        // =======================BEGIN SELECTION===================================================
        telemetry.addData("Selection", "X for Blue, B for Red");        // Which side are you on?
        telemetry.update();
        while (stone == "") {
            if (gamepad1.x) {
                stone = "blue";
            } else if (gamepad1.b) {
                stone = "red";
            }
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
        }

        switch(stone) {                                             // Display the input
            case "redLeft":
                telemetry.addData("Team", "Red");
                telemetry.addData("Stone", "Left");
                break;
            case "redRight":
                telemetry.addData("Team", "Red");
                telemetry.addData("Stone", "Right");
                break;
            case "blueLeft":
                telemetry.addData("Team", "Blue");
                telemetry.addData("Stone", "Left");
                break;
            case "blueRight":
                telemetry.addData("Team", "Blue");
                telemetry.addData("Stone", "Right");
                break;
        }

        telemetry.addData("Searching For Key", "...");
        telemetry.update();

        telemetry.update();
        Cryptokey.activate();
        decryptKey(Targets);
        waitForStart();
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        updateIMU();
        // =======================================AUTONOMOUS========================================
        switch(stone) {
            /*---------------------Red Left Autonomous--------------------------*/
            case "redLeft":
                bumpBlueJewel(1);                  // Bump Blue Jewel
                drive(0.25, 0.25);                 // Drive to Cryptobox...
                sleep(850);                        // ...for 0.85 seconds  (about 14")
                imuTurn(90);                       // Turn to Cryptobox!
                sleep(700);                        // wait 0.7 seconds just to see what is happening
                placeGlyph(stoneAndKey);           // Place Glyph in column depending on pictograph
                break;
            /*---------------------Red Right Autonomous--------------------------*/
            case "redRight":
                bumpBlueJewel(1);                  // Bump Blue Jewel
                drive(0.25, 0.25);                 // Drive to Cryptobox...
                sleep(120);                        // ...for 0.12 seconds  (about 2")
                imuTurn(-90);                      // Turn to Cryptobox!
                sleep(700);                        // wait 0.7 seconds just to see what is happening
                drive(-0.25, -0.25);               // Drive out in front of Cryptobox...
                sleep(730);                        // ...for 0.73 seconds  (about 12")
                imuTurn(-90);                      // Turn to Cryptobox!
                sleep(700);                        // wait 0.7 seconds just to see what is happening
                placeGlyph(stoneAndKey);           // Place Glyph in column depending on pictograph
                break;
            /*---------------------Blue Left Autonomous--------------------------*/
            case "blueLeft":
                bumpRedJewel(-1);                  // Bump Blue Jewel
                drive(-0.25, -0.25);               // Drive to Cryptobox...
                sleep(700);                        // ...for 0.7 seconds
                imuTurn(-90);                      // Turn to Cryptobox!
                sleep(700);                        // wait 0.7 seconds just to see what is happening
                drive(-0.25, -0.25);               // Drive out in front of Cryptobox...
                sleep(730);                        // ...for 0.73 seconds  (about 12")
                imuTurn(90);                       // Turn to Cryptobox!
                sleep(700);                        // wait 0.7 seconds just to see what is happening
                placeGlyph(stoneAndKey);           // Place Glyph in column depending on pictograph
                break;
            /*---------------------Blue Right Autonomous--------------------------*/
            case "blueRight":
                bumpRedJewel(-1);                  // Bump Red Jewel
                drive(-0.25, -0.25);               // Drive to Cryptobox...
                sleep(1100);                       // ...for 1.1 seconds
                imuTurn(90);                       // Turn to Cryptobox!
                sleep(700);                        // wait 0.7 seconds just to see what is happening
                placeGlyph(stoneAndKey);           // Place Glyph in column depending on pictograph
                break;
        }
        driveStop();
    }
}
// Please let Ian H. know if there is anything that needs to be fixed!