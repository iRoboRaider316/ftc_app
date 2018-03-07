package org.firstinspires.ftc.teamcode;

/**
 * Created by Jake on 1/24/18.
 */

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.JewelDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@Autonomous(name = "legacy_auto_SuperGlyph", group = "LinearOpMode")

public class legacy_auto_X extends LinearOpMode {

    //============================VARIABLES + CONSTANTS=============================================

    public DcMotor lfDriveM, lbDriveM, rfDriveM, rbDriveM, glyphLiftM;
    public Servo lGlyphS, rGlyphS, jewelExtendS, jewelHitS;
    public CRServo glyphSlideS;
    public TouchSensor touch;

    private double extendArm = .55;
    private double retractArm = 0;

    private double hitCenter = .5;   //jewelHitS returns to center variable
    private double hitPLeft = .25;   //Protects arm from falling in case of power outages.
    private double hitLeft = 0;      //jewelHitS knock left jewel variable
    private double hitRight = 1;     //jewelHitS knocks right jewel variable

    private double lGlyphSRelease = 0.8;               //Glyph arms will initialize in the open position.
    private double rGlyphSRelease = 0;
    private double lGlyphSGrasp = 0;              //After testing, these positions were optimal for grasping the glyphs.
    private double rGlyphSGrasp = 1;
    private double lGlyphSAlmostGrasp = 0.5;
    private double rGlyphSAlmostGrasp = 0.6;

    DcMotorSimple.Direction FORWARD = DcMotorSimple.Direction.FORWARD;
    DcMotorSimple.Direction BACKWARD = DcMotorSimple.Direction.REVERSE;
    DcMotorSimple.Direction LEFT = DcMotorSimple.Direction.FORWARD;
    DcMotorSimple.Direction RIGHT = DcMotorSimple.Direction.REVERSE;

    ClosableVuforiaLocalizer vuforia;    // The Vuforia camera
    BNO055IMU imu;               // IMU Gyro sensor inside of REV Hub
    Orientation angles;          // variables of the IMU that get the rotation of the robot

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime detecttime = new ElapsedTime();
    private ElapsedTime gyroTimer = new ElapsedTime();
    private JewelDetector jewelDetector = null;

    private String alliance = "";
    private String jewelOrder = "";
    private String lastOrder = "";
    private String lastKey = "";
    private String vufKey = "";
    private String jewelBumpType = "";
    private String stone = "";
    private String getMoreGlyphs = "";

    String vuforiaLicenseKey = "Ae3H91v/////AAAAGT+4TPU5r02VnQxesioVLr0qQzNtgdYskxP7aL6/" +     // Yay, random Vuforia license key!
            "yt9VozCBUcQrSjwec5opfpOWEuc55kDXNNSRJjLAnjGPeaku9j4nOfe7tWxio/xj/uNdPX7fEHD0j5b" +
            "5M1OgX/bkWoUV6pUTAsKj4GaaAKIf76vnX36boqJ7BaMJNuhkYhoQJWdVqwFOC4veNcABzJRw4mQmfO" +
            "3dfPvNVjxDl8kgdBEQOZRi9kFDy9w3cTLatSGZne3IvyaYYd8uckzPnQb5Mgel3ORjar/84qO+GBmG2" +
            "vDhmiv+vkY4gbCtS0em5LM+7CIMuZa5vO9GmtqXyNsoCp9zpPlgZHc1OJ7javiI5jAzWEKCPjZcmLAkSs7k+amw";

    private ElapsedTime vutimer = new ElapsedTime();

    public String decryptKey(VuforiaTrackable cryptokeys) {
        String key = lastKey;
        vutimer.reset();
        RelicRecoveryVuMark vuMark;
        while (!isStarted()) {
            vuMark = RelicRecoveryVuMark.from(cryptokeys);

            if (vuMark == RelicRecoveryVuMark.LEFT) {       //Store which cryptokey is found and
                key = "KeyLeft";            //update the telemetry accordingly.
                telemetry.addData("Spotted Key", "Left!");
                telemetry.update();
            } else if (vuMark == RelicRecoveryVuMark.CENTER) {
                key = "KeyCenter";
                telemetry.addData("Spotted Key", "Center!");
                telemetry.update();
            } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
                key = "KeyRight";
                telemetry.addData("Spotted Key", "Right!");
                telemetry.update();
            } else  {
                key = "KeyUnknown";
                telemetry.addData("Spotted Key", "Unknown");
                telemetry.update();
            }
            if(isStopRequested() || key != "KeyUnknown" || vutimer.time() > 3) {
                break;           //Stop this code if a stop is requested, or if the VuMark has been
            }                    //found, or if more than 3 seconds have passed.
        }
        return key;
    }

    // =======================================METHODS===============================================

    public <T> T hardwareMapper7129(Class<? extends T> classType, String deviceName) {
        try {
            return hardwareMap.get(classType, deviceName);
        } catch (IllegalArgumentException exc) {
            telemetry.addData("WARNING", "failed to init " + deviceName + ", but moving on...");
            telemetry.update();
            sleep(2000);
            return null;
        }
    }

    private void drive(double leftSpeed, double rightSpeed) {
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
        sleep(500);
    }

    private void glyphLifter(String direction) {
        if(direction == "UP") {
            glyphLiftM.setPower(1);
        } else {
            glyphLiftM.setPower(-0.5);
        }
        sleep(400);
        glyphLiftM.setPower(0);
    }

    private void moveSliders(DcMotorSimple.Direction Dir, int time) {
        glyphSlideS.setDirection(Dir);
        glyphSlideS.setPower(1);
        sleep(time);
        glyphSlideS.setPower(0);
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

    private void initForTeleop () {

        lGlyphS.setPosition(0.65);
        rGlyphS.setPosition(0.4);

        jewelExtendS.setPosition(0);
        jewelHitS.setPosition(0.25);

    }

    // This is the Drive Method
    private void encoderDrive(double distance, double speed, int direction) throws InterruptedException {
        int ENCODER_CPR = 1120; // Encoder counts per Revolution
        double gearRatio = 1.75; // [Gear Ratio]:1
        double circumference = 13.10; // Wheel circumference (in inches)
        double distanceToDrive = distance / (circumference * gearRatio); // Number of rotations to drive
        double COUNTS = ENCODER_CPR * distanceToDrive; // Number of encoder counts to drive

        resetEncoders();
        useEncoders();

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
    }

    /** This method checks for when Legacy has reached its desired heading.
     *  It takes an array of integers that we create within the gyro turn code itself */
    public boolean shouldKeepTurning2(int[] listOfHeadings) {
        for(int heading : listOfHeadings) {
            if(heading == (int)-angles.firstAngle) {
                return false;
            }
        }

        return true;
    }

    /**
     * Perform a turn with the IMU inside our REV Hub. For degreesToTurn, positive is clockwise,
     * and negative is counterclockwise. angles.firstAngle is used for currentRotation.
     * NOTE: angles.firstAngle is flipped on the number line because the REV Hub is upside-down
     */

    public void imuTurn(int degreesToTurn, String direction) throws InterruptedException {
        updateIMU();

        /* For us, the IMU has had us turn just a bit more than what we intend. The operation
         * below accounts for this by dividing the current degreesToTurn value by 8/9.
         */
        degreesToTurn = (degreesToTurn * 8) / 9;

        // Now we define our variables
        int targetHeading = degreesToTurn - (int)angles.firstAngle;
        int currentMotorPosition = rfDriveM.getCurrentPosition();
        int previousMotorPosition;

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
        int[] headingList = {targetHeading - 2,
                             targetHeading - 1,
                             targetHeading,
                             targetHeading + 1,
                             targetHeading + 2};

        /* As you can probably see here, the values in an array are mutable, which works very well
         * in cases like this, where are target heading values might be above or below where our
         * IMU can read.
         */
        for(int i = 0; i < headingList.length; i++) {
            headingList[i] += headingList[i] < -180 ? 360 :
                              headingList[i] > 180 ? -360 : 0;
        }

        // Finally, we reset the Gyro timer and begin our turn!
        gyroTimer.reset();

        // RIGHT TURN
        if (direction == "RIGHT") {
            while (shouldKeepTurning2(headingList)) {
                updateIMU();
                drive(-0.23, 0.23);
                telemetry.addData("Gyro", -angles.firstAngle);
                telemetry.addData("Target", targetHeading);
                telemetry.update();
                if(gyroTimer.milliseconds() % 500 == 0) {       // Every 1/2 second that passes...
                    previousMotorPosition = currentMotorPosition;
                    currentMotorPosition = rfDriveM.getCurrentPosition();
                    if (Math.abs(currentMotorPosition) + Math.abs(previousMotorPosition) < 20 ||
                            Math.abs(currentMotorPosition) + Math.abs(previousMotorPosition) > -20) {
                        telemetry.addData("Status", "Rammed!");
                        telemetry.update();
                        driveStop();
                        sleep(1000);
                        break;
                    }
                }
                if(isStopRequested()) {             // Break if we hit stop
                    break;
                }
            }
        // LEFT TURN
        } else {
            while (shouldKeepTurning2(headingList)) {
                updateIMU();
                drive(0.23, -0.23);
                telemetry.addData("Gyro", -angles.firstAngle);
                telemetry.addData("Target", targetHeading);
                telemetry.update();
                if (gyroTimer.milliseconds() % 500 == 0) {       // Every 1/2 second that passes...
                    previousMotorPosition = currentMotorPosition;
                    currentMotorPosition = rfDriveM.getCurrentPosition();
                    if (Math.abs(currentMotorPosition) + Math.abs(previousMotorPosition) < 20 ||
                            Math.abs(currentMotorPosition) + Math.abs(previousMotorPosition) > -20) {
                        telemetry.addData("Status", "Rammed!");
                        telemetry.update();
                        driveStop();
                        sleep(1000);
                        break;
                    }
                }
                if (isStopRequested()) {
                    break;
                }
            }
        }
        driveStop();
    }

    // Used to get off stone. Should be easy fix...
    public void driveOffStone(String Alliance) throws InterruptedException {
        if(alliance == "red") {
            encoderDrive(24, 0.23, 1);
            sleep(200);
        } else if(alliance == "blue") {
            encoderDrive(-25, 0.23, -1);
        }
    }

    private void deliverGlyph (String alliance, String stone, String cryptoKey) throws InterruptedException {
        switch (alliance + stone) {         //Switch case with all four balancing stones. Ian's code should
            case ("blueleft") :             //fit in with the blueright and redleft switch cases.
                switch (cryptoKey) {
                    default : case ("KeyCenter") :
                        imuTurn(35, "RIGHT");
                        driveStop();
                        encoderDrive(-13, 0.23, -1);
                        sleep(400);
                        glyphLifter("DOWN");
                        grabbers(lGlyphSAlmostGrasp, rGlyphSAlmostGrasp);
                        break;
                    case ("KeyLeft") :
                        imuTurn(15, "RIGHT");
                        driveStop();
                        encoderDrive(-10, 0.23, -1);
                        sleep(400);
                        glyphLifter("DOWN");
                        grabbers(lGlyphSAlmostGrasp, rGlyphSAlmostGrasp);
                        break;
                    case ("KeyRight") :
                        imuTurn(46, "RIGHT");
                        driveStop();
                        encoderDrive(-17, 0.23, -1);
                        sleep(400);
                        glyphLifter("DOWN");
                        grabbers(lGlyphSAlmostGrasp, rGlyphSAlmostGrasp);
                        break;
                }
                pushGlyph();
                break;
            case ("blueright") :
                switch (cryptoKey) {
                    case ("KeyLeft") :
                        encoderDrive(-4, 0.23, -1);                         // Drive in front of desired cryptobox
                        imuTurn(-90, "LEFT");                               // Turn to desired cryptobox
                        sleep(200);                                         // Just a sleep to keep Legacy from accidentally shifting left
                        encoderDrive(-7, 0.23, -1);                         // Drive into column
                        grabbers(lGlyphSAlmostGrasp, rGlyphSAlmostGrasp);   // "Dropping Glyph..."
                        sleep(200);
                        encoderDrive(7, 0.23, 1);                            // Drive backward
                        break;
                    default: case ("KeyCenter") :
                        encoderDrive(-12, 0.23, -1);
                        imuTurn(-90, "LEFT");
                        sleep(200);
                        encoderDrive(-7, 0.23, -1);
                        grabbers(lGlyphSAlmostGrasp, rGlyphSAlmostGrasp);
                        sleep(200);
                        encoderDrive(7, 0.23, 1);
                        break;
                    case ("KeyRight") :
                        encoderDrive(-17, 0.23, -1);
                        imuTurn(-90, "LEFT");
                        sleep(200);
                        encoderDrive(-7, 0.23, -1);
                        grabbers(lGlyphSAlmostGrasp, rGlyphSAlmostGrasp);
                        sleep(200);
                        encoderDrive(7, 0.23, 1);
                        break;
                }
                break;
            case ("redleft") :
                switch (cryptoKey) {
                    case ("KeyLeft") :
                        encoderDrive(19, 0.23, 1);
                        imuTurn(-90, "LEFT");
                        sleep(200);
                        encoderDrive(-7, 0.23, -1);
                        grabbers(lGlyphSAlmostGrasp, rGlyphSAlmostGrasp);
                        sleep(200);
                        encoderDrive(7, 0.23, 1);
                        break;
                    default: case ("KeyCenter") :
                        encoderDrive(10, 0.23, 1);
                        imuTurn(-90, "LEFT");
                        sleep(200);
                        encoderDrive(-7, 0.23, -1);
                        grabbers(lGlyphSAlmostGrasp, rGlyphSAlmostGrasp);
                        sleep(200);
                        encoderDrive(7, 0.23, 1);
                        break;
                    case ("KeyRight") :
                        encoderDrive(2.5, 0.23, 1);
                        imuTurn(-90, "LEFT");
                        sleep(200);
                        encoderDrive(-7, 0.23, -1);
                        grabbers(lGlyphSAlmostGrasp, rGlyphSAlmostGrasp);
                        sleep(200);
                        encoderDrive(7, 0.23, 1);
                        break;
                }
                break;
            case ("redright") :
                switch (cryptoKey) {
                    default : case ("KeyCenter") :
                        imuTurn(145, "RIGHT");
                        driveStop();
                        encoderDrive(-13, 0.23, -1);
                        sleep(400);
                        glyphLifter("DOWN");
                        grabbers(lGlyphSAlmostGrasp, rGlyphSAlmostGrasp);
                        sleep(200);
                        break;
                    case ("KeyLeft") :
                        imuTurn(134, "RIGHT");
                        driveStop();
                        encoderDrive(-17, 0.23, -1);
                        sleep(400);
                        glyphLifter("DOWN");
                        grabbers(lGlyphSAlmostGrasp, rGlyphSAlmostGrasp);
                        sleep(200);
                        break;
                    case ("KeyRight") :
                        imuTurn(170, "RIGHT");
                        driveStop();
                        encoderDrive(-10, 0.23, -1);
                        sleep(400);
                        glyphLifter("DOWN");
                        grabbers(lGlyphSAlmostGrasp, rGlyphSAlmostGrasp);
                        sleep(200);
                        break;
                }
                pushGlyph();
                break;
        }
    }

    // Used to get a brand new glyph load from the center pile.
    public void deliverExtraGlyph(String Key) throws InterruptedException {
        switch(alliance + stone) {                      // Prep for diving into the pile
            case "redright":                            // Different turns depending on where we
                switch (Key) {                          // are on the field
                    case "KeyLeft":
                        imuTurn(-80, "LEFT");
                        break;
                    default : case "KeyCenter":
                        imuTurn(-90, "LEFT");
                        break;
                    case "KeyRight":
                        imuTurn(-120, "LEFT");
                        break;
                }
                break;
            case "blueleft":
                switch (Key) {
                    case "KeyLeft":
                        imuTurn(120, "RIGHT");
                        break;
                    default : case "KeyCenter":
                        imuTurn(90, "RIGHT");
                        break;
                    case "KeyRight":
                        imuTurn(80, "RIGHT");
                        break;
                }
                break;
            default:
                imuTurn(-180, "LEFT");
                break;
        }

        glyphLifter("DOWN");
        grabbers(lGlyphSAlmostGrasp, rGlyphSRelease);
        sleep(200);
        encoderDrive(-31, 0.4, -1);                         // CHAAAAAAAAAAAARRGE!!!!
        moveSliders(RIGHT, 1000);                           // Set up left side of glyph
        encoderDrive(2, 0.3, 1);                            // Prep for setting up other side
        grabbers(lGlyphSRelease, rGlyphSAlmostGrasp);
        moveSliders(RIGHT, 300);
        encoderDrive(-2, 0.3, -1);
        moveSliders(LEFT, 1000);                            // Set up right side of glyph
        grabbers(lGlyphSGrasp, rGlyphSGrasp);               // GRAB!!!
        glyphLifter("UP");                                  // lift it up...
        glyphLifter("UP");                                  // more...
        encoderDrive(23, 0.23, 1);                          // good! Now back up
        switch (Key) {                                      // Turn to cryptobox
            case "KeyLeft":
                imuTurn(-180, "LEFT");
                break;
            default : case "KeyCenter":
                imuTurn(175, "RIGHT");
                break;
            case "KeyRight":
                imuTurn(165, "RIGHT");
                break;
        }
        sleep(100);
        drive(-0.4, -0.4);                                  // Drive into cryptobox
        sleep(800);
        driveStop();
        glyphLifter("DOWN");                                // set glyph load down...
        grabbers(lGlyphSAlmostGrasp, rGlyphSAlmostGrasp);   // RELEASE!!!
        encoderDrive(4, 0.4, 1);                            // Back up and end method
    }

    private void bumpJewelShort(String alliance, String jewel) throws InterruptedException {
        double knockVal =
                    jewelBumpType == "correct" ? (
                    ((alliance == "blue" && jewel == "RED_BLUE") || (alliance == "red"  && jewel == "BLUE_RED")) ? 0 :
                    ((alliance == "red"  && jewel == "RED_BLUE") || (alliance == "blue" && jewel == "BLUE_RED")) ? 1 : 0.5) :
                    jewelBumpType == "wrong" ? (
                    ((alliance == "blue" && jewel == "BLUE_RED") || (alliance == "red"  && jewel == "RED_BLUE")) ? 0 :
                    ((alliance == "red"  && jewel == "BLUE_RED") || (alliance == "blue" && jewel == "RED_BLUE")) ? 1 : 0.5) : 0.5;
        if(knockVal != 0.5) {
            jewelExtendS.setPosition(extendArm);
            sleep(750);
            jewelHitS.setPosition(knockVal);
            sleep(500);
            jewelHitS.setPosition(hitCenter);
            sleep(500);
            jewelExtendS.setPosition(retractArm);
            sleep(500);
        } else {
            telemetry.addData("Problem with alliance or jewelOrder", "    ;-;");
            telemetry.update();
            sleep(3000);
        }
    }

    private String activateVuforia () throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()); // Get the camera!
        VuforiaLocalizer.Parameters parameters_Vuf = new VuforiaLocalizer.Parameters(cameraMonitorViewId);  // Prepare the parameters
        parameters_Vuf.vuforiaLicenseKey = vuforiaLicenseKey;
        parameters_Vuf.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;  // Look through the inward facing camera
        this.vuforia = new ClosableVuforiaLocalizer(parameters_Vuf);             // Apply Parameters

        VuforiaTrackables Cryptokey = this.vuforia.loadTrackablesFromAsset("RelicVuMark");  // Create VuMarks from Pictograph
        VuforiaTrackable Targets = Cryptokey.get(0);
        Targets.setName("Targets");

        Cryptokey.activate();
        String key = decryptKey(Targets);

        this.vuforia.close();
        return key;
    }

    private String activateDogeCV () throws InterruptedException {
        jewelDetector = new JewelDetector();
        jewelDetector.init(hardwareMap.appContext, CameraViewDisplay.getInstance());

        //Jewel Detector Settings (using recommendations from creators of DogeCV)
        jewelDetector.areaWeight = 0.005;
        jewelDetector.detectionMode = JewelDetector.JewelDetectionMode.MAX_AREA;
        jewelDetector.debugContours = true;
        jewelDetector.maxDiffrence = 15;
        jewelDetector.ratioWeight = 30;
        jewelDetector.minArea = 700;

        jewelDetector.enable();

        while(jewelOrder == "" || jewelOrder == "UNKNOWN") {
            jewelOrder = jewelDetector.getLastOrder().toString(); //Store the jewel order for use later on.
        }

        jewelDetector.disable(); //Once we have the jewel order, disable OpenCV so we can activate Vuforia.
        jewelDetector = null;
        return jewelOrder;
    }

    private void pushGlyph () throws InterruptedException {
        drive(0.23, 0.23);                      //This function pushes the glyph inside the cryptobox once
        sleep(600);                             //it has already been delivered.
        driveStop();
        grabbers(lGlyphSGrasp, rGlyphSGrasp);
        drive(-0.23, -0.23);
        sleep(600);
        driveStop();
        drive(0.23, 0.23);
        sleep(600);
        driveStop();
    }

    @Override
    public void runOpMode() throws InterruptedException {

        // ===================================INIT==================================================

        //Drive motors
        lfDriveM = hardwareMapper7129(DcMotor.class, "lfDriveM"); //Hub 3 Port 2
        lfDriveM.setPower(0);
        lbDriveM = hardwareMapper7129(DcMotor.class, "lbDriveM"); //Hub 3 Port 3
        lbDriveM.setPower(0);
        rfDriveM = hardwareMapper7129(DcMotor.class, "rfDriveM"); //Hub 3 Port 0
        rfDriveM.setPower(0);
        rbDriveM = hardwareMapper7129(DcMotor.class, "rbDriveM"); //Hub 3 Port 1
        rbDriveM.setPower(0);

        //Lifting glypher motor
        glyphLiftM = hardwareMapper7129(DcMotor.class, "glyphLiftM"); //Hub 2 Port 0
        glyphLiftM.setPower(0);
        glyphLiftM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Glypher left-to-right motor
        glyphSlideS = hardwareMapper7129(CRServo.class, "glyphSlideS"); //Hub 3 Servo 1
        glyphSlideS.setPower(0);

        // Touch sensor on grabber
        touch = hardwareMapper7129(TouchSensor.class, "touch");

        lGlyphS = hardwareMapper7129(Servo.class, "lGlyphS"); //Hub 3 Servo 3
        rGlyphS = hardwareMapper7129(Servo.class, "rGlyphS"); //Hub 3 Servo 5
        grabbers(lGlyphSRelease, rGlyphSRelease);

        // Jewel Knocker
        jewelExtendS = hardwareMapper7129(Servo.class, "jewelExtendS"); //Hub 3 Servo 0
        jewelExtendS.setPosition(retractArm);
        jewelHitS = hardwareMapper7129(Servo.class, "jewelHitS"); //Hub 2 Servo 4
        jewelHitS.setPosition(hitCenter);

        rfDriveM.setDirection(DcMotor.Direction.REVERSE);       //Reverse the left side of the drive
        rbDriveM.setDirection(DcMotor.Direction.REVERSE);       //train for intuitive human interface
        lfDriveM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lbDriveM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rfDriveM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rbDriveM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters parameters_IMU = new BNO055IMU.Parameters();
        parameters_IMU.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters_IMU.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters_IMU.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters_IMU.loggingEnabled = true;
        parameters_IMU.loggingTag = "IMU";
        parameters_IMU.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMapper7129(BNO055IMU.class, "imu");
        imu.initialize(parameters_IMU);

        telemetry.addData("Status", "Initialized");

        // =======================BEGIN SELECTION===================================================
        telemetry.addData("Selection", "X for Blue, B for Red");        // Which alliance are you on?
        telemetry.update();
        while (alliance == "" && !isStopRequested()) {
            if (gamepad1.x) {
                alliance = "blue";
            } else if (gamepad1.b) {
                alliance = "red";
            }
            if (isStopRequested()) {
                break;
            }
        }

        sleep(500);
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Selection", "X for Left Stone, B for Right Stone");        // Which side are you on?
        telemetry.update();
        while (stone == "" && !isStopRequested()) {
            if (gamepad1.x) {
                stone = "left";
            } else if (gamepad1.b) {
                stone = "right";
            }
            if (isStopRequested()) {
                break;
            }
        }

        sleep(500);
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Selection", "X for Extra Glyph Load, B for no Extra Load");        // Do we want an extra glyph load?
        telemetry.update();                                                                   // Only for Blue Right or Red Left stone
        if((alliance == "blue" && stone == "right") ||
           (alliance == "red" && stone == "left")) {
            while (getMoreGlyphs == "") {
                if (gamepad1.x) {
                    getMoreGlyphs = "Yes";
                } else if (gamepad1.b) {
                    getMoreGlyphs = "No";
                }
                if (isStopRequested()) {
                    break;
                }
            }
        }

        sleep(500);
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Selection", "X for Correct Jewel, B for Wrong Jewel, Y for No Jewel");        // Which side are you on?
        telemetry.update();
        while (jewelBumpType == "" && !isStopRequested()) {
            if (gamepad1.x) {
                jewelBumpType = "correct";
            } else if (gamepad1.b) {
                jewelBumpType = "wrong";
            } else if(gamepad1.y) {
                jewelBumpType = "none";
            }
            if (isStopRequested()) {
                break;
            }
        }

        while(!gamepad1.a && !isStopRequested()) {
            switch(alliance + stone) {
                case "redleft":
                    telemetry.addData("Alliance", "Red");
                    telemetry.addData("Stone", "Left");
                    break;
                case "redright":
                    telemetry.addData("Alliance", "Red");
                    telemetry.addData("Stone", "Right");
                    break;
                case "blueleft":
                    telemetry.addData("Alliance", "Blue");
                    telemetry.addData("Stone", "Left");
                    break;
                case "blueright":
                    telemetry.addData("Alliance", "Blue");
                    telemetry.addData("Stone", "Right");
                    break;
            }
            telemetry.addData("Jewel Hit Type", jewelBumpType);
            telemetry.addData("Extra Glyph Load?", getMoreGlyphs);
            telemetry.addData("Press A if this is ok", "");
            telemetry.update();
            if (isStopRequested()) {
                break;
            }
        }

        telemetry.addData("Getting Jewel Order and Cryptokey...   ", "");
        telemetry.update();

        String jewels = "";
        while (!isStarted()) { //Track the jewel order and Cryptokey until init ends.
            if(detecttime.seconds() > 5) {
                telemetry.addData("Updating Jewel Order and Cryptokey...   ", "");
                telemetry.update();
                jewels = activateDogeCV();
                vufKey = activateVuforia();           //Look for the cryptokey
                telemetry.addData("Vuforia Image", vufKey);
                telemetry.addData("Jewel Order", jewels);
                telemetry.update();
                detecttime.reset();
            }
            if(isStopRequested()) {
                break;
            }
        }

        waitForStart();

//  ====================================== AUTONOMOUS ==============================================

        /*if(jewelBumpType != "none") {                   //Detect jewel
            bumpJewelShort(alliance, jewels);
        }*/
        sleep(2000);
        telemetry.addData("Cryptokey", vufKey);
        telemetry.update();
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        grabbers(lGlyphSGrasp, rGlyphSGrasp);           //Grab the glyph in front of the robot.
        glyphLifter("UP");                              //Lift the glypher slightly.

        driveOffStone(alliance);                        //Drive off the balancing stone.
        deliverGlyph(alliance, stone, vufKey);          //Deliver the glyph.

        if(getMoreGlyphs == "Yes") {
            deliverExtraGlyph(vufKey);
        }

        initForTeleop();    //Because initializing in teleop moves servos before teleop begins, this
        sleep(1000);        //function allows us to initialize legally in the correct position.

    }
}