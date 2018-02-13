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
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

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

@Autonomous(name = "legacy_auto", group = "LinearOpMode")

public class legacy_auto extends LinearOpMode {

    //============================VARIABLES + CONSTANTS=============================================

    public DcMotor lfDriveM, lbDriveM, rfDriveM, rbDriveM, glyphLiftM;
    public Servo lGlyphS, rGlyphS, jewelExtendS, jewelHitS;
    public CRServo glyphSlideS;

    private double extendArm = .55;
    private double retractArm = 0;

    private double hitCenter = .5;   //jewelHitS returns to center variable
    private double hitPLeft = .25;   //Protects arm from falling in case of power outages.
    private double hitLeft = 0;      //jewelHitS knock left jewel variable
    private double hitRight = 1;     //jewelHitS knocks right jewel variable

    private double lGlyphSRelease = 1;               //Glyph arms will initialize in the open position.
    private double rGlyphSRelease = 0;
    private double lGlyphSGrasp = 0;              //After testing, these positions were optimal for grasping the glyphs.
    private double rGlyphSGrasp = 1;

    DcMotorSimple.Direction FORWARD = DcMotorSimple.Direction.FORWARD;
    DcMotorSimple.Direction BACKWARD = DcMotorSimple.Direction.REVERSE;
    DcMotorSimple.Direction LEFT = DcMotorSimple.Direction.FORWARD;
    DcMotorSimple.Direction RIGHT = DcMotorSimple.Direction.REVERSE;

    ClosableVuforiaLocalizer vuforia;    // The Vuforia camera
    BNO055IMU imu;               // IMU Gyro sensor inside of REV Hub
    Orientation angles;          // variables of the IMU that get the rotation of the robot

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime gyroTimer = new ElapsedTime();
    private JewelDetector jewelDetector = null;

    private String alliance = "";
    private String jewelOrder = "";
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
        String key = "";
        vutimer.reset();
        RelicRecoveryVuMark vuMark;
        while (isStarted()) {
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
            } else {
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
                drive(-0.25, 0.25);
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
                drive(0.25, -0.25);
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

    public void driveOffStone(String Alliance) throws InterruptedException {
        if(alliance == "red") {
            encoderDrive(24, 0.23, 1);
            sleep(500);
        } else if(alliance == "blue") {
            encoderDrive(-25, 0.23, -1);
        }
    }

    public void driveToColumn(String Alliance, String Stone) throws InterruptedException {
        switch(Alliance + Stone) {
            case "redleft":
                encoderDrive(12, 0.23, 1);
                imuTurn(-90, "LEFT");
                break;
            case "redright":
                imuTurn(90, "RIGHT");
                encoderDrive(12, 0.23, -1);
                imuTurn(90, "RIGHT");
                break;
            case "blueleft":
                imuTurn(90, "RIGHT");
                encoderDrive(-11, 0.23, -1);
                imuTurn(-90, "LEFT");
                break;
            case "blueright":
                encoderDrive(-12, 0.23, -1);
                imuTurn(-90, "LEFT");
                break;
        }
    }

    public void placeGlyph(String Alliance, String Stone, String Key) throws InterruptedException {
        switch(Alliance + Stone) {
            case "redleft":
                switch (Key) {
                    case "KeyLeft":
                        imuTurn(-22, "LEFT");
                        drive(-0.23, -0.23);
                        sleep(1000);
                        driveStop();
                        grabbers(lGlyphSRelease, rGlyphSRelease);
                        break;
                    case "KeyCenter":
                        moveSliders(LEFT, 200);
                        drive(-0.23, -0.23);
                        sleep(900);
                        driveStop();
                        grabbers(lGlyphSRelease, rGlyphSRelease);
                        break;
                    case "KeyRight":
                        imuTurn(14, "RIGHT");
                        drive(-0.23, -0.23);
                        sleep(1000);
                        driveStop();
                        grabbers(lGlyphSRelease, rGlyphSRelease);
                        break;
                    default:
                        imuTurn(-22, "LEFT");
                        drive(-0.23, -0.23);
                        sleep(1000);
                        driveStop();
                        grabbers(lGlyphSRelease, rGlyphSRelease);
                        break;
                }
                break;
            case "redright":
                switch (Key) {
                    case "KeyLeft":
                        imuTurn(-13, "LEFT");
                        drive(-0.23, -0.23);
                        sleep(1000);
                        driveStop();
                        grabbers(lGlyphSRelease, rGlyphSRelease);
                        break;
                    case "KeyCenter":
                        drive(-0.23, -0.23);
                        sleep(700);
                        driveStop();
                        grabbers(lGlyphSRelease, rGlyphSRelease);
                        break;
                    case "KeyRight":
                        imuTurn(16, "RIGHT");
                        moveSliders(RIGHT, 200);
                        drive(-0.23, -0.23);
                        sleep(1000);
                        driveStop();
                        grabbers(lGlyphSRelease, rGlyphSRelease);
                        break;
                    default:
                        imuTurn(-13, "LEFT");
                        drive(-0.23, -0.23);
                        sleep(1000);
                        driveStop();
                        grabbers(lGlyphSRelease, rGlyphSRelease);
                        break;
                }
                break;
            case "blueleft":
                switch (Key) {
                    case "KeyLeft":
                        imuTurn(-20, "LEFT");
                        drive(-0.23, -0.23);
                        sleep(1000);
                        driveStop();
                        grabbers(lGlyphSRelease, rGlyphSRelease);
                        break;
                    case "KeyCenter":
                        moveSliders(RIGHT, 200);
                        drive(-0.23, -0.23);
                        sleep(1000);
                        driveStop();
                        grabbers(lGlyphSRelease, rGlyphSRelease);
                        break;
                    case "KeyRight":
                        imuTurn(14, "RIGHT");
                        moveSliders(LEFT, 200);
                        sleep(200);
                        drive(-0.23, -0.23);
                        sleep(1000);
                        driveStop();
                        grabbers(lGlyphSRelease, rGlyphSRelease);
                        break;
                    default:
                        imuTurn(-20, "LEFT");
                        drive(-0.23, -0.23);
                        sleep(1000);
                        driveStop();
                        grabbers(lGlyphSRelease, rGlyphSRelease);
                        break;
                }
                break;
            case "blueright":
                switch (Key) {
                    case "KeyLeft":
                        imuTurn(-15, "LEFT");
                        drive(-0.23, -0.23);
                        sleep(1000);
                        driveStop();
                        grabbers(lGlyphSRelease, rGlyphSRelease);
                        break;
                    case "KeyCenter":
                        drive(-0.23, -0.23);
                        sleep(1000);
                        driveStop();
                        grabbers(lGlyphSRelease, rGlyphSRelease);
                        break;
                    case "KeyRight":
                        imuTurn(14, "RIGHT");
                        sleep(200);
                        drive(-0.23, -0.23);
                        sleep(1000);
                        driveStop();
                        grabbers(lGlyphSRelease, rGlyphSRelease);
                        break;
                    default:
                        drive(-0.23, -0.23);
                        sleep(1000);
                        driveStop();
                        grabbers(lGlyphSRelease, rGlyphSRelease);
                        break;
                }
                break;
        }
    }

    private void bumpJewel(String alliance, String jewel) throws InterruptedException {
        if (alliance == "blue" && jewel == "RED_BLUE") {
            jewelExtendS.setPosition(extendArm);
            sleep(750);
            if (jewelBumpType == "correct") {jewelHitS.setPosition(hitLeft);
            } else { jewelHitS.setPosition(hitRight); }
            sleep(500);
            jewelHitS.setPosition(hitCenter);
            sleep(500);
            jewelExtendS.setPosition(retractArm);
            sleep(500);
        } else if (alliance == "blue" && jewel == "BLUE_RED") {
            jewelExtendS.setPosition(extendArm);
            sleep(750);
            if (jewelBumpType == "correct") {jewelHitS.setPosition(hitRight);
            } else { jewelHitS.setPosition(hitLeft); };
            sleep(500);
            jewelHitS.setPosition(hitCenter);
            sleep(500);
            jewelExtendS.setPosition(retractArm);
            sleep(500);
        } else if (alliance == "red" && jewel == "RED_BLUE") {
            jewelExtendS.setPosition(extendArm);
            sleep(750);
            if (jewelBumpType == "correct") {jewelHitS.setPosition(hitRight);
            } else { jewelHitS.setPosition(hitLeft); };
            sleep(500);
            jewelHitS.setPosition(hitCenter);
            sleep(500);
            jewelExtendS.setPosition(retractArm);
            sleep(500);
        } else if (alliance == "red" && jewel == "BLUE_RED") {
            jewelExtendS.setPosition(extendArm);
            sleep(750);
            if (jewelBumpType == "correct") {jewelHitS.setPosition(hitLeft);
            } else { jewelHitS.setPosition(hitRight); }
            sleep(500);
            jewelHitS.setPosition(hitCenter);
            sleep(500);
            jewelExtendS.setPosition(retractArm);
            sleep(500);
        } else {
            if (jewelOrder == "UNKNOWN") {
                telemetry.addData("Problem with 'jewelOrder'", "  ;-;");
                telemetry.update();
                sleep(3000);
            } else {
                telemetry.addData("Problem with 'alliance'", "  ;-;");
            }
        }
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
            telemetry.addData("Problem with alliance or jewelOrder.", "");
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
        telemetry.addData("Searching for Key", "...");
        telemetry.update();
        String vufkey = decryptKey(Targets);
        telemetry.addData("Key Found", vufkey);
        telemetry.update();
        sleep(1000); //For testing purposes to see the telemetry, we should remove this for the final auto.

        this.vuforia.close();
        return vufkey;
    }

    private void grabRelic() {
        new Thread(new Runnable() {
            public void run() {
                //
                // Relic Grabber Code Here
                //

                /* //Use this to sleep in subthreads
		        try {
		            Thread.sleep(1000);
		        } catch(InterruptedException ie) {}
		        */
            }
        }).start();
    }

    @Override
    public void runOpMode() throws InterruptedException {

        // ===================================INIT==================================================

        //Drive motors
        lfDriveM = hardwareMap.dcMotor.get("lfDriveM"); //Hub 3 Port 2
        lfDriveM.setPower(0);
        lbDriveM = hardwareMap.dcMotor.get("lbDriveM"); //Hub 3 Port 3
        lbDriveM.setPower(0);
        rfDriveM = hardwareMap.dcMotor.get("rfDriveM"); //Hub 3 Port 0
        rfDriveM.setPower(0);
        rbDriveM = hardwareMap.dcMotor.get("rbDriveM"); //Hub 3 Port 1
        rbDriveM.setPower(0);

        //Lifting glypher motor
        glyphLiftM = hardwareMap.dcMotor.get("glyphLiftM"); //Hub 2 Port 0
        glyphLiftM.setPower(0);
        glyphLiftM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Glypher left-to-right motor
        glyphSlideS = hardwareMap.crservo.get("glyphSlideS"); //Hub 3 Servo 1
        glyphSlideS.setPower(0);

        lGlyphS = hardwareMap.servo.get("lGlyphS"); //Hub 3 Servo 3
        rGlyphS = hardwareMap.servo.get("rGlyphS"); //Hub 3 Servo 5
        grabbers(lGlyphSRelease, rGlyphSRelease);

        // Jewel Knocker
        jewelExtendS = hardwareMap.servo.get("jewelExtendS"); //Hub 3 Servo 0
        jewelExtendS.setPosition(retractArm);
        jewelHitS = hardwareMap.servo.get("jewelHitS"); //Hub 2 Servo 4
        jewelHitS.setPosition(hitCenter);

        lfDriveM.setDirection(DcMotor.Direction.REVERSE);       //Reverse the left side of the drive
        lbDriveM.setDirection(DcMotor.Direction.REVERSE);       //train for intuitive human interface
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
        imu = hardwareMap.get(BNO055IMU.class, "imu");
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

        sleep(500);

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

        while (!isStarted()) { //Track the jewel order until init ends.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Current Order", "Jewel Order: " + jewelDetector.getCurrentOrder().toString()); // Current Result
            telemetry.addData("Last Order", "Jewel Order: " + jewelDetector.getLastOrder().toString()); // Last Known Result
            telemetry.update();
            if(isStopRequested()) {
                break;
            }
        }

        waitForStart();

        jewelOrder = jewelDetector.getLastOrder().toString(); //Store the jewel order for use later on.
        jewelDetector.disable(); //Once we have the jewel order, disable OpenCV so we can activate Vuforia.
        jewelDetector = null;
        telemetry.addData("Jewel Order", jewelOrder);
        telemetry.update();

//  ====================================== AUTONOMOUS ==============================================

        if(jewelBumpType != "none") {
            bumpJewelShort(alliance, jewelOrder);
        }
        String cryptoKey = activateVuforia();
        telemetry.addData("Crytpokey", cryptoKey);
        telemetry.update();
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        grabbers(lGlyphSGrasp, rGlyphSGrasp);
        glyphLifter("UP");
        driveOffStone(alliance);
        driveToColumn(alliance, stone);
        placeGlyph(alliance, stone, cryptoKey);
        drive(0.23, 0.23);
        sleep(100);
        grabbers(lGlyphSGrasp, rGlyphSGrasp);
        driveStop();
        glyphLifter("DOWN");
        drive(-0.23, -0.23);
        sleep(500);
        drive(0.23, 0.23);
        sleep(400);
        imuTurn(45, "RIGHT");


        initForTeleop();    //Because initializing in teleop moves servos before teleop begins, this
        sleep(1000);        //function allows us to initialize legally in the correct position.

    }
}