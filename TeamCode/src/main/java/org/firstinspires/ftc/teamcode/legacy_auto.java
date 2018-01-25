package org.firstinspires.ftc.teamcode;

/**
 * Created by Jake on 1/24/18.
 */

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.detectors.JewelDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;



/**
 * Copyright (c) 2017 FTC Team 5484 Enderbots
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * This is a VuforiaLocalizer replacement, as it can do everything it does, as well as detach from the camera.
 *     To use, one can replace statements like:
 *
 *         VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(parameters);
 *
 *     with
 *
 *         ClosableVuforiaLocalizer vuforia = new ClosableVuforiaLocalizer(parameters);
 *
 *     To close vuforia, simply call vuforia.close();
 */

@Autonomous(name = "legacy_auto", group = "LinearOpMode")

public class legacy_auto extends LinearOpMode {

    //============================VARIABLES + CONSTANTS=============================================

    public DcMotor lfDriveM, lbDriveM, rfDriveM, rbDriveM, glyphLiftM;
    public Servo lGlyphS, rGlyphS, jewelExtendS, jewelKnockS;
    public CRServo glyphSlideS;

    private double extendArm = .55;
    private double retractArm = 0;

    private double knockCenter = .5;   //jewelKnockS returns to center variable
    private double knockLeft = 0;      //jewelKnockS knock left jewel variable
    private double knockRight = 1;     //jewelKnockS knocks right jewel variable

    private double lGlyphSInit = .39;               //Glyph arms will initialize in the open position.
    private double rGlyphSInit = .61;
    private double lGlyphSGrasp = 0;              //After testing, these positions were optimal for grasping the glyphs.
    private double rGlyphSGrasp = 1;

    DcMotorSimple.Direction FORWARD = DcMotorSimple.Direction.FORWARD;
    DcMotorSimple.Direction BACKWARD = DcMotorSimple.Direction.REVERSE;

    int locationX = 0;
    int locationY = 0;
    int angularOffset = 0;

    ClosableVuforiaLocalizer vuforia;    // The Vuforia camera
    BNO055IMU imu;               // IMU Gyro sensor inside of REV Hub
    Orientation angles;          // variables of the IMU that get the rotation of the robot
    Acceleration gravity;

    private ElapsedTime runtime = new ElapsedTime();
    private ElapsedTime gyroTimer = new ElapsedTime();
    private JewelDetector jewelDetector = null;

    String alliance = "";
    String jewelOrder = "";
    String stone = "";
    String stoneAndKey = "";
    String vuforiaLicenseKey = "Ae3H91v/////AAAAGT+4TPU5r02VnQxesioVLr0qQzNtgdYskxP7aL6/" +     // Yay, random Vuforia license key!
            "yt9VozCBUcQrSjwec5opfpOWEuc55kDXNNSRJjLAnjGPeaku9j4nOfe7tWxio/xj/uNdPX7fEHD0j5b" +
            "5M1OgX/bkWoUV6pUTAsKj4GaaAKIf76vnX36boqJ7BaMJNuhkYhoQJWdVqwFOC4veNcABzJRw4mQmfO" +
            "3dfPvNVjxDl8kgdBEQOZRi9kFDy9w3cTLatSGZne3IvyaYYd8uckzPnQb5Mgel3ORjar/84qO+GBmG2" +
            "vDhmiv+vkY4gbCtS0em5LM+7CIMuZa5vO9GmtqXyNsoCp9zpPlgZHc1OJ7javiI5jAzWEKCPjZcmLAkSs7k+amw";

    private ElapsedTime vutimer = new ElapsedTime();

    public String decryptKey(VuforiaTrackable cryptokeys) {
        vutimer.reset();
        RelicRecoveryVuMark vuMark;
        while (isStarted()) {
            vuMark = RelicRecoveryVuMark.from(cryptokeys);

            if (vuMark == RelicRecoveryVuMark.LEFT) {       //Store which cryptokey is found and
                stoneAndKey = stone + "KeyLeft";            //update the telemetry accordingly.
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
            } else {
                stoneAndKey = "KeyUnknown";
                telemetry.addData("Spotted Key", "Unknown");
                telemetry.update();
            }
            if(isStopRequested() || stoneAndKey != "KeyUnknown" || vutimer.time() > 3) {
                break;           //Stop this code if a stop is requested, or if the VuMark has been
            }                    //found, or if more than 3 seconds have passed.
        }
        return stoneAndKey;
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


    private void bumpJewel(String alliance, String jewel) throws InterruptedException {
        if (alliance == "blue" && jewel == "RED_BLUE") {
            jewelExtendS.setPosition(extendArm);
            sleep(750);
            jewelKnockS.setPosition(knockLeft);
            sleep(500);
            jewelKnockS.setPosition(knockCenter);
            sleep(500);
            jewelExtendS.setPosition(retractArm);
            sleep(500);
        } else if (alliance == "blue" && jewel == "BLUE_RED") {
            jewelExtendS.setPosition(extendArm);
            sleep(750);
            jewelKnockS.setPosition(knockRight);
            sleep(500);
            jewelKnockS.setPosition(knockCenter);
            sleep(500);
            jewelExtendS.setPosition(retractArm);
            sleep(500);
        } else if (alliance == "red" && jewel == "RED_BLUE") {
            jewelExtendS.setPosition(extendArm);
            sleep(750);
            jewelKnockS.setPosition(knockRight);
            sleep(500);
            jewelKnockS.setPosition(knockCenter);
            sleep(500);
            jewelExtendS.setPosition(retractArm);
            sleep(500);
        } else if (alliance == "red" && jewel == "BLUE_RED") {
            jewelExtendS.setPosition(extendArm);
            sleep(750);
            jewelKnockS.setPosition(knockLeft);
            sleep(500);
            jewelKnockS.setPosition(knockCenter);
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
        sleep(5000); //For testing purposes to see the telemetry, we should remove this for the final auto.

        this.vuforia.close();
        return vufkey;
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

        //Glypher left-to-right motor
        glyphSlideS = hardwareMap.crservo.get("glyphSlideS"); //Hub 3 Servo 1
        glyphSlideS.setPower(0);

        lGlyphS = hardwareMap.servo.get("lGlyphS"); //Hub 3 Servo 3
        lGlyphS.setPosition(lGlyphSInit);
        rGlyphS = hardwareMap.servo.get("rGlyphS"); //Hub 3 Servo 5
        rGlyphS.setPosition(rGlyphSInit);


        jewelExtendS = hardwareMap.servo.get("jewelExtendS"); //Hub 3 Servo 0
        jewelExtendS.setPosition(retractArm);
        jewelKnockS = hardwareMap.servo.get("jewelKnockS"); //Hub 2 Servo 4
        jewelKnockS.setPosition(knockCenter);


        lfDriveM.setDirection(DcMotor.Direction.REVERSE);       //Reverse the left side of the drive
        lbDriveM.setDirection(DcMotor.Direction.REVERSE);       //train for intuitive human interface

        telemetry.addData("Status", "Initialized");

        // =======================BEGIN SELECTION===================================================
        telemetry.addData("Selection", "X for Blue, B for Red");        // Which side are you on?
        telemetry.update();
        while (alliance == "") {
            if (gamepad1.x) {
                alliance = "blue";
            } else if (gamepad1.b) {
                alliance = "red";
                angularOffset = 180;
            }
            if(isStopRequested()) {             // Found this one boolean in LinearOpMode
                break;                          // that checks if STOP is hit.
            }                                   // Could help with the OpModeStuckInStop issues.
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

        jewelOrder = jewelDetector.getCurrentOrder().toString(); //Store the jewel order for use later on.
        jewelDetector.disable(); //Once we have the jewel order, disable OpenCV so we can activate Vuforia.
        jewelDetector = null;
        telemetry.addData("Jewel Order", jewelOrder);
        telemetry.update();

//  =======================================AUTONOMOUS===============================================

        bumpJewel(alliance, jewelOrder);
        String cryptoKey = activateVuforia();
        telemetry.addData("Crytpokey", cryptoKey);
        telemetry.update();
        sleep(5000);

    }
}