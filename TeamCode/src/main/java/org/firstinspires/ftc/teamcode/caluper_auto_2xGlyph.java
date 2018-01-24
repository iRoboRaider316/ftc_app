package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

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

@Autonomous(name = "caluper_auto_2xGlyph", group = "LinearOpMode")
@Disabled
public class caluper_auto_2xGlyph extends LinearOpMode {

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

    boolean blue = false;
    boolean red = false;
    boolean rightStone = false;
    boolean leftStone = false;

    BNO055IMU imu;               // IMU Gyro sensor inside of REV Hub
    Orientation angles;          // variables of the IMU that get the rotation of the robot
    Acceleration gravity;

    enum Cryptokey {             // Cryptokey Enumerations for determining what the camera saw
        LEFT, CENTER, RIGHT, NONE
    }

    Cryptokey crypto = Cryptokey.NONE;  // Grabs what the camera saw from the pictograph

    VuforiaLocalizer vuforia;           // The actual Vuforia camera

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
            sleep(300);
            driveStop();
            jewelBumper(FORWARD, 2700);                          // Raise Jewel Bumper
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
            sleep(400);                                     // it's called indirect proof
            driveStop();
            jewelBumper(FORWARD, 2700);                           // Raise Jewel Bumper
            drive(0.5 * direction, 0.5 * direction);        // Drive back on stone (We won't need it later on)
            sleep(350);
            driveStop();
        } else if (sensorColorFwd.red() > sensorColorFwd.blue() || sensorColorBck.blue() > sensorColorBck.red()) { // Is detected jewel red?
            drive(0.25 * direction, 0.25 * direction);      // Drive to knock it off.
            sleep(300);
            driveStop();
            jewelBumper(FORWARD, 2700);                           // Raise the Jewel Bumper
        } else {
            jewelBumper(FORWARD, 2700);
            drive(0.25 * direction, 0.25 * direction);       // Drive to knock it off.
            sleep(300);
            driveStop();
            sleep(500);
        }
    }

    public void decryptKey(VuforiaTrackable cryptokeys) {
        RelicRecoveryVuMark vuMark;
        while (!isStarted()) {
            vuMark = RelicRecoveryVuMark.from(cryptokeys);

            if (vuMark == RelicRecoveryVuMark.LEFT) {
                crypto = Cryptokey.LEFT;
                telemetry.addData("Spotted Key", "Left!");
                telemetry.update();
                return;
            } else if (vuMark == RelicRecoveryVuMark.CENTER) {
                crypto = Cryptokey.CENTER;
                telemetry.addData("Spotted Key", "Center!");
                telemetry.update();
                return;
            } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
                crypto = Cryptokey.RIGHT;
                telemetry.addData("Spotted Key", "Right!");
                telemetry.update();
                return;
            } else {
                telemetry.addData("Spotted Key", "searching...");
                telemetry.update();
            }
        }
    }

    public void placeGlyph(Cryptokey key, boolean backUp) {
        switch (key) {
            case LEFT:
                drive(0.25, -0.25);
                sleep(300);
                drive(-0.25, -0.25);
                sleep(1000);
                driveStop();
                grabbers(lArmSRelease, rArmSRelease);
                if(backUp) {
                    drive(0.25, 0.25);
                    sleep(1000);
                    drive(-0.25, 0.25);
                    sleep(300);
                }
                break;
            case CENTER:
                drive(-0.25, -0.25);
                sleep(1000);
                driveStop();
                grabbers(lArmSRelease, rArmSRelease);
                if(backUp) {
                    drive(0.25, 0.25);
                    sleep(1000);
                }
                break;
            case RIGHT:
                drive(-0.25, 0.25);
                sleep(300);
                drive(-0.25, -0.25);
                sleep(1000);
                driveStop();
                grabbers(lArmSRelease, rArmSRelease);
                if(backUp) {
                    drive(0.25, 0.25);
                    sleep(1000);
                    drive(0.25, -0.25);
                    sleep(300);
                }
                break;
            default:
                drive(-0.25, -0.25);
                sleep(1000);
                driveStop();
                grabbers(lArmSRelease, rArmSRelease);
                if(backUp) {
                    drive(0.25, 0.25);
                    sleep(1000);
                }
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

    /*
     * Perform a turn with the gyro sensor. For degreesToTurn, positive is clockwise, and negative
     * is counterclockwise.
     * angles.firstAngle is used for currentRotation.
     */

    public void imuTurn(float degreesToTurn) {
        updateIMU();                                    // Update the IMU to see where we are,
                                                        // rotation-wise.
        /*
         * For us, the IMU has had us turn just a bit more than what we intend. The operation
         * below accounts for this by dividing the current degreesToTurn value by 8/9.
         */
        degreesToTurn *= (8.0F / 9.0F);

        float targetHeading = degreesToTurn + angles.firstAngle;

        /*
         * These operations account for when the robot would cross the IMU rotation line, which
         * separates -180 from 180. Adding or subtracting the degreesToTurn by 360 here isn't
         * always necessary, however, so we skip this operation in those cases*/

        if(targetHeading > 180) {
            targetHeading -= 360;
        }

        if(targetHeading < -180) {
            targetHeading += 360;
        }

        if (targetHeading > angles.firstAngle) {
            while (targetHeading > angles.firstAngle) {
                updateIMU();
                drive(0.2, -0.2);
                telemetry.addData("Gyro", angles.firstAngle);
                telemetry.addData("Target", targetHeading);
                telemetry.update();
            }
        } else {
            while (targetHeading < angles.firstAngle) {
                updateIMU();
                drive(-0.2, 0.2);
                telemetry.addData("Gyro", angles.firstAngle);
                telemetry.addData("Target", targetHeading);
                telemetry.update();
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

        int cameraMonitorViewId = hardwareMap.appContext.getResources().
                getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName()); // Get the camera!
        VuforiaLocalizer.Parameters parameters_Vuf = new VuforiaLocalizer.Parameters(cameraMonitorViewId);    // Prepare the parameters
        parameters_Vuf.vuforiaLicenseKey = "Ae3H91v/////AAAAGT+4TPU5r02VnQxesioVLr0qQzNtgdYskxP7aL6/" +       // This is long...
                "yt9VozCBUcQrSjwec5opfpOWEuc55kDXNNSRJjLAnjGPeaku9j4nOfe7tWxio/xj/uNdPX7fEHD0j5b" +
                "5M1OgX/bkWoUV6pUTAsKj4GaaAKIf76vnX36boqJ7BaMJNuhkYhoQJWdVqwFOC4veNcABzJRw4mQmfO" +
                "3dfPvNVjxDl8kgdBEQOZRi9kFDy9w3cTLatSGZne3IvyaYYd8uckzPnQb5Mgel3ORjar/84qO+GBmG2" +
                "vDhmiv+vkY4gbCtS0em5LM+7CIMuZa5vO9GmtqXyNsoCp9zpPlgZHc1OJ7javiI5jAzWEKCPjZcmLAkSs7k+amw";
        parameters_Vuf.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;                              // Look through the camera you use for selfies
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
        while (!blue && !red) {
            if (gamepad1.x) {
                blue = true;
            } else if (gamepad1.b) {
                red = true;
            }
        }
        sleep(500);
        telemetry.addData("Selection", "X for Left, B for Right");  // Which stone is the robot on?
        telemetry.update();
        while (!leftStone && !rightStone) {
            if (gamepad1.x) {
                leftStone = true;
            } else if (gamepad1.b) {
                rightStone = true;
            }
        }
        // Now we display what choices were made.
        if (blue) {
            telemetry.addData("Team", "Blue");  // Delay isn't necessary for now.
        } else if (red) {
            telemetry.addData("Team", "Red");
        }

        if (leftStone) {                         // The display here is based on the field edge
            telemetry.addData("Stone", "Left"); // without any cryptoboxes.
        } else if (rightStone) {                 // If you're red, it's on your left.
            telemetry.addData("Stone", "Right");// If you're blue, it's on your right.
        }

        telemetry.update();
        Cryptokey.activate();
        //AutoTransitioner.transitionOnStop(this, "caluper_teleop");     // Once Auto is done, quickly switch to our Teleop (Thank you KNO3!)
        decryptKey(Targets);
        waitForStart();
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        updateIMU();
        // =======================================AUTONOMOUS========================================
        if (red) {
            if (leftStone) {
                bumpBlueJewel(1);
                drive(0.25, 0.25);                              // Drive to Cryptobox
                sleep(1100);
                imuTurn(-90);              // Turn with IMU!
                sleep(700); // just to see what is happening
                placeGlyph(crypto, true);                             // Place Glyph in column depending on pictograph
            } else if (rightStone) {
                bumpBlueJewel(1);
                sleep(1200);
                drive(0.15, 0.15);    //Back into Safe Zone
                sleep(1200);
                driveStop();
                imuTurn(120);
                drive(0.1, 0.1);
                sleep(1200);
                driveStop();
            }
        } else if (blue) {
            if (rightStone) {
                bumpRedJewel(-1);                  // Bump Red Jewel
                drive(-0.25, -0.25);               // Drive to Cryptobox...
                sleep(1130);                       // ...for 1.1 seconds
                imuTurn(90);                       // Turn to Cryptobox!
                sleep(700);                        // wait 0.7 seconds just to see what is happening
                placeGlyph(crypto, true);          // Place Glyph in column depending on pictograph
                imuTurn(-180);                     // Turn towards the glyph pile
                drive(-0.25, -0.25);               // Drive into Glyph Pile
                sleep(1600);
                driveStop();
                grabbers(lArmSGrasp, rArmSGrasp);  // Grab and pick up a glyph
                liftM.setPower(1);
                sleep(600);
                liftM.setPower(0);
                drive(0.25, 0.25);                 // Drive back to Cryptobox
                sleep(1500);
                imuTurn(180);                      // Turn to Cryptobox
                sleep(700);
                placeGlyph(crypto, false);         // Place the glyph in Caluper
                drive(0.25, 0.25);                 // Get back in Safe Zone
                sleep(100);
            } else if (leftStone) {
                bumpRedJewel(-1);
                drive(-0.7, -0.2);                              // Drive to Cryptobox while curving
                sleep(1000);
                imuTurn(-90);             // Turn with IMU!
                driveStop();
                sleep(700); // just to see what is happening
                placeGlyph(crypto, true);                             // Place Glyph in column depending on pictograph
            }
        }
        driveStop();
    }
}
// Please let Ian H. know if there is anything that needs to be fixed!