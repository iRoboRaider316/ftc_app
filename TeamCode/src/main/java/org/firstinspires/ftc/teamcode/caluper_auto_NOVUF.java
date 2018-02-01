package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by Ian on 11/2/2017.
 */

@Autonomous(name="caluper_auto_Novuf", group="LinearOpMode")
@Disabled
public class caluper_auto_NOVUF extends LinearOpMode {
    ElapsedTime timer = new ElapsedTime();

    public DcMotor lfDriveM,     // Left Front Drive
                   rfDriveM,     // Left Back Drive
                   lbDriveM,     // Right Front Drive
                   rbDriveM,     // Right Back Drive
                   liftM;        // Lift Glyph Grabber

    public Servo   lArmS,
                   rArmS;

    public CRServo jewelArmS;

    ColorSensor sensorColorFwd;
    ColorSensor sensorColorBck;

    private double lArmSGrasp = 0;
    private double rArmSGrasp = 1;
    private double lArmSRelease = 1;
    private double rArmSRelease = 0;

    boolean blue = false;
    boolean red  = false;
    boolean rightStone = false;
    boolean leftStone  = false;

    DcMotorSimple.Direction FORWARD = DcMotorSimple.Direction.FORWARD;
    DcMotorSimple.Direction BACKWARD = DcMotorSimple.Direction.REVERSE;

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
        if(sensorColorFwd.red() > sensorColorFwd.blue() || sensorColorBck.blue() > sensorColorBck.red()) {       // Is detected jewel red?
            drive(0.25 * direction, 0.25 * direction);       // Drive to knock it off.
            sleep(200);
            driveStop();
            jewelBumper(FORWARD, 2700);                          // Raise Jewel Bumper
        } else if(sensorColorBck.red() > sensorColorBck.blue() || sensorColorFwd.blue() > sensorColorFwd.red()) {// Is detected jewel blue?
            drive(-0.25 * direction, -0.25 * direction);   // Drive to knock off red jewel
            sleep(500);                                    // it's called indirect proof
            driveStop();
            jewelBumper(FORWARD, 2700);                          // Raise Jewel Bumper
            drive(0.5 * direction, 0.5 * direction);       // Drive back on stone (We won't need it later on)
            sleep(600);
            driveStop();
        } else {
            jewelBumper(FORWARD, 2700);                          // Raise Jewel Bumper
            drive(0.25 * direction, 0.25 * direction);           // Drive
            sleep(200);
            driveStop();
            sleep(700);
        }
    }

    public void bumpBlueJewel(int direction) throws InterruptedException {              // Knock the blue jewel off. For red side only.
        jewelBumper(BACKWARD, 2700);                               // Lower Jewel Bumper
        if(sensorColorBck.red() > sensorColorBck.blue() || sensorColorFwd.blue() > sensorColorFwd.red()) {        // Is detected jewel blue?
            drive(-0.25 * direction, -0.25 * direction);    // Drive to knock off blue jewel
            sleep(300);                                     // it's called indirect proof
            driveStop();
            jewelBumper(FORWARD, 2700);                           // Raise Jewel Bumper
            drive(0.5 * direction, 0.5 * direction);        // Drive back on stone (We won't need it later on)
            sleep(400);
            driveStop();
        } else if(sensorColorFwd.red() > sensorColorFwd.blue() || sensorColorBck.blue() > sensorColorBck.red()) { // Is detected jewel red?
            drive(0.25 * direction, 0.25 * direction);      // Drive to knock it off.
            sleep(600);
            driveStop();
            jewelBumper(FORWARD, 2700);                           // Raise the Jewel Bumper
        } else {
            jewelBumper(FORWARD, 2700);                          // Raise Jewel Bumper
            drive(0.25 * direction, 0.25 * direction);           // Drive
            sleep(200);
            driveStop();
            sleep(700);
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

    public void placeGlyph() {
        drive(-0.25, -0.25);
        sleep(2000);
        driveStop();
        grabbers(lArmSRelease, rArmSRelease);
        drive(0.2, 0.2);
        sleep(400);
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
        jewelArmS = hardwareMap.crservo.get("jewelArmS"); //Jewel Arm, Hub 2, Port 3

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

        grabbers(lArmSGrasp, rArmSGrasp);

        // =======================BEGIN SELECTION===================================================
        telemetry.addData("Selection", "X for Blue, B for Red");        // Which side are you on?
        telemetry.update();
        while(!blue && !red) {
            if(gamepad1.x) {
                blue = true;
            } else if(gamepad1.b) {
                red = true;
            }
        }
        sleep(500);
        telemetry.addData("Selection", "X for Left, B for Right");  // Which stone is the robot on?
        telemetry.update();
        while(!leftStone && !rightStone) {
            if(gamepad1.x) {
                leftStone = true;
            } else if(gamepad1.b) {
                rightStone = true;
            }
        }
                                                // Now we display what choices were made.
        if(blue) {
            telemetry.addData("Team", "Blue");  // Delay isn't necessary for now.
        } else if(red) {
            telemetry.addData("Team", "Red");
        }

        if(leftStone) {                         // The display here is based on the field edge
            telemetry.addData("Stone", "Left"); // without any cryptoboxes.
        } else if(rightStone) {                 // If you're red, it's on your left.
            telemetry.addData("Stone", "Right");// If you're blue, it's on your right.
        }

        telemetry.update();
        //AutoTransitioner.transitionOnStop(this, "caluper_teleop");     // Once Auto is done, quickly switch to our Teleop (Thank you KNO3!)
        waitForStart();
        timer.reset();
        // =======================================AUTONOMOUS========================================
        if(red) {
            if(leftStone) {
                bumpBlueJewel(1);                               // Bump the Blue Jewel
                drive(0.25, 0.25);                              // Drive to Cryptobox
                sleep(1100);
                drive(0.25, -0.25);                             // Simple Turn to Cryptobox
                sleep(850);
                driveStop();
                sleep(700); // just to see what is happening
                placeGlyph();                                   // Place Glyph in column
            } else if(rightStone) {
                bumpBlueJewel(1);   //Bump the Blue Jewel
                sleep(1200);
                drive(0.15, 0.15);    //Back into Safe Zone
                sleep(1200);
                driveStop();
                drive(-0.27, 0.27);
                sleep(1100);
                drive(0.1, 0.1);
                sleep(1200);
                placeGlyph();
            }
        } else if (blue) {
            if (rightStone) {
                bumpRedJewel(-1);                               // Bump the Red Jewel
                drive(-0.25, -0.25);                            // Drive to Cryptobox
                sleep(1210);
                drive(0.25, -0.25);                             // Simple Turn to Cryptobox
                sleep(700);
                driveStop();
                sleep(700); // just to see what is happening
                placeGlyph();                             // Place Glyph in column
            } else if (leftStone) {
                bumpRedJewel(-1);
                drive(-0.2, -0.2);                  //drive forward
                sleep(500);
                driveStop();

                drive(-0.40, 0.40);                 //turn a bit to aim for cryptobox
                sleep(300);
                driveStop();

                drive(-0.25, -0.25);                //drive into cryptobox
                sleep(2000);
                driveStop();

                lArmS.setPosition(lArmSRelease);    //release glyph
                rArmS.setPosition(rArmSRelease);

                drive(0.2, 0.2);                    //drive back from the cryptobox to score
                sleep(300);
                driveStop();

                drive(0.40, -0.40);                 //turn to get the glyph MORE in the box
                sleep(300);
                driveStop();

                drive(0.2, 0.2);                    //drive back a bit to not touch the glyph
                sleep(200);
                driveStop();
            }
        }
        driveStop();
    }
}
// Please let Ian H. know if there is anything that needs to be fixed!