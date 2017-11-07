package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

/**
 * Created by Ian on 11/2/2017.
 */

@Autonomous(name="CaluperAuto", group="LinearOpMode")
//@Disabled
public class CaluperAuto extends LinearOpMode {
    ElapsedTime timer = new ElapsedTime();

    public DcMotor lfDrive;     // Left Front Drive
    public DcMotor rfDrive;     // Left Back Drive
    public DcMotor lbDrive;     // Right Front Drive
    public DcMotor rbDrive;     // Right Back Drive
    public DcMotor liftMotor;   // Lift Glyph Grabber

    public Servo lServoArm;
    public Servo rServoArm;
    public Servo jewelArm;

    ColorSensor sensorColor;

    private double lServoArmInit = .73;
    private double rServoArmInit = .1;
    private double lServoArmGrasp = .43;
    private double rServoArmGrasp = .50;
    private double lServoArmAlmostGrasp = .50;
    private double rServoArmAlmostGrasp = .43;
    private double speedFactor = .5;
    private int controlMode = 1;

    boolean blue = false;
    boolean red  = false;
    boolean rightStone = false;
    boolean leftStone  = false;
    boolean AutoFinished = false;

    public void drive(double leftPower, double rightPower) {    // Turns on motors. The reason
        lfDrive.setPower(leftPower);                            // there are two powers is so the
        lbDrive.setPower(leftPower);                            // robot may be allowed to curve.
        rfDrive.setPower(rightPower);
        rbDrive.setPower(rightPower);
    }

    public void driveStop() {                                   // Stop robot motors
        lfDrive.setPower(0);
        lbDrive.setPower(0);
        rfDrive.setPower(0);
        rbDrive.setPower(0);
    }

    public void jewelBumper(double pos, long time) throws InterruptedException { // Move Jewel Bumper
        jewelArm.setPosition(pos);                 // when pos is 0, jewel bumper is lowered
        sleep(time);                               // when pos is 1, jewel bumper is raised
        jewelArm.setPosition(0.5);
    }

    public void bumpRedJewel(int direction) throws InterruptedException {  // Knock the red jewel off. For blue side only.
        jewelBumper(0, 2000);                              // Lower jewel bumper
        if(sensorColor.red() > sensorColor.blue()) {       // Is detected jewel red?
            drive(0.2 * direction, 0.2 * direction);       // Drive to knock it off.
            sleep(300);
            driveStop();
            jewelBumper(1, 2000);                          // Raise Jewel Bumper
        } else if(sensorColor.blue() > sensorColor.red()) {// Is detected jewel blue?
            drive(-0.25 * direction, -0.25 * direction);   // Drive to knock off red jewel
            sleep(600);                                    // it's called indirect proof
            driveStop();
            jewelBumper(1, 2000);                          // Raise Jewel Bumper
            drive(0.5 * direction, 0.5 * direction);       // Drive back on stone (We won't need it later on)
            sleep(600);
            driveStop();
        }
    }

    public void bumpBlueJewel(int direction) throws InterruptedException {              // Knock the blue jewel off. For red side only.
        jewelBumper(0, 2000);                               // Lower Jewel Bumper
        if(sensorColor.blue() > sensorColor.red()) {        // Is detected jewel blue?
            drive(0.25 * direction, 0.25 * direction);      // Drive to knock it off.
            sleep(600);
            driveStop();
            jewelBumper(1, 2000);                           // Raise Jewel Bumper
        } else if(sensorColor.red() > sensorColor.blue()) { // Is detected jewel red?
            drive(-0.25 * direction, -0.25 * direction);    // Drive to knock off blue jewel
            sleep(600);                                     // it's called indirect proof
            driveStop();
            jewelBumper(1, 2000);                           // Raise Jewel Bumper
            drive(0.5 * direction, 0.5 * direction);        // Drive back on stone (We won't need it later on)
            sleep(600);
            driveStop();
        }

    }

    @Override
    public void runOpMode() throws InterruptedException {
        // ===================================INIT==================================================
        lfDrive = hardwareMap.dcMotor.get("lfDrive"); //Left front drive, Hub 1, port 2
        lbDrive = hardwareMap.dcMotor.get("lbDrive"); //Left back drive, Hub 1, port 3
        rfDrive = hardwareMap.dcMotor.get("rfDrive"); //Right front drive, Hub 1, port 1
        rbDrive = hardwareMap.dcMotor.get("rbDrive"); //Right back drive, Hub 1, port 0
        liftMotor = hardwareMap.dcMotor.get("liftMotor"); //Lift motor, Hub 2, port 3

        lServoArm = hardwareMap.servo.get("lServoArm"); //Left servo arm, Hub 1, port 2
        rServoArm = hardwareMap.servo.get("rServoArm"); //Right servo arm, Hub 2, port 1
        jewelArm = hardwareMap.servo.get("jewelArm"); //Jewel Arm, Hub 2, Port 3

        sensorColor = hardwareMap.get(ColorSensor.class, "sensor_color_distance");

        lfDrive.setPower(0);
        lbDrive.setPower(0);
        rfDrive.setPower(0);
        rbDrive.setPower(0);
        liftMotor.setPower(0);
        lfDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lbDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rfDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rbDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lfDrive.setDirection(DcMotor.Direction.REVERSE);
        lbDrive.setDirection(DcMotor.Direction.REVERSE);

        lServoArm.setPosition(lServoArmInit);
        rServoArm.setPosition(rServoArmInit);
        jewelArm.setPosition(.5);

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

        if(blue) {                              // Now we display what choices were made.
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
        waitForStart();
        timer.reset();
        // =======================================AUTONOMOUS========================================
        telemetry.addData("Code Status", "Auto Go!");
        telemetry.update();
            if(red) {
                if(leftStone) {
                    // Ava's Auto
                    bumpBlueJewel(1);
                    drive(0.3, 0.6);
                    sleep(500);
                    driveStop();
                    AutoFinished = true;              // now that this is true, the loop can break.
                } else if(rightStone) {
                    // Jake's Auto.
                    bumpBlueJewel(-1);
                    drive(0.3, 0.3);
                    sleep(1200);
                    driveStop();
                    AutoFinished = true;
                }
            } else if (blue) {
                if(rightStone) {
                    // Thor's Auto
                    bumpRedJewel(-1);
                    drive(-0.3, -0.3);
                    sleep(1200);
                    driveStop();
                    AutoFinished = true;
                } else if(leftStone) {
                    // Ian's Auto
                    bumpRedJewel(-1);
                    drive(-0.6, -0.3);
                    sleep(1000);
                    driveStop();
                    AutoFinished = true;
                }
            }
        while(timer.seconds() < 30) {                         // Robot waits for Auto time to be up.
            sleep(50);
            telemetry.addData("Auto finished early!", String.format(Locale.US, "%.02f", timer.seconds()));
            telemetry.update(); // Formatting is there just because all those decimal places are unnecessary
        }
        timer.reset();
        // =========================================TRANSITION======================================
        while(!gamepad1.a) {
            telemetry.addData("Code Status", "Waiting for TeleOp!");
            telemetry.addData("To begin TeleOp, press", "A (Driver)");
            telemetry.update();
            sleep(50);
        }
        timer.reset();
        //=================================TELEOP===================================================
        telemetry.addData("Code Status", "Teleop Go!");
        telemetry.update();
        while(opModeIsActive()) {
            if(gamepad1.right_bumper) {       // full power option
                speedFactor = 1;
            } else {                          // half power option
                speedFactor = .5;
            }

            if(gamepad1.dpad_up) {            // differential lock if DPad up is held
                controlMode = 2;
            } else {                          // classic tank drive
                controlMode = 1;
            }

            switch(controlMode) {             // apply power from joysticks to drive train based on control mode
                case 1:
                    lfDrive.setPower((-gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y)) * speedFactor); //exponential scale algorithm
                    lbDrive.setPower((-gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y)) * speedFactor); //tank drive
                    rfDrive.setPower((gamepad1.right_stick_y * Math.abs(gamepad1.right_stick_y)) * speedFactor);
                    rbDrive.setPower((gamepad1.right_stick_y * Math.abs(gamepad1.right_stick_y)) * speedFactor);
                    break;
                case 2:
                    lfDrive.setPower((-gamepad1.right_stick_y * Math.abs(gamepad1.right_stick_y)) * speedFactor); //exponential scale algorithm
                    lbDrive.setPower((-gamepad1.right_stick_y * Math.abs(gamepad1.right_stick_y)) * speedFactor); //differential lock
                    rfDrive.setPower((gamepad1.right_stick_y * Math.abs(gamepad1.right_stick_y)) * speedFactor);
                    rbDrive.setPower((gamepad1.right_stick_y * Math.abs(gamepad1.right_stick_y)) * speedFactor);
                    break;
                default:
                    lfDrive.setPower((-gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y)) * speedFactor); //exponential scale algorithm
                    lbDrive.setPower((-gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y)) * speedFactor); //tank drive
                    rfDrive.setPower((gamepad1.right_stick_y * Math.abs(gamepad1.right_stick_y)) * speedFactor);
                    rbDrive.setPower((gamepad1.right_stick_y * Math.abs(gamepad1.right_stick_y)) * speedFactor);
                    break;
            }

            if (gamepad2.dpad_up) {                     // Lift Glypder!
                liftMotor.setPower(1);
            }
            else if (gamepad2.dpad_down) {              // Lower Glypher!
                liftMotor.setPower(-1);
            }
            else {
                liftMotor.setPower(0);                  // Stop Glypher!
            }

            /*if (gamepad2.dpad_up) {                   //unnecessary
                liftMotor.setPower(1);
            }
            else if (gamepad2.dpad_down) {
                liftMotor.setPower(-1);
            }
            else {
                liftMotor.setPower(0);
            }*/
            if (gamepad1.b) { //hitting the "b" button on Gamepad 1 will cause the two servos to grasp the glyph
                lServoArm.setPosition(lServoArmGrasp);
                rServoArm.setPosition(rServoArmGrasp);
            }
            if (gamepad1.x) { //hitting the "x" button on Gamepad 1 will cause the two servos to return to their original position
                lServoArm.setPosition(lServoArmInit);
                rServoArm.setPosition(rServoArmInit);
            }

            if (gamepad2.b) { //hitting the "b" button on Gamepad 2 will cause the two servos to grasp the glyph
                lServoArm.setPosition(lServoArmGrasp);
                rServoArm.setPosition(rServoArmGrasp);
            }
            if (gamepad2.x) { //hitting the "x" button on Gamepad 2 will cause the two servos to return to their original position
                lServoArm.setPosition(lServoArmInit);
                rServoArm.setPosition(rServoArmInit);
            }
            if (gamepad1.y) {   //hitting the "y" button on Gamepad 1 will cause the glypher servos to expand slightly larger than grasping the glyphs.
                lServoArm.setPosition(lServoArmAlmostGrasp);
                rServoArm.setPosition(rServoArmAlmostGrasp);
            }
            if (gamepad2.y) {   //hitting the "y" button on Gamepad 2 will cause the glypher servos to expand slightly larger than grasping the glyphs.
                lServoArm.setPosition(lServoArmAlmostGrasp);
                rServoArm.setPosition(rServoArmAlmostGrasp);
            }
            if (gamepad2.y) { //hitting the "y" button on Gamepad 2 will cause the jewel arm to drop. Could be handy.
                jewelArm.setPosition(1);
            } else {
                jewelArm.setPosition(.5);
            }

            if (gamepad2.a) { //hitting the "a" button on Gamepad 2 will cause the jewel arm to lift. Could be handy.
                jewelArm.setPosition(0);
            } else {
                jewelArm.setPosition(.5);
            }
        }           // End of Teleop
    }
}
// Please let Ian H. know if there is anything that needs to be fixed!