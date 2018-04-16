package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DigitalChannel;

/**
 * Created by Jake from Robo Raiders FTC 7129 on 6/14/17.
 */

@TeleOp (name="legacy_teleop",group="Opmode")

public class legacy_teleop extends OpMode {

    public DcMotor lfDriveM, rfDriveM, lbDriveM, rbDriveM, glyphLiftM, relicTurn, relicExtend;  //Left front drive, right front drive, left back drive, right back drive.
    public Servo lGlyphS, rGlyphS, jewelExtendS, jewelHitS, relicGrabber, relicLower;
    public CRServo glyphSlideS;

    DigitalChannel sensorDigitalMagnetic;
    ElapsedTime runtime = new ElapsedTime();

    int liftCounter; //liftCounter and resetJoystick are used for lift controls.
    boolean resetJoystick = false;

    private double lServoArmInit = 0.65; //Glyph arms will initialize in the open position.
    private double rServoArmInit = 0.4;
    private double lServoArmGrasp = 0; //Variables for glyph
    private double rServoArmGrasp = 1;
    private double lServoArmAlmostGrasp = 0.5;
    private double rServoArmAlmostGrasp = 0.6;

    private double relicGrab = 1;
    private double relicRelease = 0;
    private double relicDown = 0.9;
    private double relicRaise = 0.3;
    private double relicSlow1 = 0.2;
    private double relicSlow2 = 0.35;

    private double speedFactor = .7;
    private int controlMode = 1;
    private String relicToggle = "inOn";
    private String relicArmToggle = "inOn";

    private double pExtendArm = .3; //Partially extend the jewel arm.
    private double extendArm = .55; //Fully extend the jewel arm.
    private double retractArm = 0; //Bring jewel arm back to the robot.

    private double hitCenter = .5;   //jewelHitS returns to center variable
    private double hitLeft = 0;      //jewelHitS Hit left jewel variable
    private double hitPLeft = .25;   //Protects arm from falling in case of power outages.
    private double hitRight = 1;     //jewelHitS Hits right jewel variable

    boolean first = false;

    enum SlippyState {
        Idle, LookingMagnet, FoundMagnet                    //These are the different states that the
    }

    public void init () {

        lfDriveM = hardwareMap.dcMotor.get("lfDriveM");       //Left front drive, Hub 1, port 2
        lfDriveM.setPower(0);
        lbDriveM = hardwareMap.dcMotor.get("lbDriveM");       //Left back drive, Hub 1, port 3
        lbDriveM.setPower(0);
        rfDriveM = hardwareMap.dcMotor.get("rfDriveM");       //Right front drive, Hub 1, port 1
        rfDriveM.setPower(0);
        rbDriveM = hardwareMap.dcMotor.get("rbDriveM");       //Right back drive, Hub 1, port 0
        rbDriveM.setPower(0);

        glyphLiftM = hardwareMap.dcMotor.get("glyphLiftM");   //Lift motor, Hub 2, port 3
        glyphLiftM.setPower(0);

        lGlyphS = hardwareMap.servo.get("lGlyphS");     //Left servo arm, Hub 1, port 2
        lGlyphS.setPosition(lServoArmInit);
        rGlyphS = hardwareMap.servo.get("rGlyphS");     //Right servo arm, Hub 2, port 1
        rGlyphS.setPosition(rServoArmInit);

        glyphSlideS = hardwareMap.crservo.get("glyphSlideS");
        glyphSlideS.setPower(0);

        jewelExtendS = hardwareMap.servo.get("jewelExtendS");   //Hub 3 Servo 0
        jewelExtendS.setPosition(retractArm);
        jewelHitS = hardwareMap.servo.get("jewelHitS");     //Hub 2 Servo 4

        relicTurn = hardwareMap.dcMotor.get("relicTurnM");       //Relic Turn Motor, Hub 2, Port 0
        relicTurn.setPower(0);
        relicExtend = hardwareMap.dcMotor.get("relicExtendM");   //Relic Extend Motor, Hub 2, port 1 20
        relicExtend.setPower(0);
        relicGrabber = hardwareMap.servo.get("relicGrabberS");
        relicLower = hardwareMap.servo.get("relicLiftS");

        relicTurn.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        relicExtend.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lfDriveM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lbDriveM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rfDriveM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rbDriveM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        glyphLiftM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        glyphLiftM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        glyphLiftM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sensorDigitalMagnetic = hardwareMap.get(DigitalChannel.class, "SensorDigitalMagnetic");
        sensorDigitalMagnetic.setMode(DigitalChannel.Mode.INPUT);
    }

    private SlippyState slippystate = SlippyState.Idle;

    @Override
    public void start() {
        runtime.reset();
        lGlyphS.setPosition(0.65);
        rGlyphS.setPosition(0.4);

        jewelExtendS.setPosition(0);
        jewelHitS.setPosition(0.25);
    }

    public void loop() {
        if(gamepad1.right_bumper) {
            speedFactor = 1; //When the right bumper is held down, run the drive train at full capacity.
        } else {
            speedFactor = .65; //Otherwise, run at 65% power.
        }
        if(gamepad1.a) {    //Either button will dictate the drive train's control mode, shown in the switch case directly below.
            controlMode = 1;
        }
        else if(gamepad1.y) {
            controlMode = 2;
        }

///----------------------------------------DRIVER CODE----------------------------------------\\\

        ///DRIVE TRAIN CONTROLS

        switch(controlMode) {
            default: case 1: //Normal Tank Drive
                lfDriveM.setPower((-gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y)) * speedFactor); //Math.abs function allows for exponential scale drive.
                lbDriveM.setPower((-gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y)) * speedFactor); //Less input gives more control, more input gives more power.
                rfDriveM.setPower((gamepad1.right_stick_y * Math.abs(gamepad1.right_stick_y)) * speedFactor);
                rbDriveM.setPower((gamepad1.right_stick_y * Math.abs(gamepad1.right_stick_y)) * speedFactor);
                break;
            case 2: //
                lfDriveM.setPower((-gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y)) * speedFactor);
                lbDriveM.setPower((-gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y)) * speedFactor);
                rfDriveM.setPower((gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y)) * speedFactor);
                rbDriveM.setPower((gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y)) * speedFactor);
                break;
        }
        if (gamepad1.dpad_up || gamepad1.dpad_down) { //If either up or down is pressed on gamepad 1, set both sides of the drive train to the right joystick.
            gamepad1.left_stick_y = gamepad1.right_stick_y;
        }

///---------------------------------------OPERATOR CODE---------------------------------------\\\

        ///RELIC CONTROLS

        relicExtend.setPower(-gamepad2.right_stick_y);
        relicTurn.setPower(-gamepad2.left_stick_x/5);
        if (gamepad2.left_stick_x == 0 && !first) {
            relicTurn.setPower(-0.1);
            first = true;
        }
        if (gamepad2.dpad_right) {
            relicGrabber.setPosition(relicSlow1);
        } else if (gamepad2.dpad_left) {
            relicGrabber.setPosition(relicSlow2);
        }

        if (gamepad2.right_trigger > 0.01 && gamepad2.y){
            relicGrabber.setPosition(gamepad2.right_trigger);
        }
        if(gamepad2.left_trigger > 0.01 && gamepad2.y){
            relicLower.setPosition(gamepad2.left_trigger);
        }
        if (gamepad2.right_bumper && relicToggle == "inOn") {
            relicToggle = "outOff";
        } else if (relicToggle == "outOff" && !gamepad2.right_bumper) {
            relicToggle = "outOn";
            relicGrabber.setPosition(relicRelease);
        } else if (gamepad2.right_bumper && relicToggle == "outOn") {
            relicToggle = "inOff";
        } else if (relicToggle == "inOff" && !gamepad2.right_bumper) {
            relicToggle = "inOn";
            relicGrabber.setPosition(relicGrab);
        }

        if (gamepad2.left_bumper && relicArmToggle == "inOn") {
            relicArmToggle = "outOff";
        } else if (relicArmToggle == "outOff" && !gamepad2.left_bumper) {
            relicArmToggle = "outOn";
            relicLower.setPosition(relicDown);
        } else if (gamepad2.left_bumper && relicArmToggle == "outOn") {
            relicArmToggle = "inOff";
        } else if (relicArmToggle == "inOff" && !gamepad2.left_bumper) {
            relicArmToggle = "inOn";
            relicLower.setPosition(relicRaise);
        }
        /*if (gamepad2.left_bumper) {
            relicLower.setPosition(relicRaise);
        }
        else if (gamepad2.right_bumper) {
            relicLower.setPosition(relicDown);
        }
        if (gamepad2.y) {
            relicGrabber.setPosition(relicGrab);
        }
        else if (gamepad2.a) {
            relicGrabber.setPosition(relicRelease);
        }*/
        if (gamepad2.dpad_up) {
            if(relicLower.getPosition() == relicDown && relicDown > 0) {
                relicDown -= 0.01;
                relicLower.setPosition(relicDown);
                try { Thread.sleep(100); }
                catch (InterruptedException exc) { Thread.currentThread().interrupt(); }
            } else if(relicLower.getPosition() != relicDown) glyphLiftM.setPower(1);
        } else if (gamepad2.dpad_down) {
            if(relicLower.getPosition() == relicDown && relicDown < 1) {
                relicDown += 0.01;
                relicLower.setPosition(relicDown);
                try { Thread.sleep(100); }
                catch (InterruptedException exc) { Thread.currentThread().interrupt(); }
            } else if(relicLower.getPosition() != relicDown) glyphLiftM.setPower(-1);
        } else {
            glyphLiftM.setPower(0);
        }

        //LIFT CONTROLS

        if (gamepad1.left_stick_y < -0.01 && !resetJoystick) {
            liftCounter += 1;
            if (liftCounter > 4) {
                liftCounter = 4;
            }
            resetJoystick = true;
        } else if (gamepad1.left_stick_y > 0.01 && !resetJoystick) {
            liftCounter += -1;
            if (liftCounter < 0) {
                liftCounter = 0;
            }
            resetJoystick = true;
        }
        if (gamepad1.left_stick_y == 0 && resetJoystick) {
            resetJoystick = false;
        }

        switch (liftCounter) {
            case 0:
                glyphLiftM.setTargetPosition(0);
                glyphLiftM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if (glyphLiftM.isBusy()) {
                    glyphLiftM.setPower(1);
                    telemetry.addData("Current Position", glyphLiftM.getCurrentPosition());
                    telemetry.update();
                } else {
                    glyphLiftM.setPower(0);
                }
                break;
            case 1:
                glyphLiftM.setTargetPosition(300);
                glyphLiftM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if (glyphLiftM.isBusy()) {
                    glyphLiftM.setPower(1);
                    telemetry.addData("Current Position", glyphLiftM.getCurrentPosition());
                    telemetry.update();
                } else {
                    glyphLiftM.setPower(0);
                }
                break;
            case 2:
                glyphLiftM.setTargetPosition(1300);
                glyphLiftM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if (glyphLiftM.isBusy()) {
                    glyphLiftM.setPower(1);
                    telemetry.addData("Current Position", glyphLiftM.getCurrentPosition());
                    telemetry.update();
                } else {
                    glyphLiftM.setPower(0);
                }
                break;
            case 3:
                glyphLiftM.setTargetPosition(2300);
                glyphLiftM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if (glyphLiftM.isBusy()) {
                    glyphLiftM.setPower(1);
                    telemetry.addData("Current Position", glyphLiftM.getCurrentPosition());
                    telemetry.update();
                } else {
                    glyphLiftM.setPower(0);
                }
                break;
            case 4:
                glyphLiftM.setTargetPosition(3300);
                glyphLiftM.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                if (glyphLiftM.isBusy()) {
                    glyphLiftM.setPower(1);
                    telemetry.addData("Current Position", glyphLiftM.getCurrentPosition());
                    telemetry.update();
                } else {
                    glyphLiftM.setPower(0);
                }
                break;
        }

        //GLYPHER CONTROLS

        if (gamepad2.b) { //hitting the "b" button on Gamepad 2 will cause the glypher servos to grasp the glyph
            lGlyphS.setPosition(lServoArmGrasp);
            rGlyphS.setPosition(rServoArmGrasp);
        }
        if (gamepad2.x) { //hitting the "x" button on Gamepad 2 will cause the glypher servos to return to their original position
            lGlyphS.setPosition(lServoArmInit);
            rGlyphS.setPosition(rServoArmInit);
        }
        if (gamepad2.y) {   //hitting the "y" button on Gamepad 2 will cause the glypher servos to expand slightly larger than grasping the glyphs.
            lGlyphS.setPosition(lServoArmAlmostGrasp);
            rGlyphS.setPosition(rServoArmAlmostGrasp);
        }

        switch (slippystate){
            case Idle:              //The Idle case is the default case
                if ((gamepad2.right_trigger < 0.1 && gamepad2.left_trigger > 0.1) || gamepad1.right_trigger < 0.1 && gamepad1.left_trigger > 0.1){
                    glyphSlideS.setDirection(DcMotorSimple.Direction.FORWARD);
                    glyphSlideS.setPower(1);
                    //If the right trigger is pressed, the timing belt moves to the right
                }
                else if ((gamepad2.right_trigger > 0.1 && gamepad2.left_trigger < 0.1) || gamepad1.right_trigger > 0.1 && gamepad1.left_trigger < 0.1) {
                    glyphSlideS.setDirection(DcMotorSimple.Direction.REVERSE);
                    glyphSlideS.setPower(1);
                    //If the left trigger is pressed, the timing belt moves to the left
                }
                else if ((gamepad2.right_trigger > 0.1 && gamepad2.left_trigger > 0.1) || gamepad1.y) {
                    if (glyphSlideS.getDirection() == DcMotorSimple.Direction.FORWARD) {
                        glyphSlideS.setDirection(DcMotorSimple.Direction.REVERSE);
                        //if both triggers (operator) or the y button (driver) is pressed, the timing
                        // belt moves the opposite direction of whatever direction it last moved
                    } else {
                        glyphSlideS.setDirection(DcMotorSimple.Direction.FORWARD);
                    }
                    slippystate = slippystate.LookingMagnet;
                    //if both triggers (operator) or the y button (driver) is pressed, then switch to
                    //the Looking case
                }
                else {
                    glyphSlideS.setPower(0);
                }
                break;
            case LookingMagnet:    //The timing belt is moving until the magnetic sensor is triggered
                glyphSlideS.setPower(1);
                if (!sensorDigitalMagnetic.getState()) {
                    slippystate = slippystate.FoundMagnet;
                    //If the sensor is triggered, switch to Found case
                }
                if ((gamepad2.right_trigger > 0.1) !=(gamepad2.left_trigger >0.1) || gamepad1.y) {
                    glyphSlideS.setPower(0);
                    slippystate = slippystate.Idle;
                    //if either triggers (operator) or the y button (driver) is pressed, stop
                    //searching for the sensor and return to Idle case
                }
                break;
            case FoundMagnet:
                glyphSlideS.setPower(0);
                slippystate = slippystate.Idle;
                //Once the magnetic sensor has been triggered, don't move the timing belt and return
                //to Idle case
                break;
        }
    }
}