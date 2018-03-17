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
 * Created by jrahm on 6/14/17.
 */
@TeleOp (name="legacy_teleop",group="Opmode")
@Disabled
public class legacy_teleop extends OpMode {

    public DcMotor lfDriveM, rfDriveM, lbDriveM, rbDriveM, glyphLiftM;  //Left front drive, right front drive, left back drive, right back drive.
    public Servo lGlyphS, rGlyphS, jewelExtendS, jewelHitS;
    public CRServo glyphSlideS;
    DigitalChannel sensorDigitalMagnetic;

    private double lServoArmInit = 0.65;                     //Glyph arms will initialize in the open position./
    private double rServoArmInit = 0.4;
    private double lServoArmGrasp = 0;                    //After testing, these positions were optimal for grasping the glyphs.
    private double rServoArmGrasp = 1;
    private double lServoArmAlmostGrasp = 0.5;
    private double rServoArmAlmostGrasp = 0.6;

    private double speedFactor = .65;
    private int controlMode = 1;

    private double pExtendArm = .3; //Partially extend the jewel arm.
    private double extendArm = .55; //Fully extend the jewel arm.
    private double retractArm = 0; //Bring jewel arm back to the robot.

    private double hitCenter = .5;   //jewelHitS returns to center variable
    private double hitLeft = 0;      //jewelHitS Hit left jewel variable
    private double hitPLeft = .25;   //Protects arm from falling in case of power outages.
    private double hitRight = 1;     //jewelHitS Hits right jewel variable

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
        jewelHitS.setPosition(hitPLeft);

        lfDriveM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lbDriveM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rfDriveM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rbDriveM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        glyphLiftM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        sensorDigitalMagnetic = hardwareMap.get(DigitalChannel.class, "SensorDigitalMagnetic");
        sensorDigitalMagnetic.setMode(DigitalChannel.Mode.INPUT);
    }

    private SlippyState slippystate = SlippyState.Idle;

    public void loop() {
        if(gamepad1.right_bumper) {
            speedFactor = 1;
        } else {
            speedFactor = .65;
        }
        if(gamepad1.a) {            //Hitting the "a" button will allow the drive train to run at full capacity.
            controlMode = 1;
        }
        else if(gamepad1.y) {     //Hitting the "y" button will cut the drive train's power by 50%.
            controlMode = 2;
        }
///DRIVER CODE
        switch(controlMode) {
            case 1:
                lfDriveM.setPower((-gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y)) * speedFactor); //exponential scale algorithm
                lbDriveM.setPower((-gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y)) * speedFactor); //tank drive
                rfDriveM.setPower((gamepad1.right_stick_y * Math.abs(gamepad1.right_stick_y)) * speedFactor);
                rbDriveM.setPower((gamepad1.right_stick_y * Math.abs(gamepad1.right_stick_y)) * speedFactor);
                break;
            case 2:
                lfDriveM.setPower((-gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y)) * speedFactor); //exponential scale algorithm
                lbDriveM.setPower((-gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y)) * speedFactor); //tank drive
                rfDriveM.setPower((gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y)) * speedFactor);
                rbDriveM.setPower((gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y)) * speedFactor);
                break;
            default:
                lfDriveM.setPower((-gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y)) * speedFactor); //exponential scale algorithm
                lbDriveM.setPower((-gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y)) * speedFactor); //tank drive
                rfDriveM.setPower((gamepad1.right_stick_y * Math.abs(gamepad1.right_stick_y)) * speedFactor);
                rbDriveM.setPower((gamepad1.right_stick_y * Math.abs(gamepad1.right_stick_y)) * speedFactor);
                break;
        }
        if (gamepad1.dpad_up || gamepad1.dpad_down) {
            gamepad1.left_stick_y = gamepad1.right_stick_y;
        }

///OPERATOR CODE
        if (gamepad2.dpad_up) {
            glyphLiftM.setPower(1);
        } else if (gamepad2.dpad_down) {
            glyphLiftM.setPower(-1);
        } else {
            glyphLiftM.setPower(0);
        }

        if (gamepad2.dpad_up) {
            glyphLiftM.setPower(1);
        } else if (gamepad2.dpad_down) {
            glyphLiftM.setPower(-1);
        } else {
            glyphLiftM.setPower(0);
        }

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