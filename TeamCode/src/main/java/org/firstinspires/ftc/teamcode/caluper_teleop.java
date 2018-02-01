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

/**
 * Created by jrahm on 6/14/17.
 */

@TeleOp (name="caluper_teleop",group="Opmode")
@Disabled
public class caluper_teleop extends OpMode {

    public DcMotor lfDriveM, rfDriveM, lbDriveM, rbDriveM, liftM;  //Left front drive, right front drive, left back drive, right back drive.
    public Servo lArmS, rArmS;
    public CRServo jewelArmS;            //lServoArm in port 5, rServoArm in port 4

    private double lServoArmInit = 1;                     //Glyph arms will initialize in the open position.
    private double rServoArmInit = 0;
    private double lServoArmGrasp = 0;                    //After testing, these positions were optimal for grasping the glyphs.
    private double rServoArmGrasp = 1;
    private double lServoArmAlmostGrasp = .5;
    private double rServoArmAlmostGrasp = .5;

    boolean rightBackwardBrake = false;                      //These four variables initiate an abrupt stop (see below).
    boolean rightForwardBrake = false;
    boolean leftBackwardBrake = false;
    boolean leftForwardBrake = false;

    boolean liftBrake = false;


    private double speedFactor = .5;
    private int controlMode = 1;

    ElapsedTime timer = new ElapsedTime();

    public void init () {
        lfDriveM = hardwareMap.dcMotor.get("lfDriveM");       //Left front drive, Hub 1, port 2
        lfDriveM.setPower(0);
        lfDriveM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lbDriveM = hardwareMap.dcMotor.get("lbDriveM");       //Left back drive, Hub 1, port 3
        lbDriveM.setPower(0);
        lbDriveM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rfDriveM = hardwareMap.dcMotor.get("rfDriveM");       //Right front drive, Hub 1, port 1
        rfDriveM.setPower(0);
        rfDriveM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rbDriveM = hardwareMap.dcMotor.get("rbDriveM");       //Right back drive, Hub 1, port 0
        rbDriveM.setPower(0);
        rbDriveM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftM = hardwareMap.dcMotor.get("liftM");   //Lift motor, Hub 2, port 3
        liftM.setPower(0);
        liftM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lArmS = hardwareMap.servo.get("lArmS");     //Left servo arm, Hub 1, port 2
        lArmS.setPosition(lServoArmInit);

        rArmS = hardwareMap.servo.get("rArmS");     //Right servo arm, Hub 2, port 1
        rArmS.setPosition(rServoArmInit);

        jewelArmS = hardwareMap.crservo.get("jewelArmS");       //Jewel Arm, Hub 2, port 3
        jewelArmS.setPower(0);
    }

    public void loop() {
        if(gamepad1.right_bumper) {
            speedFactor = 1;
        } else {
            speedFactor = .5;
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

        if (gamepad1.dpad_up) {
            gamepad1.left_stick_y = gamepad1.right_stick_y;
        }

        if (gamepad1.dpad_down) {
            gamepad1.left_stick_y = gamepad1.right_stick_y;
        }

        //ABRUPT STOP (Right Side)
        /*//Backwards
        if (gamepad1.right_stick_y > 0.01) {        //As soon as the joystick activates, set the brake variable to true.
            rightBackwardBrake = true;
        }

        if (gamepad1.right_stick_y == 0 && rightBackwardBrake == true) {
            timer.reset();
            rfDriveM.setPower(-.3);          //If the brake variable is true and the joystick resets to 0, the robot will reverse power briefly to cause an abrupt stop.
            rbDriveM.setPower(-.3);

            if (timer.milliseconds() > 150) {
                rfDriveM.setPower(0);
                rbDriveM.setPower(0);
            }
            rightBackwardBrake = false;     //As soon as the robot stops, reset the brake variable to false.
        }

        //Forwards
        if (gamepad1.right_stick_y < -0.01) {
            rightForwardBrake = true;
        }

        if (gamepad1.right_stick_y == 0 && rightForwardBrake == true) {
            timer.reset();
            rfDriveM.setPower(.3);
            rbDriveM.setPower(.3);

            if (timer.milliseconds() > 150) {
                rfDriveM.setPower(0);
                rbDriveM.setPower(0);
            }
            rightForwardBrake = false;
        }

        //ABRUPT STOP (Left Side)
        //Backwards
        if (gamepad1.left_stick_y < -0.01) {
            leftBackwardBrake = true;
        }

        if (gamepad1.left_stick_y == 0 && leftBackwardBrake == true) {
            timer.reset();
            lfDriveM.setPower(.3);
            lbDriveM.setPower(.3);

            if (timer.milliseconds() > 150) {
                lfDriveM.setPower(0);
                lbDriveM.setPower(0);
            }
            leftBackwardBrake = false;
        }

        //Forwards
        if (gamepad1.left_stick_y > 0.01) {
            leftForwardBrake = true;
        }

        if (gamepad1.left_stick_y == 0 && leftForwardBrake == true) {
            timer.reset();
            lfDriveM.setPower(-.3);
            lbDriveM.setPower(-.3);

            if (timer.milliseconds() > 150) {
                lfDriveM.setPower(0);
                lbDriveM.setPower(0);
            }
            leftForwardBrake = false;
        }*/

///OPERATOR CODE

        if (gamepad2.dpad_up) {
            liftM.setPower(1);
        }
        else if (gamepad2.dpad_down) {
            liftM.setPower(-1);
        }
        else {
            liftM.setPower(0);
            liftM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        if (gamepad2.dpad_up) {
            liftM.setPower(1);
        }
        else if (gamepad2.dpad_down) {
            liftM.setPower(-1);
        }
        else {
            liftM.setPower(0);
        }

/*

        if (gamepad1.right_stick_y == 0 && rightBackwardBrake == true) {
            timer.reset();
            rfDriveM.setPower(-.3);          //If the brake variable is true and the joystick resets to 0, the robot will reverse power briefly to cause an abrupt stop.
            rbDriveM.setPower(-.3);

            if (timer.milliseconds() > 150) {
                rfDriveM.setPower(0);
                rbDriveM.setPower(0);
            }
            rightBackwardBrake = false;     //As soon as the robot stops, reset the brake variable to false.
        }*/



        if (gamepad2.b) { //hitting the "b" button on Gamepad 2 will cause the glypher servos to grasp the glyph
            lArmS.setPosition(lServoArmGrasp);
            rArmS.setPosition(rServoArmGrasp);
        }
        if (gamepad2.x) { //hitting the "x" button on Gamepad 2 will cause the glypher servos to return to their original position
            lArmS.setPosition(lServoArmInit);
            rArmS.setPosition(rServoArmInit);
        }

        if (gamepad2.y) {   //hitting the "y" button on Gamepad 2 will cause the glypher servos to expand slightly larger than grasping the glyphs.
            lArmS.setPosition(lServoArmAlmostGrasp);
            rArmS.setPosition(rServoArmAlmostGrasp);
        }

        if (gamepad2.dpad_right) {
            jewelArmS.setDirection(DcMotorSimple.Direction.FORWARD);
            jewelArmS.setPower(1);
        } else if (gamepad2.left_bumper) {
            jewelArmS.setDirection(DcMotorSimple.Direction.REVERSE);
            jewelArmS.setPower(1);
        } else {
            jewelArmS.setPower(0);
        }

    }
}