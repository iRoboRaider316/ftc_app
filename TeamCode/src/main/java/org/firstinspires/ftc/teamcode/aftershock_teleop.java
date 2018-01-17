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

@TeleOp(name="aftershock_teleop", group="TeleOp")

public class aftershock_teleop extends OpMode{

    public DcMotor lfDriveM, lbDriveM, rfDriveM, rbDriveM, glyphLiftM;
    public Servo lGlyphS, rGlyphS, jewelExtendS, jewelKnockS;
    public CRServo glyphSlideS;

    int power = 1;

    boolean halfSpeed = false;
    boolean fullSpeed = false;

    boolean rightBackwardBrake = false;                      //These four variables initiate an abrupt stop (see below).
    boolean rightForwardBrake = false;
    boolean leftBackwardBrake = false;
    boolean leftForwardBrake = false;

    double floorLeft;
    double floorRight;

    private double lGlyphSInit = 0;               //Glyph arms will initialize in the open position.
    private double rGlyphSInit = 1;
    private double lGlyphSGrasp = 1;              //After testing, these positions were optimal for grasping the glyphs.
    private double rGlyphSGrasp = 0;
    private double lGlyphSAlmostGrasp = .5;
    private double rGlyphSAlmostGrasp = .5;

    private double getDirection(double inputPower) {
        if(inputPower == 0) {
            return 1;
        } else {
            return inputPower * Math.abs(inputPower);
        }
    }

    int drive = 0;

    ElapsedTime timer = new ElapsedTime();

    public void init() {
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

/*
        jewelExtendS = hardwareMap.servo.get("jewelExtendS"); //Hub 3 Servo 5
        jewelKnockS = hardwareMap.servo.get("jewelKnockS"); //Hub 2 Servo 4
*/

        lfDriveM.setDirection(DcMotor.Direction.REVERSE);       //Reverse the left side of the drive
        lbDriveM.setDirection(DcMotor.Direction.REVERSE);       //train for intuitive human interface
    }

    public void loop() {
        float rStick1 = gamepad1.right_stick_y;     //Simplify names in the code.
        float lStick1 = gamepad1.left_stick_y;

        boolean lBumper1 = gamepad1.left_bumper;
        boolean rBumper1 = gamepad1.right_bumper;

        double leftPower;       //Used for switch-case (see below).
        double rightPower;
        double speed;

///-----------------------------------------OPERATOR CODE-----------------------------------------\\\

        if (gamepad2.dpad_up) {            //Hit up on the d-pad to lift the glypher.
            glyphLiftM.setPower(1);
        } else if (gamepad2.dpad_down) {   //Hit down on the d-pad to lower the glypher.
            glyphLiftM.setPower(-1);
        } else {                           //If neither button is pressed, stop the motor.
            glyphLiftM.setPower(0);
        }

        if (gamepad2.left_trigger > 0.1) {     //If left trigger is pushed, set power to full forward.
            glyphSlideS.setDirection(DcMotorSimple.Direction.FORWARD);
            glyphSlideS.setPower(1);
        } else if (gamepad2.right_trigger > 0.1) {  //If right trigger is pushed, set power to full forward.0
            glyphSlideS.setDirection(DcMotorSimple.Direction.REVERSE);
            glyphSlideS.setPower(1);
        } else {
            glyphSlideS.setPower(0);            //Otherwise, set power to 0 (stationary).
        }

        if (gamepad2.b) { //hitting the "b" button on Gamepad 2 will cause the glypher servos to grasp the glyph
            lGlyphS.setPosition(lGlyphSGrasp);
            rGlyphS.setPosition(rGlyphSGrasp);
        }
        if (gamepad2.x) { //hitting the "x" button on Gamepad 2 will cause the glypher servos to return to their original position
            lGlyphS.setPosition(lGlyphSInit);
            rGlyphS.setPosition(rGlyphSInit);
        }

        if (gamepad2.y) {   //hitting the "y" button on Gamepad 2 will cause the glypher servos to expand slightly larger than grasping the glyphs.
            lGlyphS.setPosition(lGlyphSAlmostGrasp);
            rGlyphS.setPosition(rGlyphSAlmostGrasp);
        }

//        if (gamepad1.a) {
//            jewelExtendS.setPosition(1);
//        }


///------------------------------------------DRIVER CODE------------------------------------------\\\


        // Universal drive train power switch case
        if (rBumper1 && power == 1) {
            halfSpeed = true;
        } else if (halfSpeed && !rBumper1) {
            power = 2;
            halfSpeed = false;
        } else if (rBumper1 && power == 2) {
            fullSpeed = true;
        } else if (fullSpeed && !rBumper1) {
            power = 1;
            fullSpeed = false;
        }

        switch (power) {
            case 1:
                // divide all powers by 1 (full speed)
                speed = 1;
                power = 1;
                break;
            case 2:
                // divide all powers by 2 (half speed)
                speed = 2;
                power = 2;
                break;
            default:
                speed = 1;
                break;
        }

        // This sets the power of the drive motors to based on the joystick position using an Exponential Scale Algorithm
        if (gamepad1.a) { // Backward 100%
            drive = 1;
        }
        if (gamepad1.y) { //Forward 100%
            drive = 2;
        }
        if (rBumper1) { //1/2 speed
            speed = 2;
        }
        if (lBumper1) { //full speed
            speed = 1;
        }
        if (gamepad1.dpad_up) { //Forward in a straight line
            drive = 3;
        }
        if (gamepad1.dpad_down) { // Backward in a straight line
            drive = 4;
        }

        // Floor-ceiling algorithm to account for motor stall torque and
        // increase the resolution of driving
        floorLeft = lStick1 <= 0.01 && lStick1 >= -0.01 ? 0 : 0.2 * getDirection(lStick1);
        floorRight = rStick1 <= 0.01 && rStick1 >= -0.01 ? 0 : 0.2 * getDirection(rStick1);

        // Switchcase for drivetrain direction and power. Floor-Ceiling algorithms
        // have been included for greater control
        switch (drive) {
            case 1:
                // forward default
                rightPower = ((-lStick1 * Math.abs(lStick1)) / speed) - floorLeft;
                leftPower = ((-rStick1 * Math.abs(rStick1)) / speed) - floorRight;
                drive = 1;
                break;
            case 2:
                // backward
                rightPower = ((rStick1 * Math.abs(rStick1)) / speed) + floorRight;
                leftPower = ((lStick1 * Math.abs(lStick1)) / speed) + floorLeft;
                drive = 2;
                break;
            case 3:
                rightPower = (Math.abs(rStick1) * -1) - floorRight;
                leftPower = (Math.abs(rStick1) * -1) - floorRight;
                drive = 1;
                break;
            case 4:
                rightPower = Math.abs(rStick1) + floorRight;
                leftPower = Math.abs(rStick1) + floorRight;
                drive = 1;
                break;

            default:
                leftPower = ((lStick1 * Math.abs(lStick1)) / speed) + floorLeft;
                rightPower = ((rStick1 * Math.abs(rStick1)) / speed) + floorRight;
        }

        //ABRUPT STOP (Right Side)
        //Backwards
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
        }


        // Clips off the power in case it's over 1 or under -1 from the floor-ceiling function.
        leftPower = Range.clip(leftPower, -1, 1);
        rightPower = Range.clip(rightPower, -1, 1);

        lfDriveM.setPower(leftPower);
        lbDriveM.setPower(leftPower);
        rfDriveM.setPower(rightPower);
        rbDriveM.setPower(rightPower);
    }
}