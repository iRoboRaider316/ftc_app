package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="aftershock_teleop", group="TeleOp")
@Disabled
public class aftershock_teleop extends OpMode{

    public DcMotor lfDriveM, lbDriveM, rfDriveM, rbDriveM, glyphLiftM;
    public Servo lGlyphS, rGlyphS, jewelExtendS, jewelKnockS;

    int power = 1;

    boolean halfSpeed = false;
    boolean fullSpeed = false;

    double floorLeft;
    double floorRight;

    private double lGlyphSInit = .73;                     //Glyph arms will initialize in the open position.
    private double rGlyphSInit = .1;
    private double lGlyphSGrasp = .43;                    //After testing, these positions were optimal for grasping the glyphs.
    private double rGlyphSGrasp = .50;
    private double lGlyphSAlmostGrasp = .50;
    private double rGlyphSAlmostGrasp = .43;

    private double getDirection(double inputPower) {
        if(inputPower == 0) {
            return 1;
        } else {
            return inputPower * Math.abs(inputPower);
        }
    }

    int drive = 0;



    public void init() {
        lfDriveM = hardwareMap.dcMotor.get("lfDriveM");
        lbDriveM = hardwareMap.dcMotor.get("lbDriveM");
        rfDriveM = hardwareMap.dcMotor.get("rfDriveM");
        rbDriveM = hardwareMap.dcMotor.get("rbDriveM");

        glyphLiftM = hardwareMap.dcMotor.get("glyphLiftM");

        lGlyphS = hardwareMap.servo.get("lGlyphS");
        rGlyphS = hardwareMap.servo.get("rGlyphS");


//        jewelExtendS = hardwareMap.servo.get("jewelExtendS");
//        jewelKnockS = hardwareMap.servo.get("jewelKnockS");


        rfDriveM.setDirection(DcMotor.Direction.REVERSE);
        rbDriveM.setDirection(DcMotor.Direction.REVERSE);
    }

    public void loop() {
        float rStick1 = gamepad1.right_stick_y;
        float lStick1 = gamepad1.left_stick_y;

        boolean lBumper1 = gamepad1.left_bumper;
        boolean rBumper1 = gamepad1.right_bumper;

        double leftPower;
        double rightPower;
        double speed;

        ///OPERATOR CODE

        if (gamepad2.dpad_up) {
            glyphLiftM.setPower(1);
        }
        else if (gamepad2.dpad_down) {
            glyphLiftM.setPower(-1);
        }
        else {
            glyphLiftM.setPower(0);
        }
        if (gamepad2.dpad_up) {
            glyphLiftM.setPower(1);
        }
        else if (gamepad2.dpad_down) {
            glyphLiftM.setPower(-1);
        }
        else {
            glyphLiftM.setPower(0);
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

        ///DRIVER CODE\\\


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


        // Clips off the power in case it's over 1 or under -1 from the floor-ceiling function.
        leftPower = Range.clip(leftPower, -1, 1);
        rightPower = Range.clip(rightPower, -1, 1);

        lfDriveM.setPower(leftPower);
        lbDriveM.setPower(leftPower);
        rfDriveM.setPower(rightPower);
        rbDriveM.setPower(rightPower);
    }
}
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;