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
@TeleOp (name="legacy_teleop",group="Opmode")
public class legacy_teleop extends OpMode {
    public DcMotor lfDriveM, rfDriveM, lbDriveM, rbDriveM, liftM;  //Left front drive, right front drive, left back drive, right back drive.
    public Servo lArmS, rArmS;
    public CRServo glyphSlideS;
    //lServoArm in port 5, rServoArm in port 4
    private double lServoArmInit = 0.5;                     //Glyph arms will initialize in the open position./
    private double rServoArmInit = 0.5;
    private double lServoArmGrasp = 0;                    //After testing, these positions were optimal for grasping the glyphs.
    private double rServoArmGrasp = 1;
    private double lServoArmAlmostGrasp = .25;
    private double rServoArmAlmostGrasp = .75;
    private double speedFactor = .65;
    private int controlMode = 1;
    private double floorLeft;
    private double floorRight;

    private double getDirection(double inputPower) {
        if(inputPower == 0) {
            return 1;
        } else {
            return inputPower / Math.abs(inputPower);
        }
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
        liftM = hardwareMap.dcMotor.get("liftM");   //Lift motor, Hub 2, port 3
        liftM.setPower(0);
        lArmS = hardwareMap.servo.get("lArmS");     //Left servo arm, Hub 1, port 2
        lArmS.setPosition(lServoArmInit);
        rArmS = hardwareMap.servo.get("rArmS");     //Right servo arm, Hub 2, port 1
        rArmS.setPosition(rServoArmInit);
        glyphSlideS = hardwareMap.crservo.get("glyphSlideS");
        glyphSlideS.setPower(0);
        lfDriveM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lbDriveM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rfDriveM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rbDriveM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lfDriveM.setDirection(DcMotorSimple.Direction.REVERSE);
        lbDriveM.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public void loop() {
        double leftPower;       //Used for switch-case (see below).
        double rightPower;

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

        // Floor-ceiling algorithm to account for motor stall torque and
        // increase the resolution of driving
        floorLeft = gamepad1.left_stick_y <= 0.01 && gamepad1.left_stick_y >= -0.01 ? 0 : 0.2 * getDirection(gamepad1.left_stick_y);
        floorRight = gamepad1.right_stick_y <= 0.01 && gamepad1.right_stick_y >= -0.01 ? 0 : 0.2 * getDirection(gamepad1.right_stick_y);

///DRIVER CODE
        switch(controlMode) {
            case 1:
                rightPower = ((-gamepad1.right_stick_y * Math.abs(gamepad1.right_stick_y)) * speedFactor) - floorLeft;
                leftPower = ((-gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y)) * speedFactor) - floorRight;
                break;
            case 2:
                rightPower = ((gamepad1.right_stick_y * Math.abs(gamepad1.right_stick_y)) * speedFactor) + floorLeft;
                leftPower = ((gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y)) * speedFactor) + floorRight;
                break;
            default:
                rightPower = ((-gamepad1.right_stick_y * Math.abs(gamepad1.right_stick_y)) * speedFactor) - floorLeft;
                leftPower = ((-gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y)) * speedFactor) - floorRight;
                break;
        }
        if (gamepad1.dpad_up) {
            leftPower = rightPower;
        }
        if (gamepad1.dpad_down) {
            leftPower = rightPower;
        }

        // Clips off the power in case it's over 1 or under -1 from the floor-ceiling function.
        leftPower = Range.clip(leftPower, -1, 1);
        rightPower = Range.clip(rightPower, -1, 1);

        lfDriveM.setPower(leftPower);
        lbDriveM.setPower(leftPower);
        rfDriveM.setPower(rightPower);
        rbDriveM.setPower(rightPower);

///OPERATOR CODE
        if (gamepad2.dpad_up) {
            liftM.setPower(1);
        } else if (gamepad2.dpad_down) {
            liftM.setPower(-1);
        } else {
            liftM.setPower(0);
        }

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

        if (gamepad1.left_trigger > 0.1) {     //If left trigger is pushed, set power to full forward.
            glyphSlideS.setDirection(DcMotorSimple.Direction.FORWARD);
            glyphSlideS.setPower(1);
        } else if (gamepad1.right_trigger > 0.1) {  //If right trigger is pushed, set power to full forward.0
            glyphSlideS.setDirection(DcMotorSimple.Direction.REVERSE);
            glyphSlideS.setPower(1);
        } else if (gamepad2.right_trigger > 0.1) {  //If right trigger is pushed, set power to full forward.0
            glyphSlideS.setDirection(DcMotorSimple.Direction.REVERSE);
            glyphSlideS.setPower(1);
        } else if (gamepad2.left_trigger > 0.1) {  //If left trigger is pushed, set power to full forward.0
            glyphSlideS.setDirection(DcMotorSimple.Direction.FORWARD);
            glyphSlideS.setPower(1);
        } else {
            glyphSlideS.setPower(0);            //Otherwise, set power to 0 (stationary).
        }
    }
}