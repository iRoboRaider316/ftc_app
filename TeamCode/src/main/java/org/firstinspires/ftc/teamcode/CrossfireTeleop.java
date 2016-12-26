package org.firstinspires.ftc.teamcode;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp(name="CrossFireTeleOp", group="TeleOp")
//@Disabled
public class CrossfireTeleop extends OpMode {
    private DcMotor rDrive1;
    private DcMotor rDrive2;
    private DcMotor lDrive1;
    private DcMotor lDrive2;
    private DcMotor catapult;
    private DcMotor sweeper;
    private DcMotor lift1;
    private DcMotor lift2;
    private DcMotor Button;
    private Servo hopper;
    private TouchSensor touch;

    // Function to reset the catapult to the launch position
    /*private void launchPosition() throws InterruptedException{
        while (!touch.isPressed() && opModeIsActive()){
            catapult.setPower(0.5);
        }
        catapult.setPower(0);
    }
    // Function that utlizes the launchPosition, handleBall, and launch functions to fire and reload the catapult
    private void fire() throws InterruptedException {
        launchBall();
        launchPosition();
        loadBall();
    }
    // Function to load the catapult
    private void loadBall() throws InterruptedException {
        hopper.setPosition(.5);
        Thread.sleep(1000);
        hopper.setPosition(.8);
    }
    // Fires the ball
    private void launchBall() throws InterruptedException {
        catapult.setPower(1);
        Thread.sleep(800);
        catapult.setPower(0);
    }*/

    @Override
    public void init(){
        rDrive1 = hardwareMap.dcMotor.get("rDrive1");
        rDrive2 = hardwareMap.dcMotor.get("rDrive2");
        lDrive1 = hardwareMap.dcMotor.get("lDrive1");
        lDrive2 = hardwareMap.dcMotor.get("lDrive2");
        sweeper = hardwareMap.dcMotor.get("sweeper");
        catapult = hardwareMap.dcMotor.get("catapult");
        lift1 = hardwareMap.dcMotor.get("lift1");
        lift2 = hardwareMap.dcMotor.get("lift2");
        Button = hardwareMap.dcMotor.get("Button");
        hopper = hardwareMap.servo.get("hopper");
        rDrive1.setDirection(DcMotor.Direction.REVERSE);
        rDrive2.setDirection(DcMotor.Direction.REVERSE);
        touch = hardwareMap.touchSensor.get("t");
    }

        public void loop() {
            float rStick1 = gamepad1.left_stick_y;
            float lStick1 = gamepad1.right_stick_y;
            //float lStick2 = gamepad2.left_stick_y;
            float rStick2 = gamepad2.right_stick_y;
            boolean up = gamepad2.dpad_up;
            boolean left = gamepad2.dpad_left;
            boolean down = gamepad2.dpad_down;
            boolean y = gamepad2.y;
            //boolean x = gamepad2.x;
            boolean a = gamepad2.a;
            boolean b = gamepad2.b;
            boolean rBumper1 = gamepad1.right_bumper;
            boolean lBumper1 = gamepad1.left_bumper;
            boolean rBumper2 = gamepad2.right_bumper;
            //boolean lBumper2 = gamepad2.left_bumper;
            float leftPower;
            float rightPower;
            float lTrigger2 = gamepad2.left_trigger;
            float rTrigger2 = gamepad2.right_trigger;

            ///OPERATOR CODE\\\
            Button.setPower(rTrigger2);

            // Set collection to on, off, or reversed
            if (up)
                sweeper.setPower(-1);
            else if (down)
                sweeper.setPower(1);
            else if (left)
                sweeper.setPower(0);

            // Manual catapult controls
            if (y)
                catapult.setPower(1);
            else if (a)
                catapult.setPower(-1);
            else
                catapult.setPower(0);

            // Manual catapult loading
            if (b)
                hopper.setPosition(.5);
            else
                hopper.setPosition(.8);

            /*if (rBumper2) {
                try {
                    fire();
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }*/

            lift1.setPower(-rStick2);
            lift2.setPower(-rStick2);

            ///DRIVER CODE\\\
            // This sets the power of the drive motors to based on the joystick position using an Exponential Scale Algorithm
            if (lBumper1) {
                leftPower = ((rStick1 * Math.abs(rStick1)) / 2);
                rightPower = ((lStick1 * Math.abs(lStick1)) / 2);
            } else if (rBumper1) {
                leftPower = ((rStick1 * Math.abs(rStick1)) / 4);
                rightPower = ((lStick1 * Math.abs(lStick1)) / 4);
            } else {
                leftPower = (rStick1 * Math.abs(rStick1));
                rightPower = (lStick1 * Math.abs(lStick1));
            }
            lDrive1.setPower(leftPower);
            lDrive2.setPower(leftPower);
            rDrive1.setPower(rightPower);
            rDrive2.setPower(rightPower);

            telemetry.addData("Hopper position", hopper.getPortNumber());
            telemetry.addData("Button power", Button.getPower());
        }
        }