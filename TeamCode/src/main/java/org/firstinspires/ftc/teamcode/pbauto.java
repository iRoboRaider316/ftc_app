package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="pbauto", group="LinearOPMode")

public class pbauto extends LinearOpMode {
    private DcMotor l1;
    private DcMotor r1;
//    private Servo sweeper;


    // This is the Drive Method
    // It will take in two static values: distance and maxSpeed.
    // It will then calculate the encoder counts to drive and drive the distance at the specified power,
    // accelerating to max speed for the first third of the distance, maintaining that speed for the second third,
    // and decelerating to a minimum speed for the last third.
    // If the robot deviates from the initial gyro heading, it will correct itself proportionally to the error.
    private void encoderDrive(double distance, double maxSpeed, int direction) throws InterruptedException {
        int ENCODER_CPR = 1120; // Encoder counts per Rev
        double gearRatio = 1.75; // [Gear Ratio]:1
        double circumference = 13.10; // Wheel circumference
        double ROTATIONS = distance / (circumference * gearRatio); // Number of rotations to drive
        double COUNTS = ENCODER_CPR * ROTATIONS; // Number of encoder counts to drive

        r1.setTargetPosition(r1.getCurrentPosition() + (int) COUNTS);

        if (direction == 1) {
            while (r1.getCurrentPosition() < r1.getTargetPosition() - 5 && opModeIsActive()) {
                drive(maxSpeed, maxSpeed);
                telemetry.addData("1. speed", maxSpeed);
                updateTelemetry(telemetry);
            }
            driveStop();
        }
        else if (direction == -1) {
            while (Math.abs(r1.getCurrentPosition()) < Math.abs(r1.getTargetPosition() - 5) && opModeIsActive()) {
                drive(-maxSpeed, -maxSpeed);
                telemetry.addData("1. speed", maxSpeed);
                updateTelemetry(telemetry);
            }
            driveStop();
        }
        else {
            telemetry.addLine("Invalid direction");
            telemetry.update();
            sleep(10000);
        }
    }

    private void drive(double leftSpeed, double rightSpeed){
        l1.setPower(leftSpeed);
        r1.setPower(rightSpeed);
    }

    private void driveStop(){
        l1.setPower(0);
        r1.setPower(0);
    }

    public void runOpMode() throws InterruptedException {
        r1 = hardwareMap.dcMotor.get("r1");
        l1 = hardwareMap.dcMotor.get("l1");
//        l1.setDirection(DcMotor.Direction.REVERSE);

        double distance;
        double maxSpeed;
        int direction;

        waitForStart();

        //sleep(10000);
        distance = 25;
        maxSpeed = .25;
        direction = 1;
        encoderDrive(distance, maxSpeed, direction);
        sleep(1000);
        direction = -1;
        encoderDrive(distance, maxSpeed, direction);
    }
}
