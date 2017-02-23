package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="Encoder Turn", group="Methods")

public class cf_encoder_turn extends LinearOpMode {

    DcMotor lDrive1;
    DcMotor lDrive2;
    DcMotor rDrive1;
    DcMotor rDrive2;

    // This is the Drive Method
    // It will take in two static values: distance and maxSpeed.
    // It will then calculate the encoder counts to drive and drive the distance at the specified power,
    // accelerating to max speed for the first third of the distance, maintaining that speed for the second third,
    // and decelerating to a minimum speed for the last third.
    // If the robot deviates from the initial gyro heading, it will correct itself proportionally to the error.
    private void encoderTurn(int degrees, double maxSpeed, int direction) throws InterruptedException {
        // one full rotation = 2200 encoder ticks
        // 1 degree = 6.11 encoder ticks
        double ticks = degrees * 6;
        double speed = 0;
        double leftSpeed;
        double rightSpeed;

        double start = rDrive1.getCurrentPosition();

        rDrive1.setTargetPosition(rDrive1.getCurrentPosition() + (int) ticks);

        rDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        sleep(500);

        // right turn
        if (direction == 1) {
            while (Math.abs(rDrive1.getCurrentPosition()) < Math.abs(rDrive1.getTargetPosition() - 5) && opModeIsActive()) {

                lDrive1.setPower(maxSpeed);
                lDrive2.setPower(maxSpeed);
                rDrive1.setPower(-maxSpeed);
                rDrive2.setPower(-maxSpeed);

                telemetry.addData("1. start", start);
                telemetry.addData("2. current", rDrive1.getCurrentPosition());
                updateTelemetry(telemetry);
            }

            lDrive1.setPower(0);
            rDrive1.setPower(0);
            lDrive2.setPower(0);
            rDrive2.setPower(0);
        }
        else if (direction == -1) {
            while (Math.abs(rDrive1.getCurrentPosition()) < Math.abs(rDrive1.getTargetPosition() - 5) && opModeIsActive()) {
                lDrive1.setPower(-maxSpeed);
                lDrive2.setPower(-maxSpeed);
                rDrive1.setPower(maxSpeed);
                rDrive2.setPower(maxSpeed);

                telemetry.addData("1. start", start);
                telemetry.addData("2. current", rDrive1.getCurrentPosition());
                updateTelemetry(telemetry);
            }

            lDrive1.setPower(0);
            rDrive1.setPower(0);
            lDrive2.setPower(0);
            rDrive2.setPower(0);

        }
        else {
            telemetry.addLine("Invalid direction");
            telemetry.update();
            sleep(10000);
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        double distance;
        double maxSpeed;
        int degrees;
        int direction;

        lDrive1 = hardwareMap.dcMotor.get("lDrive1");
        lDrive2 = hardwareMap.dcMotor.get("lDrive2");
        rDrive1 = hardwareMap.dcMotor.get("rDrive1");
        rDrive2 = hardwareMap.dcMotor.get("rDrive2");
        lDrive1.setDirection(DcMotor.Direction.REVERSE);
        lDrive2.setDirection(DcMotor.Direction.REVERSE);

        lDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        lDrive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rDrive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        idle();

        waitForStart();

        degrees = 90;
        maxSpeed = .3;
        direction = -1;
        encoderTurn(degrees, maxSpeed, direction);
        sleep(5000);


    }
}
