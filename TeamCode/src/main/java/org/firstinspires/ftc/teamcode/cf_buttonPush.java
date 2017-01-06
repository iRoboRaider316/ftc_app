package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="cf_button_push", group="LinearOpMode")

public class cf_buttonPush extends LinearOpMode {

    private DcMotor catapult;
    //private DcMotor sweeper;
    private DcMotor lDrive1;
    private DcMotor lDrive2;
    private DcMotor rDrive1;
    private DcMotor rDrive2;
    private Servo button;
    private Servo hopper;

    private TouchSensor touch;
    private GyroSensor gyroSensor;
    private ModernRoboticsI2cGyro gyro;
    private ColorSensor color;

    private void drive(double distance, double maxSpeed) throws InterruptedException {
        int ENCODER_CPR = 1120; // Encoder counts per Rev
        double gearRatio = 1.75; // [Gear Ratio]:1
        double circumference = 13.10; // Wheel circumference
        double ROTATIONS = distance / (circumference * gearRatio); // Number of rotations to drive
        double COUNTS = ENCODER_CPR * ROTATIONS; // Number of encoder counts to drive
        //double startPosition = rDrive1.getCurrentPosition();

        //double oldSpeed;
        double speed = 0;
        //double minSpeed = 0.3;
        //double acceleration = 0.01;
        double leftSpeed;
        double rightSpeed;
        double error;

        rDrive1.setTargetPosition(rDrive1.getCurrentPosition() + (int) COUNTS);

        rDrive1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lDrive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double heading = gyroSensor.getHeading();
        sleep(500);

        while (Math.abs(rDrive1.getCurrentPosition())<Math.abs(rDrive1.getTargetPosition()-5)&&opModeIsActive()){

            // Calculate distance from original heading and divide by 10
            error = (gyroSensor.getHeading()-heading);
            // Deal with wraparound from 359 to 0
            if (error >180)
                error = ((gyroSensor.getHeading()-heading-360)/30);
            else if(error <-180)
                error = ((gyroSensor.getHeading()-heading+360)/30);
            else
                error = ((gyroSensor.getHeading()-heading)/30);

            leftSpeed = maxSpeed-error;
            rightSpeed = maxSpeed+error;

            leftSpeed = Range.clip(leftSpeed, -1, 1);
            rightSpeed = Range.clip(rightSpeed, -1, 1);

            lDrive1.setPower(leftSpeed);
            rDrive1.setPower(rightSpeed);
            lDrive2.setPower(leftSpeed);
            rDrive2.setPower(rightSpeed);

            telemetry.addData("1. speed", speed);
            telemetry.addData("2. leftSpeed", leftSpeed);
            telemetry.addData("3. rightSpeed", rightSpeed);
            telemetry.addData("4. IntegratedZValue", gyro.getIntegratedZValue());
            updateTelemetry(telemetry);
        }

        lDrive1.setPower(0);
        rDrive1.setPower(0);
        lDrive2.setPower(0);
        rDrive2.setPower(0);

        lDrive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rDrive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    private void setUpGyro() throws InterruptedException {

        // setup the Gyro
        // write some device information (connection info, name and type)
        // to the log file.
        hardwareMap.logDevices();
        // get a reference to our GyroSensor object.
        gyroSensor = hardwareMap.gyroSensor.get("gyro");
        gyro = (ModernRoboticsI2cGyro) gyroSensor;
        // calibrate the gyro.
        gyroSensor.calibrate();
        while (gyroSensor.isCalibrating())  {
            sleep(50);
            telemetry.addLine("Calibrating Gyro");
            updateTelemetry(telemetry);
        }
        // End of setting up Gyro
    }

    public void buttonPush() throws InterruptedException {
        double distance;
        double maxSpeed;

        color.enableLed(false);
        //if the beacon is red
        if (color.blue() > color.red()){
            //insert code here
            ButtonPush();
            telemetry.addData("Color is red",color.red());
            sleep(1000);
        }
        //if the beacon is blue
        else if (color.red() > color.blue()) {
        drive(4,0.4);
            sleep(500);
            ButtonPush();
            telemetry.addData("Color is blue", color.blue());
            sleep(1000);
        }


    }

    public void ButtonPush() throws InterruptedException {
        button.setPosition(1);
        sleep(1000);
    }


    public void runOpMode() throws InterruptedException {
        color = hardwareMap.colorSensor.get("color");
        button = hardwareMap.servo.get("button");
        rDrive1 = hardwareMap.dcMotor.get("rDrive1");
        rDrive2 = hardwareMap.dcMotor.get("rDrive2");
        lDrive1 = hardwareMap.dcMotor.get("lDrive1");
        lDrive2 = hardwareMap.dcMotor.get("lDrive2");
        double distance;
        double maxSpeed;
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
        setUpGyro();
        waitForStart();
        telemetry.addData("Color is red",color.red());
        telemetry.addData("Color is blue", color.blue());
        buttonPush();
        sleep(3000);
    }
}