package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="cf_4_ball_red_NSR", group="LinearOPMode")
@Disabled

public class cf_NSR_4_ball extends LinearOpMode {
    private DcMotor catapult;
    //private DcMotor sweeper;
    private DcMotor lDrive1;
    private DcMotor lDrive2;
    private DcMotor rDrive1;
    private DcMotor rDrive2;
    private DcMotor sweeper;
    private Servo belt;
    private Servo button;
    private Servo hopper;
    private Servo wheels;
    private TouchSensor touch;

    GyroSensor gyroSensor;
    ModernRoboticsI2cGyro gyro;

    // Function to set up the Gyro
    // Function called in the init
    // Calibrates and does other preparations for the gyro sensor before autonomous
    // Needs nothing passed to it
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
        }
        // End of setting up Gyro
    }

    // Function that utilizes the launchPosition, handleBall, and launch functions to fire and reload the catapult
    private void fire() throws InterruptedException {
        sleep(1000);
        launchPosition();
        launchBall();
        loadBall();
        launchPosition();
        launchBall();
        launchPosition();
    }
    // Resets catapult to the launch position
    private void launchPosition() throws InterruptedException{
        while (!touch.isPressed()){
            catapult.setPower(0.5);
        }
        catapult.setPower(0);
    }
    // Function to load the catapult
    private void loadBall() throws InterruptedException {
        hopper.setPosition(.5);
        sleep(1000);
        hopper.setPosition(.8);
    }
    // Fires the ball
    private void launchBall() throws InterruptedException {
        catapult.setPower(1);
        sleep(800);
        catapult.setPower(0);
    }

    private void drive(double leftSpeed, double rightSpeed){
        lDrive1.setPower(leftSpeed);
        lDrive2.setPower(leftSpeed);
        rDrive1.setPower(rightSpeed);
        rDrive2.setPower(rightSpeed);
    }
    private void driveStop(){
        drive(0,0);
    }

    private void noEncoders(){
        lDrive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rDrive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while(lDrive1.isBusy()){
            sleep(10);
        }
    }
    private void useEncoders(){
        rDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while(lDrive1.isBusy()){
            sleep(10);
        }
    }
    private void resetEncoders(){
        lDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        while(lDrive1.isBusy()){
            sleep(10);
        }
    }

    private void encoderDrive(double distance, double leftSpeed, double rightSpeed, int direction) throws InterruptedException {
        int ENCODER_CPR = 1120; // Encoder counts per Rev
        double gearRatio = 1.75; // [Gear Ratio]:1
        double circumference = 13.10; // Wheel circumference
        double ROTATIONS = distance / (circumference * gearRatio); // Number of rotations to drive
        double COUNTS = ENCODER_CPR * ROTATIONS; // Number of encoder counts to drive

        // set the position we want to run to
        if (direction == 1)
            rDrive1.setTargetPosition(rDrive1.getCurrentPosition() + (int) COUNTS);
        else if (direction ==-1)
            rDrive1.setTargetPosition(rDrive1.getCurrentPosition() - (int) COUNTS);


        if (direction == 1) {
            // drive forward until we reach the encoder target
            while (rDrive1.getCurrentPosition() < rDrive1.getTargetPosition() - 5 && opModeIsActive()) {
                drive(leftSpeed, rightSpeed);
                telemetry.addData("1. left speed", leftSpeed);
                telemetry.addData("2. right speed", rightSpeed);
                updateTelemetry(telemetry);
            }
            driveStop();
        }
        else if (direction == -1) {
            // drive backward until we reach the encoder target
            while (rDrive1.getCurrentPosition() > rDrive1.getTargetPosition() + 5 && opModeIsActive()) {
                drive(-leftSpeed, -rightSpeed);
                telemetry.addData("1. left speed", leftSpeed);
                telemetry.addData("2. right speed", rightSpeed);
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

    private void gyroTurn (int targetHeading){
        boolean done = false;
        double error;
        double currentHeading;
        double kp = .0035;
        double power;
        gyro.resetZAxisIntegrator();
        sleep(250);

        while (!done && opModeIsActive()){
            currentHeading = -gyro.getIntegratedZValue();

            // calculate the difference between where we are
            // and where we want to be
            error = (targetHeading-currentHeading);
            power = error*kp;

            // Set minimum motor speeds
            if (power > 0 && power < .15)
                power = .15;
            else if (power < 0 && power > -.15)
                power = -.15;

            telemetry.addData("error", error);
            telemetry.addData("power", power);
            telemetry.addData("currentHeading", currentHeading);
            telemetry.addData("targetHeading", targetHeading);
            telemetry.update();

            if (currentHeading <= targetHeading+1 && currentHeading >= targetHeading-1){
                done = true;
                driveStop();
            }
            else{
                done = false;
                drive(0+power, 0-power);
            }

        }
        driveStop();
    }

    public void runOpMode() throws InterruptedException {
        rDrive1 = hardwareMap.dcMotor.get("rDrive1");
        rDrive2 = hardwareMap.dcMotor.get("rDrive2");
        lDrive1 = hardwareMap.dcMotor.get("lDrive1");
        lDrive2 = hardwareMap.dcMotor.get("lDrive2");
        belt = hardwareMap.servo.get("belt");
        lDrive1.setDirection(DcMotor.Direction.REVERSE);
        lDrive2.setDirection(DcMotor.Direction.REVERSE);
        sweeper = hardwareMap.dcMotor.get("sweeper");
        catapult = hardwareMap.dcMotor.get("catapult");
        hopper = hardwareMap.servo.get("hopper");
        touch = hardwareMap.touchSensor.get("t");
        button = hardwareMap.servo.get("button");
        wheels = hardwareMap.servo.get("wheels");
        hopper.setPosition(0.8);
        button.setPosition(0.5);
        belt.setPosition(.5);
        wheels.setPosition(.2);
        setUpGyro();
        boolean redSide = false;
        boolean blueSide = false;

        while (!redSide&&!blueSide){
            telemetry.addLine("Press dpad_left for RED side");
            telemetry.addLine("Press dpad_right for BLUE side");
            telemetry.update();
            if (gamepad1.dpad_left)
                redSide = true;
            else if (gamepad1.dpad_right)
                blueSide = true;
        }
        telemetry.update();
        waitForStart();

        if (redSide){
            encoderDrive(/*Distance*/125, /*leftSpeed*/.6, /*rightSpeed*/.6, /*direction*/-1);
            fire();
            encoderDrive(/*Distance*/100, /*leftSpeed*/.6, /*rightSpeed*/.6, /*direction*/1);
        }
        else if (blueSide){

        }


    }
}