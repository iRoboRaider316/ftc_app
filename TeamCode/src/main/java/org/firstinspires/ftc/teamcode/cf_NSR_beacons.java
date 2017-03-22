package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Autonomous(name="cf_NSR_beacons", group="LinearOpMode")
//@Disabled

public class cf_NSR_beacons extends LinearOpMode {

    private DcMotor catapult;
    //private DcMotor sweeper;
    private DcMotor lDrive1;
    private DcMotor lDrive2;
    private DcMotor rDrive1;
    private DcMotor rDrive2;
    private Servo button;
    private Servo hopper;
    private Servo belt;
    private Servo wheels;
    private TouchSensor touch;
    private ColorSensor color;
    private OpticalDistanceSensor fODSensor;
    private OpticalDistanceSensor bODSensor;

    GyroSensor gyroSensor;
    ModernRoboticsI2cGyro gyro;

    boolean blue = true;
    boolean red = false;
    boolean redSide = false;
    boolean blueSide = false;

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

    private void driveToLine(int direction) throws InterruptedException {
        // drive forward until the front OD sensor detects the white line
        if (direction == 1){
            while (fODSensor.getRawLightDetected() < .06 && opModeIsActive()) {
                drive(.2,.17);
            }
        }
        // drive backward until the back OD sensor detects the white line
        else if (direction == -1){
            while (bODSensor.getRawLightDetected() < .06 && opModeIsActive()) {
                drive(-.2,-.17);
            }
        }
        else {
            telemetry.addLine("Invalid Direction");
            telemetry.update();
            sleep(10000);
        }

        driveStop();
    }

    private void huntBeacon(int direction, boolean checkBlue){
        color.enableLed(false);
        // while we don't see the color we're looking for,
        // drive in the direction passed the function
        while(!recognizeColor(checkBlue)){
            drive(.2*direction, .18*direction);
            telemetry.addData("Blue", color.blue());
            telemetry.addData("Red", color.red());
            telemetry.update();
        }
        driveStop();
        pushButton();
        telemetry.addData("Blue", color.blue());
        telemetry.addData("Red", color.red());
        telemetry.update();
    }

    private void wrongColor(boolean checkBlue) {
        // if not the correct color, sleep 5 seconds and re-push the button
        if (!recognizeColor(checkBlue)) {
            sleep(5000);
            pushButton();
        }
    }

    private void pushButton(){
        // Extend button pusher
        button.setPosition(0);
        sleep(1200);
        // Retract button pusher
        button.setPosition(1);
        sleep(1200);
        // Stop movement
        button.setPosition(0.5);
    }

    private boolean recognizeColor(boolean checkBlue){
        boolean blue;
        boolean red;
        if(color.blue() > color.red()+1){
            // color is sufficiently blue
            blue = true;
            red = false;
        }
        else if (color.red() > color.blue()+1){
            // color is sufficiently red
            red = true;
            blue = false;
        }
        else{
            // Neither color is strong enough
            red = false;
            blue = false;
        }
        if (checkBlue){
            // we are looking for blue
            return(blue);
        }
        else{
            // we are looking for red
            return(red);
        }
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
        //##############Init##############
        rDrive1 = hardwareMap.dcMotor.get("rDrive1");
        rDrive2 = hardwareMap.dcMotor.get("rDrive2");
        lDrive1 = hardwareMap.dcMotor.get("lDrive1");
        lDrive2 = hardwareMap.dcMotor.get("lDrive2");
        lDrive1.setDirection(DcMotor.Direction.REVERSE);
        lDrive2.setDirection(DcMotor.Direction.REVERSE);

        //sweeper = hardwareMap.dcMotor.get("sweeper");
        catapult = hardwareMap.dcMotor.get("catapult");
        button = hardwareMap.servo.get("button");
        hopper = hardwareMap.servo.get("hopper");
        belt = hardwareMap.servo.get("belt");
        touch = hardwareMap.touchSensor.get("t");
        color = hardwareMap.colorSensor.get("color");
        wheels = hardwareMap.servo.get("wheels");

        fODSensor = hardwareMap.opticalDistanceSensor.get("fOD");
        bODSensor = hardwareMap.opticalDistanceSensor.get("bOD");

        hopper.setPosition(0.8);
        button.setPosition(0.5);
        belt.setPosition(.5);
        wheels.setPosition(.62);

        boolean center = false;
        boolean ramp = false;
        boolean vortex = false;

        resetEncoders();
        idle();
        useEncoders();
        idle();
        setUpGyro();

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
        sleep(1000);

        while (!center && !ramp && !vortex){
            telemetry.addLine("Press dpad_up to park center");
            telemetry.addLine("Press dpad_left to park ramp");
            telemetry.addLine("Press dpad_right to turn vortex");
            telemetry.update();
            if (gamepad1.dpad_up)
                center = true;
            else if (gamepad1.dpad_left)
                ramp = true;
            else if (gamepad1.dpad_right)
                vortex = true;
        }

        while (!isStarted()){
            if (blueSide)
                telemetry.addLine("BLUE");
            else if (redSide)
                telemetry.addLine("RED");
            if (center)
                telemetry.addLine("Parking center");
            else if (ramp)
                telemetry.addLine("Parking ramp");
            else
                telemetry.addLine("Turning vortex");
            telemetry.update();
        }

        waitForStart();

        if (redSide){
            // drive forward for clearance
            encoderDrive(/*Distance*/10, /*leftSpeed*/.5, /*rightSpeed*/.5, /*direction*/-1);
            // lower side wheels
            wheels.setPosition(1);
            // curve until parallel with wall
            encoderDrive(/*Distance*/60, /*leftSpeed*/.62, /*rightSpeed*/.75, /*direction*/-1);
            // Drive backward
            encoderDrive(/*Distance*/25, /*leftSpeed*/.5, /*rightSpeed*/.45, /*direction*/-1);
            // Drive backward until we reach the far white line
            driveToLine(-1);
            // drive backward until we see red strong enough, then push the button
            huntBeacon(-1, red);
            // double check to make sure we hit the right color
            //wrongColor(red);
            // Drive forward past the line
            encoderDrive(/*Distance*/38, /*leftSpeed*/.6, /*rightSpeed*/.6, /*direction*/1);
            // Track forward along the wall until the next white line
            driveToLine(1);
            // Push the button for red
            huntBeacon(1, red);
            // double check to make sure we hit the right color
            //wrongColor(red);
            // drive back past line
            encoderDrive(/*Distance*/11, /*leftSpeed*/.5, /*rightSpeed*/.5, /*direction*/-1);
            // drive forward to the line
            driveToLine(1);
            // drive forward correct distance for shooting
            encoderDrive(/*Distance*/7, /*leftSpeed*/.5, /*rightSpeed*/.5, /*direction*/1);
            // raise the side wheels
            wheels.setPosition(.62);
            sleep(250);
            // Turn 90 degrees left to face vortex
            gyroTurn(-85);
            // Drive forward into range
            encoderDrive(/*Distance*/5, /*leftSpeed*/.5, /*rightSpeed*/.5, /*direction*/1);
            // Shoot both balls
            fire();

            if(vortex) {
                // drive forward into cap ball
                drive(0.5, 0.5);
                // wait until vortex rotates
                sleep(2500);
                driveStop();
                // turn to move cap ball
                gyroTurn(40);
                gyroTurn(-40);
                // drive onto center
                drive(0.5, 0.5);
                sleep(1000);
                driveStop();
            }
            else if(center) {
                // drive forward into cap ball
                drive(0.5, 0.5);
                sleep(1000);
                driveStop();
                // turn to move cap ball
                gyroTurn(40);
                gyroTurn(-40);
                // drive onto center
                encoderDrive(/*Distance*/10, /*leftSpeed*/.5, /*rightSpeed*/.5, /*direction*/1);
            }
            else {
                // turn toward ramp
                gyroTurn(100);
                // drive into ramp
                encoderDrive(/*Distance*/24, /*leftSpeed*/.5, /*rightSpeed*/.5, /*direction*/1);
            }
        }
        else if (blueSide){
            // drive forward for clearance
            encoderDrive(/*Distance*/10, /*leftSpeed*/.5, /*rightSpeed*/.5, /*direction*/1);
            // lower side wheels
            wheels.setPosition(1);
            // curve until parallel with wall
            encoderDrive(/*Distance*/60, /*leftSpeed*/.62, /*rightSpeed*/.75, /*direction*/1);
            // Drive backward
            encoderDrive(/*Distance*/25, /*leftSpeed*/.5, /*rightSpeed*/.45, /*direction*/1);
            // Drive backward until we reach the far white line
            driveToLine(1);
            // drive backward until we see red strong enough, then push the button
            huntBeacon(1, blue);
            // double check to make sure we hit the right color
            //wrongColor(red);
            // Drive forward past the line
            encoderDrive(/*Distance*/38, /*leftSpeed*/.6, /*rightSpeed*/.6, /*direction*/-1);
            // Track forward along the wall until the next white line
            driveToLine(-1);
            // Push the button for red
            huntBeacon(-1, blue);
            // double check to make sure we hit the right color
            //wrongColor(red);
            // drive back past line
            encoderDrive(/*Distance*/11, /*leftSpeed*/.5, /*rightSpeed*/.5, /*direction*/1);
            // drive forward to the line
            driveToLine(-1);
            // drive forward correct distance for shooting
            encoderDrive(/*Distance*/7, /*leftSpeed*/.5, /*rightSpeed*/.5, /*direction*/-1);
            // raise the side wheels
            wheels.setPosition(.62);
            sleep(250);
            // Turn 80 degrees left to face vortex
            gyroTurn(-75);
            // Drive forward into range
            encoderDrive(/*Distance*/5, /*leftSpeed*/.5, /*rightSpeed*/.5, /*direction*/1);
            // Shoot both balls
            fire();

            if(vortex) {
                // drive forward into cap ball
                drive(0.5, 0.5);
                // wait until vortex rotates
                sleep(2500);
                driveStop();
                // turn to move cap ball
                gyroTurn(-40);
                gyroTurn(40);
                // drive onto center
                drive(0.5, 0.5);
                sleep(1000);
                driveStop();
            }
            else if(center) {
                // drive forward into cap ball
                drive(0.5, 0.5);
                sleep(1000);
                driveStop();
                // turn to move cap ball
                gyroTurn(40);
                gyroTurn(-40);
                // drive onto center
                encoderDrive(/*Distance*/10, /*leftSpeed*/.5, /*rightSpeed*/.5, /*direction*/1);
            }
            else {
                // turn toward ramp
                gyroTurn(-100);
                // drive into ramp
                encoderDrive(/*Distance*/24, /*leftSpeed*/.5, /*rightSpeed*/.5, /*direction*/1);
            }
        }
    }
}