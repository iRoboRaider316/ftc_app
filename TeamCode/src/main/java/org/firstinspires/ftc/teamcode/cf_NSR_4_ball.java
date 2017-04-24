package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="cf_4_ball_NSR", group="LinearOPMode")
//@Disabled

public class cf_NSR_4_ball extends LinearOpMode {
    private DcMotor catapult;
    private DcMotor lDrive1;
    private DcMotor lDrive2;
    private DcMotor rDrive1;
    private DcMotor rDrive2;
    private DcMotor sweeper;
    Servo belt1;
    Servo belt2;
    private Servo button;
    private Servo hopper;
    private Servo wheels;
    private TouchSensor touch;
    private OpticalDistanceSensor fODSensor;
    private OpticalDistanceSensor bODSensor;
    GyroSensor gyroSensor;
    ModernRoboticsI2cGyro gyro;
    private ColorSensor color;
    private ColorSensor collectionColor;

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
        sleep(800);
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
            if (power > 0 && power < .2)
                power = .2;
            else if (power < 0 && power > -.2)
                power = -.2;

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
    private void timedGyroTurn (int targetHeading, double time){
        //boolean done = false;
        double error;
        double currentHeading;
        double kp = .003;
        double power;
        ElapsedTime runtime = new ElapsedTime();
        gyro.resetZAxisIntegrator();
        runtime.reset();
        sleep(250);

        while (runtime.seconds() < time && opModeIsActive()){
            currentHeading = -gyro.getIntegratedZValue();
            error = (targetHeading-currentHeading);

            if (error > 0)
                power = .15+(error*kp);
            else if (error < 0)
                power = -.15+(error*kp);
            else
                power = 0;

            drive(0+power, 0-power);

            telemetry.addData("error", error);
            telemetry.addData("power", power);
            telemetry.addData("currentHeading", currentHeading);
            telemetry.addData("targetHeading", targetHeading);
            telemetry.update();
        }
        driveStop();
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

    public void collectRed() {
        new Thread(new Runnable() {
            public void run() {
                sweeper.setPower(1);
                try
                {
                    Thread.sleep(500);
                }catch(InterruptedException ie){
                }
                while (opModeIsActive()){
                    if (collectionColor.blue()>0&&collectionColor.red()<1){
                        // reverse sweeper
                        sweeper.setPower(1);
                        try
                        {
                            Thread.sleep(1000);
                        }catch(InterruptedException ie){
                        }
                    }
                    else{
                        // collect
                        sweeper.setPower(-1);
                    }
                    telemetry.addData("Blue", collectionColor.blue());
                    telemetry.addData("Red", collectionColor.red());
                    telemetry.update();
                }
            }
        }).start();
    }

    public void runOpMode() throws InterruptedException {
        rDrive1 = hardwareMap.dcMotor.get("rDrive1");
        rDrive2 = hardwareMap.dcMotor.get("rDrive2");
        lDrive1 = hardwareMap.dcMotor.get("lDrive1");
        lDrive2 = hardwareMap.dcMotor.get("lDrive2");
        belt1 = hardwareMap.servo.get("belt1");
        belt2 = hardwareMap.servo.get("belt2");
        lDrive1.setDirection(DcMotor.Direction.REVERSE);
        lDrive2.setDirection(DcMotor.Direction.REVERSE);
        sweeper = hardwareMap.dcMotor.get("sweeper");
        catapult = hardwareMap.dcMotor.get("catapult");
        hopper = hardwareMap.servo.get("hopper");
        touch = hardwareMap.touchSensor.get("t");
        button = hardwareMap.servo.get("button");
        wheels = hardwareMap.servo.get("wheels");
        color = hardwareMap.colorSensor.get("color");
        collectionColor = hardwareMap.colorSensor.get("collectionColor");
        fODSensor = hardwareMap.opticalDistanceSensor.get("fOD");
        bODSensor = hardwareMap.opticalDistanceSensor.get("bOD");
        hopper.setPosition(0.8);
        button.setPosition(0.5);
        belt1.setPosition(.5);
        belt2.setPosition(.5);
        wheels.setPosition(.2);
        setUpGyro();
        boolean redSide = false;
        boolean blueSide = false;
        color.setI2cAddress(I2cAddr.create7bit(0x1e));
        collectionColor.setI2cAddress(I2cAddr.create7bit(0x1d));

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
        while (!isStarted()){
            if (blueSide)
                telemetry.addLine("BLUE");
            else if (redSide)
                telemetry.addLine("RED");
            telemetry.update();
        }
        collectionColor.enableLed(false);
        waitForStart();
        collectionColor.enableLed(true);

        if (redSide){
            encoderDrive(/*Distance*/59, /*leftSpeed*/.7, /*rightSpeed*/.7, /*direction*/1);
//            encoderDrive(/*Distance*/5, /*leftSpeed*/.7, /*rightSpeed*/.7, /*direction*/1);
//            timedGyroTurn(-58,2);
//            encoderDrive(/*Distance*/43, /*leftSpeed*/.7, /*rightSpeed*/.7, /*direction*/1);
            timedGyroTurn(119,2.5);
            encoderDrive(/*Distance*/2, /*leftSpeed*/.7, /*rightSpeed*/.7, /*direction*/1);
            collectRed();
            fire();
            timedGyroTurn(60,2);
            wheels.setPosition(1);
            encoderDrive(/*Distance*/10, /*leftSpeed*/.7, /*rightSpeed*/.7, /*direction*/1);
            encoderDrive(/*Distance*/60, /*leftSpeed*/.52, /*rightSpeed*/.75, /*direction*/1);
            encoderDrive(/*Distance*/15, /*leftSpeed*/.8, /*rightSpeed*/.75, /*direction*/1);
            driveToLine(1);
            encoderDrive(/*Distance*/1, /*leftSpeed*/.8, /*rightSpeed*/.85, /*direction*/1);
            sleep(2000);
            encoderDrive(/*Distance*/40, /*leftSpeed*/.8, /*rightSpeed*/.85, /*direction*/-1);
            wheels.setPosition(.62);
            sleep(250);
            // Turn 90 degrees left to face vortex
            timedGyroTurn(-115,2);
            // Drive forward into range
            encoderDrive(/*Distance*/20, /*leftSpeed*/.7, /*rightSpeed*/.7, /*direction*/1);
            // Shoot both balls
            loadBall();
            fire();
            encoderDrive(/*Distance*/10, /*leftSpeed*/.7, /*rightSpeed*/.7, /*direction*/1);
            timedGyroTurn(-160,3);
            encoderDrive(/*Distance*/10, /*leftSpeed*/.7, /*rightSpeed*/.7, /*direction*/-1);
        }
        else if (blueSide){
            collectRed();
            sleep(30000);
        }


    }
}