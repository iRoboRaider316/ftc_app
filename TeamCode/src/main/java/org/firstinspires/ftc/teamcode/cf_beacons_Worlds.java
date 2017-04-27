package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

@Autonomous(name="cf_beacons_Worlds", group="LinearOpMode")
//@Disabled

public class cf_beacons_Worlds extends LinearOpMode {

    private DcMotor catapult;
    private DcMotor sweeper;
    private DcMotor lDrive1;
    private DcMotor lDrive2;
    private DcMotor rDrive1;
    private DcMotor rDrive2;
    private Servo button;
    private Servo hopper;
    Servo belt1;
    Servo belt2;
    private Servo wheels;
    private TouchSensor touch;
    private ColorSensor color;
    private ColorSensor collectionColor;
    private OpticalDistanceSensor fODSensor;
    private OpticalDistanceSensor bODSensor;

    GyroSensor gyroSensor;
    ModernRoboticsI2cGyro gyro;

    double[]pastError = new double[5];
    double sum = 0;
    double lastTime = 0;
    double lastError = 0;
    double speed = 0;
    double time = 0;
    boolean blue = true;
    boolean red = false;
    boolean redSide = false;
    boolean blueSide = false;
    boolean nothing = false;

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
        launchBall();
        launchPosition();
        loadBall();
        launchBall();
    }
    // Resets catapult to the launch position
    private void launchPosition() throws InterruptedException{
        // reset catapult to the firing position
        while (!touch.isPressed()){
            catapult.setPower(0.5);
        }
        catapult.setPower(0);
    }
    // Function to load the catapult
    private void loadBall() throws InterruptedException {
        // load a ball into the catapult
        hopper.setPosition(.5);
        sleep(800);
        hopper.setPosition(.8);
    }
    // Fires the ball
    private void launchBall() throws InterruptedException {
        // fire the catapult
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
    }
    private void useEncoders(){
        rDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    private void resetEncoders(){
        lDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
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

    private void PIDGyroTurn (int targetHeading, double time){
        double error;
        double currentHeading;
        double kp = .006;
        double ki = 0.0001;
        double kd = 0.002;
        double power;
        ElapsedTime runtime = new ElapsedTime();
        gyro.resetZAxisIntegrator();
        runtime.reset();
        sleep(250);
        lastError = targetHeading;
        lastTime = runtime.seconds();

        while (runtime.seconds() < time && opModeIsActive()){
            // positive angles are to the right, negative to the left.
            currentHeading = -gyro.getIntegratedZValue();
            // calculate error
            error = (targetHeading-currentHeading);

            // Set the power using PID control based off of the error.
            // Also uses a power offset of 0.2 to account for motor stall torque
            if (error > 0)
                power = .2+(error*kp)+(integral(error)*ki)+(derivative(error,runtime.seconds())*kd);
            else if (error < 0)
                power = -.2+(error*kp)+(integral(error)*ki)+(derivative(error,runtime.seconds())*kd);
            else
                power = 0;

            power = Range.clip(power, -1, 1);
            drive(0+power, 0-power);

            telemetry.addData("error", error);
            telemetry.addData("currentHeading", currentHeading);
            telemetry.addData("targetHeading", targetHeading);
            telemetry.addData("proportional",(error*kp));
            telemetry.addData("integral",(integral(error)*ki));
            telemetry.addData("derivative",(derivative(error,runtime.seconds())*kd));
            telemetry.addData("power", power);
            telemetry.update();
            // Wait to account for i2c bus lag for the gyro
            sleep(100);
        }
        driveStop();
    }

    private double integral(double error){
        sum = 0;
        pastError[4] = pastError[3];
        pastError[3] = pastError[2];
        pastError[2] = pastError[1];
        pastError[1] = pastError[0];
        pastError[0] = error;
        // Sum the past 5 error values
        // (Essentially take the integral of error vs time for the past 5 readings)
        for( double i : pastError) {
            sum += i;
        }
        return
                sum;
    }

    private double derivative(double error, double time){
        // Calculate the negative slope of the error vs time curve
        speed = (error-lastError)/(time-lastTime);
        lastError = error;
        lastTime = time;
        return
                speed;
    }

    public void collectRed() {
        // Create a new thread so the collection can be run parallel to everything else
        new Thread(new Runnable() {
            public void run() {
                // Reverse the ball collector to deploy it
                sweeper.setPower(1);
                try
                {
                    // wait for the ball collector to deploy
                    Thread.sleep(500);
                }catch(InterruptedException ie){
                }
                while (opModeIsActive()){
                    // if we collect a blue ball (on red alliance)
                    if (collectionColor.blue()>0&&collectionColor.red()<1){
                        // reverse sweeper
                        sweeper.setPower(1);
                        try
                        {
                            // wait for ball to be ejected
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

    public void collectBlue() {
        // Create a new thread so the collection can be run parallel to everything else
        new Thread(new Runnable() {
            public void run() {
                // Reverse the ball collector to deploy it
                sweeper.setPower(1);
                try
                {
                    // wait for the ball collector to deploy
                    Thread.sleep(500);
                }catch(InterruptedException ie){
                }
                while (opModeIsActive()){
                    // if we collect a red ball (on blue alliance)
                    if (collectionColor.red()>0&&collectionColor.blue()<1){
                        // reverse sweeper
                        sweeper.setPower(1);
                        try
                        {
                            // wait for ball to be ejected
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
        //##############Init##############
        rDrive1 = hardwareMap.dcMotor.get("rDrive1");
        rDrive2 = hardwareMap.dcMotor.get("rDrive2");
        lDrive1 = hardwareMap.dcMotor.get("lDrive1");
        lDrive2 = hardwareMap.dcMotor.get("lDrive2");
        lDrive1.setDirection(DcMotor.Direction.REVERSE);
        lDrive2.setDirection(DcMotor.Direction.REVERSE);
        sweeper = hardwareMap.dcMotor.get("sweeper");
        catapult = hardwareMap.dcMotor.get("catapult");
        button = hardwareMap.servo.get("button");
        hopper = hardwareMap.servo.get("hopper");
        belt1 = hardwareMap.servo.get("belt1");
        belt2 = hardwareMap.servo.get("belt2");
        touch = hardwareMap.touchSensor.get("t");
        color = hardwareMap.colorSensor.get("color");
        collectionColor = hardwareMap.colorSensor.get("collectionColor");
        color.setI2cAddress(I2cAddr.create7bit(0x1e));
        collectionColor.setI2cAddress(I2cAddr.create7bit(0x1d));
        wheels = hardwareMap.servo.get("wheels");
        fODSensor = hardwareMap.opticalDistanceSensor.get("fOD");
        bODSensor = hardwareMap.opticalDistanceSensor.get("bOD");
        belt1.setPosition(.5);
        belt2.setPosition(.5);
        hopper.setPosition(0.8);
        button.setPosition(0.5);
        wheels.setPosition(.62);

        boolean center = false;
        boolean ramp = false;
        boolean threeBall = false;
        boolean twoBall = false;

        resetEncoders();
        sleep(500);
        useEncoders();
        sleep(500);
        setUpGyro();

        // Menu to select alliance color
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

        // Select if we are collecting our alliance partner's ball at the start
        while (!threeBall&&!twoBall){
            telemetry.addLine("Press dpad_left for 3 ball");
            telemetry.addLine("Press dpad_right for 2 ball");
            telemetry.update();
            if (gamepad1.dpad_left)
                threeBall = true;
            else if (gamepad1.dpad_right)
                twoBall = true;
        }
        telemetry.update();
        sleep(1000);

        // Select where to park
        while (!center && !ramp && !nothing){
            telemetry.addLine("Press dpad_up to park center");
            telemetry.addLine("Press dpad_left to park ramp");
            telemetry.addLine("Press dpad_down to not park");
            telemetry.update();
            if (gamepad1.dpad_up)
                center = true;
            else if (gamepad1.dpad_left)
                ramp = true;
            else if (gamepad1.dpad_down)
                nothing = true;
        }

        // Read back telemetry of menu choices
        while (!isStarted()){
            if (blueSide)
                telemetry.addLine("BLUE");
            else if (redSide)
                telemetry.addLine("RED");
            if (center)
                telemetry.addLine("Parking center");
            else if (ramp)
                telemetry.addLine("Parking ramp");
            else if (nothing)
            telemetry.addLine("No Park");
            if (threeBall)
                telemetry.addLine("3 balls");
            else if (twoBall)
                telemetry.addLine("2 balls");
            telemetry.update();
        }

        collectionColor.enableLed(false);
        waitForStart();
        collectionColor.enableLed(true);

        if (redSide){
            if (twoBall){
                // drive forward for clearance
                encoderDrive(/*Distance*/10, /*leftSpeed*/.5, /*rightSpeed*/.5, /*direction*/-1);
            }
            else if (threeBall){
                // collect our alliance partner's ball
                collectRed();
                sleep(1500);
                // drive forward for clearance
                encoderDrive(/*Distance*/10, /*leftSpeed*/.5, /*rightSpeed*/.5, /*direction*/-1);
            }
            // lower side wheels
            wheels.setPosition(1);
            // curve until parallel with wall
            // left: 0.62, right: 0.75
            encoderDrive(/*Distance*/60, /*leftSpeed*/.60, /*rightSpeed*/.75, /*direction*/-1);
            // Drive backward
            encoderDrive(/*Distance*/25, /*leftSpeed*/.5, /*rightSpeed*/.45, /*direction*/-1);
            // Drive backward until we reach the far white line
            driveToLine(-1);
            // drive backward until we see red strong enough, then push the button
            huntBeacon(-1, red);
            // Drive forward past the line
            encoderDrive(/*Distance*/38, /*leftSpeed*/.6, /*rightSpeed*/.6, /*direction*/1);
            // Track forward along the wall until the next white line
            driveToLine(1);
            // Push the button for red
            huntBeacon(1, red);
            // drive back past line
            encoderDrive(/*Distance*/11, /*leftSpeed*/.5, /*rightSpeed*/.5, /*direction*/-1);
            // drive forward to the line
            driveToLine(1);
            // raise the side wheels
            wheels.setPosition(.62);
            noEncoders();
            sleep(500);
            // Turn left to face vortex
            PIDGyroTurn(-90,2.5);
            // Drive forward into range
            encoderDrive(/*Distance*/14, /*leftSpeed*/.5, /*rightSpeed*/.5, /*direction*/1);
            // Shoot both balls
            fire();
            // Shoot our alliance partner's 3rd ball
            if (threeBall){
                launchPosition();
                loadBall();
                launchBall();
            }
            if(center) {
                // drive forward into cap ball
                encoderDrive(/*Distance*/10, /*leftSpeed*/.5, /*rightSpeed*/.5, /*direction*/1);
                // turn to move cap ball
                PIDGyroTurn(160,3);
                // drive onto center
                encoderDrive(/*Distance*/22, /*leftSpeed*/.5, /*rightSpeed*/.6, /*direction*/-1);
                // Turn to fully park
                drive(.5,-.5);
                sleep(300);
                driveStop();
            }
            else if (nothing)
                sleep(100000);
            else {
                // turn toward ramp
                PIDGyroTurn(100,2.5);
                // drive into ramp
                encoderDrive(/*Distance*/35, /*leftSpeed*/.5, /*rightSpeed*/.5, /*direction*/1);
            }
        }
        else if (blueSide){
            if (twoBall){
                // drive forward for clearance
                encoderDrive(/*Distance*/10, /*leftSpeed*/.5, /*rightSpeed*/.5, /*direction*/1);
            }
            else if (threeBall){
                // collect our alliance partner's ball
                collectBlue();
                sleep(1500);
                // drive forward for clearance
                encoderDrive(/*Distance*/10, /*leftSpeed*/.5, /*rightSpeed*/.5, /*direction*/-1);
                // Turn around
                PIDGyroTurn(175,4);
            }
            // lower side wheels
            wheels.setPosition(1);
            // curve until parallel with wall
            encoderDrive(/*Distance*/60, /*leftSpeed*/.60, /*rightSpeed*/.75, /*direction*/1);
            // Drive backward
            encoderDrive(/*Distance*/25, /*leftSpeed*/.5, /*rightSpeed*/.45, /*direction*/1);
            // Drive backward until we reach the far white line
            driveToLine(1);
            // drive backward until we see red strong enough, then push the button
            huntBeacon(1, blue);
            // Drive forward past the line
            encoderDrive(/*Distance*/38, /*leftSpeed*/.6, /*rightSpeed*/.6, /*direction*/-1);
            // Track forward along the wall until the next white line
            driveToLine(-1);
            // Push the button for red
            huntBeacon(-1, blue);
            // drive back past line
            encoderDrive(/*Distance*/11, /*leftSpeed*/.5, /*rightSpeed*/.5, /*direction*/1);
            // drive forward to the line
            driveToLine(-1);
            // raise the side wheels
            wheels.setPosition(.62);
            noEncoders();
            sleep(500);
            // Turn 80 degrees left to face vortex
            PIDGyroTurn(-88,2.5);
            // Drive forward into range
            encoderDrive(/*Distance*/14, /*leftSpeed*/.5, /*rightSpeed*/.5, /*direction*/1);
            // Shoot both balls
            fire();
            if (threeBall){
                // shoot our alliance partner's ball
                launchPosition();
                loadBall();
                launchBall();
            }
            if(center) {
                // drive forward into cap ball
                encoderDrive(/*Distance*/10, /*leftSpeed*/.5, /*rightSpeed*/.5, /*direction*/1);
                // turn to move cap ball
                PIDGyroTurn(-160,3);
                // drive onto center
                encoderDrive(/*Distance*/22, /*leftSpeed*/.5, /*rightSpeed*/.6, /*direction*/-1);
                // turn to fully park
                drive(-.5,.5);
                sleep(300);
                driveStop();
            }
            else if (nothing)
            sleep(100000);
            else    {
                // turn toward ramp
                PIDGyroTurn(-100,2.5);
                // drive into ramp
                encoderDrive(/*Distance*/35, /*leftSpeed*/.5, /*rightSpeed*/.5, /*direction*/1);
            }
        }
    }
}