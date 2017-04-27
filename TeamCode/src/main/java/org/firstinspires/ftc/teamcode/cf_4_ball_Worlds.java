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

@Autonomous(name="cf_4_ball_Worlds", group="LinearOPMode")
//@Disabled

public class cf_4_ball_Worlds extends LinearOpMode {
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
    double[]pastError = new double[5];
    double sum = 0;
    double lastTime = 0;
    double lastError = 0;
    double speed = 0;
    double time = 0;

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

        noEncoders();
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
        // Read back selections
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
            // Drive between the corner and center vortexes
            encoderDrive(/*Distance*/59, /*leftSpeed*/.7, /*rightSpeed*/.7, /*direction*/1);
            // Turn to face vortex
            PIDGyroTurn(119,2.5);
            // Drive forward into firing range
            encoderDrive(/*Distance*/2, /*leftSpeed*/.7, /*rightSpeed*/.7, /*direction*/1);
            // Start thread to control ball collector and check ball color
            collectRed();
            // Shoot both balls
            fire();
            // Turn toward close wall
            PIDGyroTurn(65,2.5);
            // Lower side wheels
            wheels.setPosition(1);
            // Drive forward
            //encoderDrive(/*Distance*/10, /*leftSpeed*/.7, /*rightSpeed*/.7, /*direction*/1);
            // Curve into wall
            encoderDrive(/*Distance*/70, /*leftSpeed*/.5, /*rightSpeed*/.75, /*direction*/1);
            // Drive forward
            encoderDrive(/*Distance*/15, /*leftSpeed*/.8, /*rightSpeed*/.75, /*direction*/1);
            // Drive until we see the red/blue tape
            driveToLine(1);
            // Drive forward
            encoderDrive(/*Distance*/1, /*leftSpeed*/.8, /*rightSpeed*/.85, /*direction*/1);
            // Wait for balls to be collected from corner
            sleep(2000);
            // Drive back out of corner
            encoderDrive(/*Distance*/40, /*leftSpeed*/.8, /*rightSpeed*/.85, /*direction*/-1);
            // Raise side wheels
            wheels.setPosition(.62);
            sleep(250);
            // Turn left to face vortex
            PIDGyroTurn(-115,2);
            // Drive forward into range
            encoderDrive(/*Distance*/20, /*leftSpeed*/.7, /*rightSpeed*/.7, /*direction*/1);
            // Shoot balls
            loadBall();
            fire();
            // Drive forward onto center base
            encoderDrive(/*Distance*/20, /*leftSpeed*/.7, /*rightSpeed*/.7, /*direction*/1);
        }
        else if (blueSide){
            // Drive between the corner and center vortexes
            encoderDrive(/*Distance*/62, /*leftSpeed*/.7, /*rightSpeed*/.7, /*direction*/1);
            // Turn to face vortex
            PIDGyroTurn(-119,2.5);
            // Drive forward into firing range
            encoderDrive(/*Distance*/2, /*leftSpeed*/.7, /*rightSpeed*/.7, /*direction*/1);
            // Start thread to control ball collector and check ball color
            collectBlue();
            // Shoot both balls
            fire();
            // Turn toward close wall
            PIDGyroTurn(65,2.5);
            // Lower side wheels
            wheels.setPosition(1);
            // Drive forward
            //encoderDrive(/*Distance*/10, /*leftSpeed*/.7, /*rightSpeed*/.7, /*direction*/1);
            // Curve into wall
            encoderDrive(/*Distance*/74, /*leftSpeed*/.5, /*rightSpeed*/.75, /*direction*/-1);
            // Drive forward
            encoderDrive(/*Distance*/15, /*leftSpeed*/.8, /*rightSpeed*/.75, /*direction*/1);
            // Drive until we see the red/blue tape
            driveToLine(-1);
            // Drive forward
            encoderDrive(/*Distance*/2, /*leftSpeed*/.8, /*rightSpeed*/.85, /*direction*/1);
            // Raise side wheels
            wheels.setPosition(.62);
            sleep(250);
            // Turn around to face corner
            PIDGyroTurn(-175,3);
            // Wait for balls to be collected from corner
            sleep(2000);
            // Drive back out of corner
            encoderDrive(/*Distance*/20, /*leftSpeed*/.8, /*rightSpeed*/.85, /*direction*/-1);
            sleep(250);
            // Turn left to face vortex
            PIDGyroTurn(-115,2);
            // Drive forward into range
            encoderDrive(/*Distance*/20, /*leftSpeed*/.7, /*rightSpeed*/.7, /*direction*/1);
            // Shoot balls
            loadBall();
            fire();
            // Drive forward onto center base
            encoderDrive(/*Distance*/20, /*leftSpeed*/.7, /*rightSpeed*/.7, /*direction*/1);
        }

    }
}