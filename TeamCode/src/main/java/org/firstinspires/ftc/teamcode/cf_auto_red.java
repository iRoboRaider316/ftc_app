package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="cf_auto_red", group="LinearOpMode")

public class cf_auto_red extends LinearOpMode {

    private DcMotor catapult;
    //private DcMotor sweeper;
    private DcMotor lDrive1;
    private DcMotor lDrive2;
    private DcMotor rDrive1;
    private DcMotor rDrive2;
    private Servo lButton;
    //private Servo rButton;
    private Servo hopper;

    private TouchSensor touch;
    private GyroSensor gyroSensor;
    private ModernRoboticsI2cGyro gyro;
    private ColorSensor color;
    private OpticalDistanceSensor rODSensor;
    private OpticalDistanceSensor lODSensor;
    private I2cAddr RANGE1ADDRESS = new I2cAddr(0x14); //Default I2C address for MR Range (7-bit)
    private static final int RANGE1_REG_START = 0x04; //Register to start reading
    private static final int RANGE1_READ_LENGTH = 2; //Number of byte to read

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
            telemetry.addLine("Calibrating Gyro");
            updateTelemetry(telemetry);
        }
        // End of setting up Gyro
    }

    // Function to reset the catapult to the launch position
    private void launchPosition() throws InterruptedException{

       while (!touch.isPressed()){
         catapult.setPower(0.5);
        }
        catapult.setPower(0);

        }
    // Function that utlizes the launchPosition, handleBall, and launch functions to fire and reload the catapult
    private void fire() throws InterruptedException {
        launchPosition();
        launchBall();
        launchPosition();
        loadBall();
        launchBall();
        launchPosition();
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

    // Function to use the gyro to do a spinning turn in place.
    // It points the robot at an absolute heading, not a relative turn.  0 will point robot to same
    // direction we were at the start of program.
    // Pass:
    // targetHeading = the new heading we want to point robot at,
    // maxSpeed = the max speed the motor can run in the range of 0 to 1
    // direction = the direction we will turn, 1 is clockwise, -1 is counter-clockwise
    // Returns:
    // heading = the new heading the gyro reports
    public void gyroTurn(int targetHeading, double maxSpeed, int direction) throws InterruptedException {
        //int startHeading = gyroSensor.getHeading();
        int deceleration;
        int currentHeading;
        double oldSpeed;
        double speed = 0;
        double minSpeed = 0.3;
        double acceleration = 0.01;
        double ka = 0.01;             // Proportional acceleration constant

        lDrive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rDrive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        idle();

        // Calls turnCompeted function to determine if we need to keep turning
        while ((!turnCompleted(gyroSensor.getHeading(), targetHeading, 5, direction) && opModeIsActive())) {

            // Move speed management to separate function

            // Calculate the speed we should be moving at
            oldSpeed = speed;           // save our old speed for use later.

            currentHeading = gyroSensor.getHeading();
            // Reuses the degreesToTurn function by passing different values to obtain our error
            deceleration = degreesToTurn(targetHeading, currentHeading, direction);

            speed = deceleration * ka * direction;

            // Limit the acceleration of the motors speed at beginning of turns.
            if( Math.abs(speed) > Math.abs(oldSpeed))
                speed = oldSpeed + (direction * acceleration);

            // Set a minimum power for the motors to make sure they move
            if (Math.abs(speed) < minSpeed)
                speed = minSpeed * direction;


            // Don't exceed the maximium speed requested
            if (Math.abs(speed) > maxSpeed)
                speed = maxSpeed * direction;

            // Set the motor speeds
            lDrive1.setPower(speed);
            lDrive2.setPower(speed);
            rDrive1.setPower(-speed);
            rDrive2.setPower(-speed);
            telemetry.addData("Current Heading", gyroSensor.getHeading());
            telemetry.addData("Current Speed", speed);
            updateTelemetry(telemetry);
        }
        // Done with the turn so shut off motors
        lDrive1.setPower(0);
        lDrive2.setPower(0);
        rDrive1.setPower(0);
        rDrive2.setPower(0);

    }

    // Function used by the turnCompleted function (which is used by the gyroTurn function) to determine the degrees to turn.
    // Also used to determine error for proportional speed control in the gyroTurn function
    // by passing targetHeading and currentHeading instead of startHeading and targetHeading, respectively
    // Pass:
    // startHeading = heading that we are at before we turn
    // targetHeading = heading we want to turn to
    // direction = the direction we want to turn (1 for right, -1 for left)
    public static int degreesToTurn(int startHeading, int targetHeading, int direction) {

        int degreesToTurn;

        // Turning right
        if (direction == 1)
            degreesToTurn = targetHeading - startHeading;
            // degreesToTurn = targetHeading - currentHeading
            // Turning left
        else
            degreesToTurn = startHeading - targetHeading;

        if (degreesToTurn < 0) // Changed from "while"
            degreesToTurn = degreesToTurn + 360;
        else if (degreesToTurn > 360) // Changed from "while"
            degreesToTurn = degreesToTurn + 360;

        return (degreesToTurn);
    }

    // Function used by the GyroTurn function to determine if we have turned far enough.
    // Pulls from degreesToTurn function to determine degrees to turn.
    // Pass:
    // currentHeading = the heading the robot is currently at (live value, changes during the turn)
    // targetHeading = the heading we want the robot to be at once the turn is completed
    // range = the degrees of error we are allowing so that the robot doesn't spin in circles
    // if it overshoots by a small amount
    // direction = direction the robot is turning (1 for right, -1 for left)
    // Returns:
    // result = true if we have reached target heading, false if we haven't
    public static boolean turnCompleted(int currentHeading, int targetHeading, int range, int direction)  {

        // Value we want to stop at
        int stop;
        // Will be sent to the GyroTurn function; robot stops turning when true
        boolean result;

        // Turn right
        if (direction >= 1) {
            stop = targetHeading - range;
            if (stop >= 0){
                result = (currentHeading >= stop && currentHeading <= targetHeading);
            }
            else{
                result = ((currentHeading >= (stop+360) && currentHeading <=359)|| (currentHeading >= 0 && currentHeading <= targetHeading));
            }
        }
        // Turn left
        else {
            stop = targetHeading + range;
            if (stop <=359){
                result = (currentHeading <= stop && currentHeading >= targetHeading);
            }
            else{
                result = (currentHeading >= targetHeading && currentHeading <= 359) || (currentHeading >=0 && currentHeading <= (stop-360));
            }
        }
        return (result);
    }

    // This is the Drive Method
    // It will take in two static values: distance and maxSpeed.
    // It will then calculate the encoder counts to drive and drive the distance at the specified power,
    // accelerating to max speed for the first third of the distance, maintaining that speed for the second third,
    // and decelerating to a minimum speed for the last third.
    // If the robot deviates from the initial gyro heading, it will correct itself proportionally to the error.
    private void drive(double distance, double maxSpeed, int direction) {
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

        while (Math.abs(rDrive1.getCurrentPosition())<Math.abs(rDrive1.getTargetPosition()-5)){

            // Calculate distance from original heading and divide by 10
            error = (gyroSensor.getHeading()-heading);
            // Deal with wraparound from 359 to 0
            if (error >180)
                error = ((gyroSensor.getHeading()-heading-360)/20);
            else if(error <-180)
                error = ((gyroSensor.getHeading()-heading+360)/20);
            else
                error = ((gyroSensor.getHeading()-heading)/20);

            leftSpeed = (maxSpeed-error)*direction;
            rightSpeed = (maxSpeed+error)*direction;

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

    private void driveBackward(long time, double maxSpeed) throws InterruptedException {
        double leftSpeed;
        double rightSpeed;
        double error;
        double heading = gyroSensor.getHeading();
        /*// Calculate distance from original heading and divide by 40
        error = ((gyroSensor.getHeading()-heading)/60);
        // Deal with wraparound from 359 to 0
        if (error >180)
            error = ((gyroSensor.getHeading()-heading-360)/60);
        else if(error <-180)
            error = ((gyroSensor.getHeading()-heading+360)/60);

        leftSpeed = -maxSpeed+error;
        rightSpeed = -maxSpeed-error;

        leftSpeed = Range.clip(leftSpeed, -1, 1);
        rightSpeed = Range.clip(rightSpeed, -1, 1);

        lDrive1.setPower(leftSpeed);
        lDrive2.setPower(leftSpeed);
        rDrive1.setPower(rightSpeed);
        rDrive2.setPower(rightSpeed);

        rDrive1.setPower(-maxSpeed);
        rDrive2.setPower(-maxSpeed);
        lDrive1.setPower(-maxSpeed-.1);
        lDrive2.setPower(-maxSpeed-.1);
        sleep(time);
        rDrive1.setPower(0);
        rDrive2.setPower(0);
        lDrive1.setPower(0);
        lDrive2.setPower(0);
        sleep(700);
    }

    // This is the driveToLine method.
    // When called, the robot drives forward until the optical distance sensor detects a white line on the mat.
    private void driveToLine() {
        lDrive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rDrive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double leftSpeed;
        double rightSpeed;
        double error;

        double heading = gyroSensor.getHeading();

        rODSensor.enableLed(true);
        lODSensor.enableLed(true);
        while(rODSensor.getRawLightDetected()<0.21&&lODSensor.getRawLightDetected()<0.21&&opModeIsActive()){
            telemetry.addData("rLight", rODSensor.getRawLightDetected());
            telemetry.addData("lLight", lODSensor.getRawLightDetected());
            telemetry.update();

            // Calculate distance from original heading
            error = (gyroSensor.getHeading()-heading);
            // Deal with wraparound from 359 to 0
            if (error >180)
                error = ((gyroSensor.getHeading()-heading-360)/60);
            else if(error <-180)
                error = ((gyroSensor.getHeading()-heading+360)/60);
            else
                error = ((gyroSensor.getHeading()-heading)/60);

            leftSpeed = 0.3-error;
            rightSpeed = 0.3+error;

            leftSpeed = Range.clip(leftSpeed, -1, 1);
            rightSpeed = Range.clip(rightSpeed, -1, 1);

            lDrive1.setPower(leftSpeed);
            lDrive2.setPower(leftSpeed);
            rDrive1.setPower(rightSpeed);
            rDrive2.setPower(rightSpeed);
        }
        lDrive1.setPower(0);
        lDrive2.setPower(0);
        rDrive1.setPower(0);
        rDrive2.setPower(0);
    }

    // This is the driveToLine method.
    // When called, the robot drives forward until the optical distance sensor detects a white line on the mat.
    private void driveBackwardToLine() {
        lDrive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rDrive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double leftSpeed;
        double rightSpeed;
        double error;

        double heading = gyroSensor.getHeading();

        rODSensor.enableLed(true);
        lODSensor.enableLed(true);
        while(rODSensor.getRawLightDetected()<0.21&&lODSensor.getRawLightDetected()<0.21&&opModeIsActive()){
            telemetry.addData("rLight", rODSensor.getRawLightDetected());
            telemetry.addData("lLight", lODSensor.getRawLightDetected());
            telemetry.update();

            // Calculate distance from original heading and divide by 40
            error = ((gyroSensor.getHeading()-heading)/60);
            // Deal with wraparound from 359 to 0
            if (error >180)
                error = ((gyroSensor.getHeading()-heading-360)/60);
            else if(error <-180)
                error = ((gyroSensor.getHeading()-heading+360)/60);

            leftSpeed = -0.4-error;
            rightSpeed = -0.3+error;

            leftSpeed = Range.clip(leftSpeed, -1, 1);
            rightSpeed = Range.clip(rightSpeed, -1, 1);

            lDrive1.setPower(leftSpeed);
            lDrive2.setPower(leftSpeed);
            rDrive1.setPower(rightSpeed);
            rDrive2.setPower(rightSpeed);
        }
        lDrive1.setPower(0);
        lDrive2.setPower(0);
        rDrive1.setPower(0);
        rDrive2.setPower(0);
    }
    // Function to line the robot up a certain distance from the wall using the range function
    private void lineUp() throws InterruptedException {

        I2cDevice RANGE1 = hardwareMap.i2cDevice.get("range");
        I2cDeviceSynch RANGE1Reader = new I2cDeviceSynchImpl(RANGE1, RANGE1ADDRESS, false);
        RANGE1Reader.engage();
        byte[] range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
        if ( (range1Cache[0] & 0xFF) > 13) {
            telemetry.addData("Ultra Sonic", range1Cache[0] & 0xFF);
            telemetry.update();
            rDrive1.setPower(-0.5);
            rDrive2.setPower(-0.5);

            sleep(400);

            rDrive1.setPower(0);
            rDrive2.setPower(0);

            while((range1Cache[0] & 0xFF) > 13) {
                range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
                rDrive1.setPower(-0.5);
                rDrive2.setPower(-0.5);
                lDrive1.setPower(-0.5);
                lDrive2.setPower(-0.5);
                telemetry.addData("Ultra Sonic", range1Cache[0] & 0xFF);
                telemetry.update();
            }
            rDrive1.setPower(0);
            rDrive2.setPower(0);
            lDrive1.setPower(0);
            lDrive2.setPower(0);

            sleep(100);

            telemetry.addData("Ultra Sonic", "NOICE");
            gyroTurn(45,.4,-1);
            driveToLine();
        }
        else if ((range1Cache[0] & 0xFF) <= 11){
            telemetry.addData("Ultra Sonic", range1Cache[0] & 0xFF);
            telemetry.update();
            sleep(1000);
            gyroTurn(45,.4,-1);
            rDrive1.setPower(-0.4);
            rDrive2.setPower(-0.4);
            lDrive1.setPower(-0.4);
            lDrive2.setPower(-0.4);
            sleep(200);
            rDrive1.setPower(0);
            rDrive2.setPower(0);
            lDrive1.setPower(0);
            lDrive2.setPower(0);
            driveToLine();
        }
        telemetry.addData("Ultra Sonic", range1Cache[0] & 0xFF);
        telemetry.update();
    }

    private void recognizeColor() throws InterruptedException {
        color.enableLed(false);
        //if the beacon is red
        if (color.red() > color.blue()){
            lButton.setPosition(.6);
            sleep(1000);
            lButton.setPosition(0);
            telemetry.addData("Color is red",color.red());
        }
        //if the beacon is blue
        while (color.red() < (color.blue())) {
            rDrive1.setPower(.3);
            rDrive2.setPower(.3);
            lDrive1.setPower(.3);
            lDrive2.setPower(.3);
        }
        rDrive1.setPower(0);
        rDrive2.setPower(0);
        lDrive1.setPower(0);
        lDrive2.setPower(0);
        lButton.setPosition(.6);
        sleep(1000);
        lButton.setPosition(0);
        telemetry.update();
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
        lButton = hardwareMap.servo.get("lButton");
        //rButton = hardwareMap.servo.get("rButton");
        hopper = hardwareMap.servo.get("hopper");
        touch = hardwareMap.touchSensor.get("t");
        color = hardwareMap.colorSensor.get("color");

        rODSensor = hardwareMap.opticalDistanceSensor.get("rOD");
        lODSensor = hardwareMap.opticalDistanceSensor.get("lOD");
        hopper.setPosition(.8);
        lButton.setPosition(0);
        setUpGyro();

        double distance;
        double maxSpeed;
        int targetHeading;
        int direction;
        long time;

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

        // Drive forward
        distance = 10;
        maxSpeed = 1;
        direction = 1;
        drive(distance, maxSpeed, direction);
        // Turn to face vortex
        targetHeading = 12;
        maxSpeed = .5;
        direction = 1;
        gyroTurn(targetHeading, maxSpeed, direction);
        // Fire balls
        fire();
        // Turn to hit cap ball
        targetHeading = 355;
        maxSpeed = .4;
        direction = -1;
        gyroTurn(targetHeading, maxSpeed, direction);
        // Drive forward
        distance = 89;
        maxSpeed = 1;
        direction = 1;
        drive(distance, maxSpeed, direction);
        // Turn towards line
        targetHeading = 44;
        maxSpeed = .5;
        direction = 1;
        gyroTurn(targetHeading, maxSpeed, direction);
        // Drive until the robot detects the line
        driveToLine();
        // Turn until parallel with wall
        targetHeading = 45;
        maxSpeed = .5;
        direction = 1;
        gyroTurn(targetHeading, maxSpeed, direction);
        // Drive backward
        time = 400;
        maxSpeed = .4;
        driveBackward(time, maxSpeed);
        // Adjust the robot's distance from the wall
        lineUp();
        time = 300;
        maxSpeed = .4;
        driveBackward(time,maxSpeed);
        // Detect beacon color and push the button for red
        recognizeColor();
        // Drive backward past the line
        time = 1000;
        maxSpeed = .5;
        driveBackward(time, maxSpeed);
        // Drive backward until we hit the second line
        driveBackwardToLine();
        // Drive forward
        distance = 2;
        maxSpeed = 1;
        direction = 1;
        drive(distance, maxSpeed, direction);
        // Detect beacon color and push the button for red
        recognizeColor();

    }
}
