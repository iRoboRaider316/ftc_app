package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="hehexdev_stuff", group="LinearOpMode")
@Disabled
public class dev_AdjustToLine extends LinearOpMode {

    DcMotor sweeper;
    Servo Button;
    Servo hopper;
    DcMotor catapult;
    DcMotor lDrive1;
    DcMotor lDrive2;
    DcMotor rDrive1;
    DcMotor rDrive2;
    ColorSensor color;
    GyroSensor gyroSensor;
    TouchSensor touch;
    OpticalDistanceSensor rODSensor;
    OpticalDistanceSensor lODSensor;
    ModernRoboticsI2cGyro gyro;

    double MAX_SPEED = 1;
    double MIN_SPEED = 0.4;

    private void setUpGyro() throws InterruptedException {
        // setup the Gyro
        // write some device information (connection info, name and type)
        // to the log file.
        hardwareMap.logDevices();
        // get a reference to our GyroSensor object.
        gyroSensor = hardwareMap.gyroSensor.get("gyro");
        // calibrate the gyro.
        gyroSensor.calibrate();
        while (gyroSensor.isCalibrating())  {
            sleep(50);
        }
        // End of setting up Gyro
    }

    public void moveMotors(double left, double right, long time) throws InterruptedException {
        rDrive1.setPower(right);        // moves forward at certain power...
        rDrive2.setPower(right);
        lDrive1.setPower(left);
        lDrive2.setPower(left);
        sleep(time);               // ...until it's been running for a certain time.
        rDrive1.setPower(0);            // at that point, the robot stops...
        rDrive2.setPower(0);
        lDrive1.setPower(0);
        lDrive2.setPower(0);
        sleep(500);                     // ...and waits a half second.
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

    public void gyroTurn(int targetHeading, double maxSpeed, int direction) {
        int startHeading = gyroSensor.getHeading();
        int deceleration;
        int currentHeading;
        double oldSpeed;
        double speed = 0;
        double minSpeed = 0.05;
        double acceleration = 0.01;
        double ka = 0.01;             // Proportional acceleration constant
        lDrive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rDrive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

    public void drive(double distance, double maxSpeed) {
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

        rDrive1.setTargetPosition(rDrive1.getCurrentPosition() + (int) COUNTS);

        rDrive1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lDrive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double heading = gyroSensor.getHeading();

        while (rDrive1.getCurrentPosition()<(rDrive1.getTargetPosition()-5)){

            leftSpeed = maxSpeed-((gyroSensor.getHeading()-heading)/10);
            rightSpeed = maxSpeed+((gyroSensor.getHeading()-heading)/10);

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


    public void AdjustToLine() throws InterruptedException { // this code has no parameters.
        lDrive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rDrive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rODSensor.enableLed(true);
        lODSensor.enableLed(true);
        if(lODFoundLine()) {
            moveMotors(MIN_SPEED, MIN_SPEED, 600);
            gyroTurn(45, MIN_SPEED, 1);
            while(!lODFoundLine()) {
                rDrive1.setPower(-MIN_SPEED);
                rDrive2.setPower(-MIN_SPEED);
                lDrive1.setPower(-MIN_SPEED);
                lDrive2.setPower(-MIN_SPEED);
            }
        } else if(rODFoundLine()) {
            while(!lODFoundLine()) {
                lDrive1.setPower(MIN_SPEED);
                lDrive2.setPower(MIN_SPEED);
                rDrive1.setPower(MIN_SPEED);
                rDrive2.setPower(MIN_SPEED);
            }
            gyroTurn(45, MIN_SPEED, 1);
            while(!lODFoundLine()) {
                rDrive1.setPower(-MIN_SPEED);
                rDrive2.setPower(-MIN_SPEED);
                lDrive1.setPower(-MIN_SPEED);
                lDrive2.setPower(-MIN_SPEED);
            }
        }
    }

    public boolean rODFoundLine() { // checks right OD sensor for light greater than 0.11
        return rODSensor.getLightDetected() >= 0.11;
    }

    public boolean lODFoundLine() { // checks left OD sensor for light greater than 0.11
        return lODSensor.getLightDetected() >= 0.11;
    }

    public void runOpMode() throws InterruptedException {

        rDrive1 = hardwareMap.dcMotor.get("rDrive1");
        rDrive2 = hardwareMap.dcMotor.get("rDrive2");
        lDrive1 = hardwareMap.dcMotor.get("lDrive1");
        lDrive2 = hardwareMap.dcMotor.get("lDrive2");
        sweeper = hardwareMap.dcMotor.get("sweeper");
        catapult = hardwareMap.dcMotor.get("catapult");
        Button = hardwareMap.servo.get("lButton");
        hopper = hardwareMap.servo.get("hopper");
        touch = hardwareMap.touchSensor.get("t");
        color = hardwareMap.colorSensor.get("color");
        rODSensor = hardwareMap.opticalDistanceSensor.get("rOD");
        lODSensor = hardwareMap.opticalDistanceSensor.get("lOD");
        lDrive2.setDirection(DcMotor.Direction.REVERSE);
        lDrive1.setDirection(DcMotor.Direction.REVERSE);
        setUpGyro();
        lDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();
        lDrive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rDrive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        AdjustToLine();
    }
}