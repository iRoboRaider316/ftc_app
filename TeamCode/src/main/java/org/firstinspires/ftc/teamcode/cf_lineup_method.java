package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

@Autonomous(name="dev_to_wall", group="LinearOpMode")
@Disabled

public class cf_lineup_method extends LinearOpMode {
    ColorSensor color;
    Servo hopper;
    DcMotor catapult;
    DcMotor sweeper;
    DcMotor lDrive1;
    DcMotor lDrive2;
    DcMotor rDrive1;
    DcMotor rDrive2;
    Servo lButton;
    Servo rButton;
    TouchSensor touch;
    GyroSensor gyroSensor;
    byte[] range1Cache;
    I2cDevice RANGE1;
    I2cDeviceSynch RANGE1Reader;
    private OpticalDistanceSensor rODSensor;
    private OpticalDistanceSensor lODSensor;

    I2cAddr RANGE1ADDRESS = new I2cAddr(0x14); //Default I2C address for MR Range (7-bit)
    public static final int RANGE1_REG_START = 0x04; //Register to start reading
    public static final int RANGE1_READ_LENGTH = 2; //Number of byte to read


    // Function to line the robot up a certain distance from the wall using the range function
    public void lineUp() throws InterruptedException {

        I2cDevice RANGE1 = hardwareMap.i2cDevice.get("range");
        I2cDeviceSynch RANGE1Reader = new I2cDeviceSynchImpl(RANGE1, RANGE1ADDRESS, false);
        RANGE1Reader.engage();
        byte[] range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
        if ( (range1Cache[0] & 0xFF) > 11) {
            telemetry.addData("Ultra Sonic", range1Cache[0] & 0xFF);
            telemetry.update();
            rDrive1.setPower(-0.4);
            rDrive2.setPower(-0.4);

            sleep(400);

            rDrive1.setPower(0);
            rDrive2.setPower(0);

            while((range1Cache[0] & 0xFF) > 11) {
                range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);
                rDrive1.setPower(-0.4);
                rDrive2.setPower(-0.4);
                lDrive1.setPower(-0.4);
                lDrive2.setPower(-0.4);
                telemetry.addData("Ultra Sonic", range1Cache[0] & 0xFF);
                telemetry.update();
            }
            rDrive1.setPower(0);
            rDrive2.setPower(0);
            lDrive1.setPower(0);
            lDrive2.setPower(0);

            sleep(100);

            telemetry.addData("Ultra Sonic", "NOICE");
            gyroTurn(44,.3,-1);
            //driveToLine();
        }
        else if ((range1Cache[0] & 0xFF) <= 11){
            telemetry.addData("Ultra Sonic", range1Cache[0] & 0xFF);
            telemetry.update();
            sleep(1000);
        }
        telemetry.addData("Ultra Sonic", range1Cache[0] & 0xFF);
        telemetry.update();
    }

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

    public void driveToLine() {
        lDrive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rDrive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        double leftSpeed;
        double rightSpeed;

        double heading = gyroSensor.getHeading();

        rODSensor.enableLed(true);
        lODSensor.enableLed(true);
        while(rODSensor.getRawLightDetected()<0.21&&lODSensor.getRawLightDetected()<0.21&&opModeIsActive()){
            telemetry.addData("rLight", rODSensor.getRawLightDetected());
            telemetry.addData("lLight", lODSensor.getRawLightDetected());
            telemetry.update();

            leftSpeed = 0.5 - ((gyroSensor.getHeading()-heading)/40);
            rightSpeed = 0.5 + ((gyroSensor.getHeading()-heading)/40);
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

    public void runOpMode() throws InterruptedException {
        //##############Init##############
        rDrive1 = hardwareMap.dcMotor.get("rDrive1");
        rDrive2 = hardwareMap.dcMotor.get("rDrive2");
        lDrive1 = hardwareMap.dcMotor.get("lDrive1");
        lDrive2 = hardwareMap.dcMotor.get("lDrive2");
        sweeper = hardwareMap.dcMotor.get("sweeper");
        catapult = hardwareMap.dcMotor.get("catapult");
        lButton = hardwareMap.servo.get("lButton");
        rButton = hardwareMap.servo.get("rButton");
        hopper = hardwareMap.servo.get("hopper");
        touch = hardwareMap.touchSensor.get("t");
        color = hardwareMap.colorSensor.get("color");
        rODSensor = hardwareMap.opticalDistanceSensor.get("rOD");
        lODSensor = hardwareMap.opticalDistanceSensor.get("lOD");


        waitForStart();
        lineUp();


    }
}
