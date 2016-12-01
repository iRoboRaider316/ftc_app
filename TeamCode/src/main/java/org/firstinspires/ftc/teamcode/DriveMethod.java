package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="Drive Method", group="Methods")
//@Disabled
public class DriveMethod extends LinearOpMode {

    DcMotor lDrive1;
    DcMotor lDrive2;
    DcMotor rDrive1;

    GyroSensor gyroSensor;
    DcMotor rDrive2;
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

    /* This is the Drive Method
    It will take in two static values: distance and maxSpeed.
    It will then calculate the encoder counts to drive and drive the distance at the specified power,
    accelerating to max speed for the first third of the distance, maintaining that speed for the second third,
    and decelerating to a minimum speed for the last third.
    If the robot deviates from the initial gyro heading, it will correct itself proportionally to the error.*/
    public void drive(double distance, double maxSpeed) {
        int ENCODER_CPR = 1120; // Encoder counts per Rev
        double gearRatio = 1.75; // [Gear Ratio]:1
        double circumference = 13.10; // Wheel circumference
        double ROTATIONS = distance / (circumference * gearRatio); // Number of rotations to drive
        double COUNTS = ENCODER_CPR * ROTATIONS; // Number of encoder counts to drive
        double startPosition = rDrive1.getCurrentPosition();

        double oldSpeed;
        double speed = 0;
        double minSpeed = 0.3;
        double acceleration = 0.01;
        double leftSpeed;
        double rightSpeed;

        rDrive1.setTargetPosition(rDrive1.getCurrentPosition() + (int) COUNTS);

        rDrive1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lDrive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        gyro.resetZAxisIntegrator();

        double heading = gyro.getIntegratedZValue();

        while (rDrive1.getCurrentPosition()<(rDrive1.getTargetPosition()-5)){
            
            /*oldSpeed = speed;

            // Accelerate during first third of the distance
            if ((rDrive1.getCurrentPosition()-startPosition)<(0.33*COUNTS)){
                // Calculate the speed we should be moving at
                // Set a minimum power for the motors to make sure they move
                if (speed < minSpeed)
                    speed = minSpeed;
                    // Don't exceed the maximum speed requested
                else if (speed > maxSpeed)
                    speed = maxSpeed;
                    // accelerate
                else
                    speed = oldSpeed + acceleration;
            }

            // Maintain top speed during second third of the total distance
            else if ((rDrive1.getCurrentPosition()-startPosition)<(0.33*COUNTS)
                    && ((rDrive1.getCurrentPosition()-startPosition)<(0.66*COUNTS))){
                speed = maxSpeed;
            }

            // Decelerate to a minimum speed during the last third of the total distance
            else if ((rDrive1.getCurrentPosition()-startPosition)>(0.66*COUNTS)){
                // Calculate the speed we should be moving at
                // Set a minimum power for the motors to make sure they move
                if (speed < minSpeed)
                    speed = minSpeed;
                    // Don't exceed the maximum speed requested
                else if (speed > maxSpeed)
                    speed = maxSpeed;
                    // Decelerate
                else
                    speed = oldSpeed - acceleration;
            }
            // Default speed in case of encoder error
            else {
                speed = minSpeed;
            }

            // Adjust motor speeds to keep the robot on the initial heading
            leftSpeed = speed + ((gyro.getIntegratedZValue()-heading)/30);
            rightSpeed = speed - ((gyro.getIntegratedZValue()-heading)/30);*/
            leftSpeed = maxSpeed + ((gyro.getIntegratedZValue()-heading)/10);
            rightSpeed = maxSpeed - ((gyro.getIntegratedZValue()-heading)/10);

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


    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        double distance;
        double maxSpeed;

        lDrive1 = hardwareMap.dcMotor.get("lDrive1");
        lDrive2 = hardwareMap.dcMotor.get("lDrive2");
        rDrive1 = hardwareMap.dcMotor.get("rDrive1");
        rDrive2 = hardwareMap.dcMotor.get("rDrive2");
        lDrive1.setDirection(DcMotor.Direction.REVERSE);
        lDrive2.setDirection(DcMotor.Direction.REVERSE);

        setUpGyro();

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

        distance = 90;
        maxSpeed = .6;
        drive(distance, maxSpeed);
    }
}