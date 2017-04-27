package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name="GyroTurn", group="Methods")
@Disabled
public class GyroTurnMethod extends LinearOpMode {

    DcMotor lDrive1;
    DcMotor lDrive2;
    DcMotor rDrive1;
    DcMotor rDrive2;
    double[]pastError = new double[5];
    //ColorSensor colorSensor;
    GyroSensor gyroSensor;
    ModernRoboticsI2cGyro gyro;
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

    private void timedGyroTurn (int targetHeading, double time){
        //boolean done = false;
        double error;
        double currentHeading;
        double kp = .0055;
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

    private void PIDGyroTurn (int targetHeading, double time){

        double error;
        double currentHeading;
        double kp = 0.003; // 0.005 without encoders
        double ki = 0.000; // 0.00025 without encoders
        double kd = 0.002; // 0.002 without encoders
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

            if (error > 0)
                power = .15+(error*kp)+(integral(error)*ki)+(derivative(error,runtime.seconds())*kd);
            else if (error < 0)
                power = -.15+(error*kp)+(integral(error)*ki)+(derivative(error,runtime.seconds())*kd);
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
            //sleep(50);
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
        for( double i : pastError) {
            sum += i;
        }
        return
                sum;
    }

    private double derivative(double error, double time){
        speed = (error-lastError)/(time-lastTime);
        lastError = error;
        lastTime = time;
        return
                speed;
    }

    private void gyroTurn (int targetHeading){
        boolean done = false;
        double error;
        double currentHeading;
        double kp = .0035;
        double power;
        ElapsedTime runtime = new ElapsedTime();
        gyro.resetZAxisIntegrator();
        runtime.reset();
        sleep(250);

        while (!done && opModeIsActive()){
            currentHeading = -gyro.getIntegratedZValue();

            error = (targetHeading-currentHeading);
            power = error*kp;

            error = Range.clip(error, -.3, .3);
            if (error > 0 && error < .15)
                error = .15;
            else if (error < 0 && error > -.15)
                error = -.15;

            drive(0+power, 0-power);

            telemetry.addData("error", error);
            telemetry.addData("power", power);
            telemetry.addData("currentHeading", currentHeading);
            telemetry.addData("targetHeading", targetHeading);
            telemetry.update();

            if (currentHeading <= targetHeading+1 && currentHeading >= targetHeading-1)
                done = true;
            else
                done = false;

        }
        driveStop();
    }

    private void drive(double leftSpeed, double rightSpeed){
        lDrive1.setPower(leftSpeed);
        lDrive2.setPower(leftSpeed);
        rDrive1.setPower(rightSpeed);
        rDrive2.setPower(rightSpeed);
    }

    private void driveStop(){
        lDrive1.setPower(0);
        lDrive2.setPower(0);
        rDrive1.setPower(0);
        rDrive2.setPower(0);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        int direction;
        double maxSpeed;
        int targetHeading;
        double error;
        double currentHeading;

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
        lDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rDrive2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        idle();

        waitForStart();

        PIDGyroTurn(90,5);

        // 2.5 seconds for a 90 degree turn
        // 3 seconds for a 170 degree turn

    }
}