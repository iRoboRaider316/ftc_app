package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.Range;


@Autonomous(name="DriveToLine", group="Methods")
@Disabled
public class cf_driveToLine_method extends LinearOpMode {

    DcMotor lDrive1;
    DcMotor lDrive2;
    DcMotor rDrive1;
    DcMotor rDrive2;

    OpticalDistanceSensor rODSensor;
    OpticalDistanceSensor lODSensor;
    private GyroSensor gyroSensor;
    private ModernRoboticsI2cGyro gyro;

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


    // This is the driveToLine method.
    // When called, the robot drives forward until the optical distance sensor detects a white line on the mat.
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

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        lDrive1 = hardwareMap.dcMotor.get("lDrive1");
        lDrive2 = hardwareMap.dcMotor.get("lDrive2");
        rDrive1 = hardwareMap.dcMotor.get("rDrive1");
        rDrive2 = hardwareMap.dcMotor.get("rDrive2");
        lDrive1.setDirection(DcMotor.Direction.REVERSE);
        lDrive2.setDirection(DcMotor.Direction.REVERSE);

        rODSensor = hardwareMap.opticalDistanceSensor.get("rOD");
        lODSensor = hardwareMap.opticalDistanceSensor.get("lOD");

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

        driveToLine();
    }
}
