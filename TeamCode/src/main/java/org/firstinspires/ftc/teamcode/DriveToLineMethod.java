package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.util.Range;


@Autonomous(name="OD_telemetry", group="Methods")
@Disabled

public class DriveToLineMethod extends LinearOpMode {

    DcMotor lDrive1;
    DcMotor lDrive2;
    DcMotor rDrive1;
    DcMotor rDrive2;
    GyroSensor gyroSensor;
    ModernRoboticsI2cGyro gyro;

    OpticalDistanceSensor fODSensor;
    OpticalDistanceSensor bODSensor;

    public void driveToLine() {
        lDrive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rDrive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        fODSensor.enableLed(true);
        bODSensor.enableLed(true);


        while(opModeIsActive()){
       telemetry.addData("Driving...", fODSensor.getLightDetected());
            telemetry.addData("Still Driving...", bODSensor.getLightDetected());
            telemetry.update();
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
        double distance;
        double maxSpeed;

        lDrive1 = hardwareMap.dcMotor.get("lDrive1");
        lDrive2 = hardwareMap.dcMotor.get("lDrive2");
        rDrive1 = hardwareMap.dcMotor.get("rDrive1");
        rDrive2 = hardwareMap.dcMotor.get("rDrive2");
        lDrive1.setDirection(DcMotor.Direction.REVERSE);
        lDrive2.setDirection(DcMotor.Direction.REVERSE);
        fODSensor = hardwareMap.opticalDistanceSensor.get("fOD");
        bODSensor = hardwareMap.opticalDistanceSensor.get("bOD");

        waitForStart();
        while(opModeIsActive()){
            telemetry.addData("fOD", fODSensor.getLightDetected());
            telemetry.addData("bOD", bODSensor.getLightDetected());
            telemetry.update();
        }

    }
}
