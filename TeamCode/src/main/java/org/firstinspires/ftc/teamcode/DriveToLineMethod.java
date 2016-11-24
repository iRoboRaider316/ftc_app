package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;


@Autonomous(name="DriveToLine", group="Methods")
@Disabled
public class DriveToLineMethod extends LinearOpMode {

    DcMotor lDrive1;
    DcMotor lDrive2;
    DcMotor rDrive1;
    DcMotor rDrive2;

    OpticalDistanceSensor rODSensor;
    OpticalDistanceSensor lODSensor;

    // This is the driveToLine method.
    // When called, the robot drives forward until the optical distance sensor detects a white line on the mat.
    public void driveToLine() {
        lDrive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rDrive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rODSensor.enableLed(true);
        lODSensor.enableLed(true);
        while(rODSensor.getRawLightDetected()<0.21&&lODSensor.getRawLightDetected()<0.21){
            telemetry.addData("rLight", rODSensor.getRawLightDetected());
            telemetry.addData("lLight", lODSensor.getRawLightDetected());
            telemetry.update();
            lDrive1.setPower(.3);
            lDrive2.setPower(.3);
            rDrive1.setPower(.3);
            rDrive2.setPower(.3);
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
