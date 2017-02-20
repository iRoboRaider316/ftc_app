package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.AdafruitIMUcode.bno055driver;

@Autonomous(name="IMUGyro",group="LinearOpMode")
@Disabled
public class AdafruitTest_3 extends LinearOpMode {

    public bno055driver imu;

    DcMotor lDrive1;
    DcMotor lDrive2;
    DcMotor rDrive1;
    DcMotor rDrive2;

    public void IMU_GyroTurn(double heading, double power, int direction) {

        double Target = heading;
        double speed;
        double speedBoost = 0;

        if(direction == 1) {
            Target = imu.getAngles()[0] + heading;
            Target -= 20;
        }
        if(direction == -1) {
            Target = imu.getAngles()[0] + -heading;
            Target += 20;
        }

        if(Target > 180) {
            Target -= 360;
        }
        if(Target <= -180) {
            Target += 360;
        }
        lDrive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rDrive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while(!turnComplete(Target, imu.getAngles()[0])) {

            speedBoost = 0.1 * direction;
            speed = (power * direction) + speedBoost;

            lDrive1.setPower(speed);
            lDrive2.setPower(speed);
            rDrive2.setPower(speed);
            rDrive1.setPower(speed);

            telemetry.addData("Values", imu.getAngles()[0]);
            telemetry.addData("Speed", speed);
            updateTelemetry(telemetry);

        }

        lDrive1.setPower(0);
        lDrive2.setPower(0);
        rDrive1.setPower(0);
        rDrive2.setPower(0);

        while(!turnComplete(Target, imu.getAngles()[0])) {

            speed = (power * direction);

            lDrive1.setPower(speed);
            lDrive2.setPower(speed);
            rDrive2.setPower(speed);
            rDrive1.setPower(speed);

            telemetry.addData("Values", imu.getAngles()[0]);
            telemetry.addData("Speed", speed);
            updateTelemetry(telemetry);

        }

        lDrive1.setPower(0);
        lDrive2.setPower(0);
        rDrive1.setPower(0);
        rDrive2.setPower(0);
        for(int i = 0; i < 5000; i++) {
            sleep(1);
            telemetry.addData("Values", imu.getAngles()[0]);
            telemetry.addData("Time Left", i);
            updateTelemetry(telemetry);
        }
    }

    public boolean turnComplete(double targetHeading, double currentHeading) {
        double minrange = targetHeading - 2;
        double maxrange = targetHeading + 2;
        double minXD = 0;
        double maxXD = 0;
        boolean result;

        if(minrange <= -180) {
            minrange += 360;
            minXD = 360;
        }
        if(maxrange > 180) {
            maxrange -= 360;
            maxXD = 360;
        }

        result = (currentHeading + minXD >= minrange && currentHeading - maxXD <= maxrange);

        return result;
    }

    public void runInitSequence()
    {
        imu = new bno055driver("imu", hardwareMap);
        rDrive1 = hardwareMap.dcMotor.get("rDrive1");
        rDrive2 = hardwareMap.dcMotor.get("rDrive2");
        lDrive1 = hardwareMap.dcMotor.get("lDrive1");
        lDrive2 = hardwareMap.dcMotor.get("lDrive2");
    }

    public void runOpMode() throws InterruptedException {
        runInitSequence();

        waitForStart();

        telemetry.addData("Say", "GET ROBOT INTO POSITION!!!");
        for(int i = 0; i < 5000; i++) {
            sleep(1);
            telemetry.addData("GET ROBOT INTO POSITION!!!", i);
            telemetry.addData("Values", imu.getAngles()[0]);
            updateTelemetry(telemetry);
        }
        IMU_GyroTurn(-83, 0.2, -1);
    }
}