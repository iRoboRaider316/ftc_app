package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.adafruit.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.Quaternion;

import java.util.Locale;

/**
 * This is an Adafruit Bosch BNO055 Inertial Measurement Unit.
 * It gets data in quaternions from the fused sensor data, and returns yaw, pitch, and roll.
 * Created by Varun Singh, Lead Programmer of FTC Team 4997 Masquerade.
 */

@Autonomous(name="IMUGyro",group="LinearOpMode")

public class AdafruitTest_3 extends LinearOpMode {

    public BNO055IMU imu;

    DcMotor lDrive1;
    DcMotor lDrive2;
    DcMotor rDrive1;
    DcMotor rDrive2;

    private void setParameters() {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.useExternalCrystal = true;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.pitchMode = BNO055IMU.PitchMode.WINDOWS;
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
    }

    /**
     * This method returns a 3x1 array of doubles with the yaw, pitch, and roll in that order.
     * The equations used in this method came from:
     * https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles#Euler_Angles_from_Quaternion
     */
    public double[] getAngles() {
        Quaternion quatAngles = imu.getQuaternionOrientation();
        double w = quatAngles.w;
        double x = quatAngles.x;
        double y = quatAngles.y;
        double z = quatAngles.z;
        // for the Adafruit IMU, yaw and roll are switched
        double roll = Math.atan2( 2*(w*x + y*z) , 1 - 2*(x*x + y*y) ) * 180.0 / Math.PI;
        double pitch = Math.asin( 2*(w*y - x*z) ) * 180.0 / Math.PI;
        double yaw = Math.atan2( 2*(w*z + x*y), 1 - 2*(y*y + z*z) ) * 180.0 / Math.PI;
        return new double[]{yaw, pitch, roll};
    }

    // This method returns a string that can be used to output telemetry data easily in other classes.
    public String telemetrize() {
        double[] angles = getAngles();
        return String.format(Locale.US, "Yaw: %.3f  Pitch: %.3f  Roll: %.3f", angles[0], angles[1], angles[2]);
    }

    public void IMU_GyroTurn(double heading, double power, int direction) {

        double speed;
        double decel = 0;
        double subtract;

        lDrive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rDrive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while(!turnComplete(heading, getAngles()[0])) {

            double speedBoost = 0;
            if(getAngles()[0] + 20 <= heading && direction == 1) {
                speedBoost = 0.1;
            } else if(getAngles()[0] - 20 >= heading && direction == -1) {
                speedBoost = -0.1;
            }
            speed = (power * direction) + speedBoost;

            lDrive1.setPower(speed);
            lDrive2.setPower(speed);
            rDrive2.setPower(speed);
            rDrive1.setPower(speed);

            telemetry.addData("Values", telemetrize());
            telemetry.addData("Speed", speed);
            updateTelemetry(telemetry);

        }

        lDrive1.setPower(0);
        lDrive2.setPower(0);
        rDrive1.setPower(0);
        rDrive2.setPower(0);
        sleep(10000);
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

    public void runOpMode() throws InterruptedException {
        setParameters();
        rDrive1 = hardwareMap.dcMotor.get("rDrive1");
        rDrive2 = hardwareMap.dcMotor.get("rDrive2");
        lDrive1 = hardwareMap.dcMotor.get("lDrive1");
        lDrive2 = hardwareMap.dcMotor.get("lDrive2");

        waitForStart();

        IMU_GyroTurn(90, 0.22, 1);
        IMU_GyroTurn(0, 0.22, -1);

    }
}
