package org.firstinspires.ftc.teamcode;

//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

//import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by rahm on 6/6/17.
 */

public class Bobo1 {
    private DcMotor l1, r1;
    private Servo sweeper, wiper;
//    private Telemetry telemetry;

//    private int lencoder;
//    private int rencoder;

//   Bobo1(HardwareMap hardwareMap, Telemetry _telemetry) throws InterruptedException {
    Bobo1(HardwareMap hardwareMap) throws InterruptedException {
        l1 = hardwareMap.dcMotor.get("l1");
        r1 = hardwareMap.dcMotor.get("r1");
        sweeper = hardwareMap.servo.get("sweeper");
        wiper = hardwareMap.servo.get("wiper");

        l1.setDirection(DcMotor.Direction.REVERSE);

//        telemetry = _telemetry;
    }

    protected void setDriveMotorSpeeds(double left, double right) {
        l1.setPower(Range.clip(left, -1, 1));
        r1.setPower(Range.clip(right, 1, 1));
    }

//    public void sendTelemetry() {
//        telemetry.addData("Encoders", String.format("L: %d, R: %d",
//                getLeftEncoder(),
//                getRightEncoder()
//        ));
//    }
//
//    public int getLeftEncoder() {
//        return -(l1.getCurrentPosition() - lencoder);
//    }
//
//    public int getRightEncoder() {
//        return -(r1.getCurrentPosition() - rencoder);
//    }
//
//    public void zeroEncoders() {
//        lencoder = l1.getCurrentPosition();
//        rencoder = r1.getCurrentPosition();
//    }

    public static final double FULL_SPEED = 1.0;
    public static final double HALF_SPEED = 0.5;

    public void turnLeft() {
        setDriveMotorSpeeds(-FULL_SPEED, FULL_SPEED);
    }

    public void turnLeftSlowly() {
        setDriveMotorSpeeds(-HALF_SPEED, HALF_SPEED);
    }

    public void turnRight() {
        setDriveMotorSpeeds(FULL_SPEED, -FULL_SPEED);
    }

    public void turnRightSlowly() {
        setDriveMotorSpeeds(HALF_SPEED, -HALF_SPEED);
    }

    public void driveForward() {
        setDriveMotorSpeeds(FULL_SPEED, FULL_SPEED);
    }

    public void driveForwardSlowly() {
        setDriveMotorSpeeds(HALF_SPEED, HALF_SPEED);
    }

    public void driveBackward() {
        setDriveMotorSpeeds(-FULL_SPEED, -FULL_SPEED);
    }

    public void driveBackwardSlowly() {
        setDriveMotorSpeeds(-HALF_SPEED, -HALF_SPEED);
    }

    public void fullStop() {
        setDriveMotorSpeeds(0.0, 0.0);
    }

    public void engageWiper() {
        wiper.setPosition(1.0);
    }

    public void disengageWiper() {
        wiper.setPosition(0.0);
    }

    public void engageSweeper() {
        sweeper.setPosition(1.0);
    }

    public void disengageSweeper() {
        sweeper.setPosition(0.0);
    }

}