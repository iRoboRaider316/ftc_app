package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

public class dev_AdjustToLine extends LinearOpMode {

    Servo feeder;
    Servo lButtonPush;
    Servo rButtonPush;
    DcMotor catapult1;
    DcMotor catapult2;
    DcMotor paddle;
    DcMotor lDrive1;
    DcMotor lDrive2;
    DcMotor rDrive1;
    DcMotor rDrive2;
    GyroSensor gyroSensor;
    OpticalDistanceSensor rODSensor;
    OpticalDistanceSensor lODSensor;

    boolean ODCondition;

    double MIN_SPEED = 0.3;
    double MAX_SPEED = 1;

    public void driveToLine() throws InterruptedException { // this code has no parameters.
        lDrive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rDrive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rODSensor.enableLed(true);
        while(!ODCondition) {
            if(rODFoundLine()) {
                if(lODFoundLine()) {
                    // runs when the right and left sensors see the line
                    lDrive1.setPower(0);
                    lDrive2.setPower(0);
                    rDrive1.setPower(0);
                    rDrive2.setPower(0);
                    ODCondition = true; // exits the code
                } else {
                    // runs when the right sensor see the line
                    lDrive1.setPower(MIN_SPEED);
                    lDrive2.setPower(MIN_SPEED);
                    rDrive1.setPower(0);
                    rDrive2.setPower(0);
                }
            } else {
                if(lODFoundLine()) {
                    // runs when the left sensor see the line
                    lDrive1.setPower(0);
                    lDrive2.setPower(0);
                    rDrive1.setPower(MIN_SPEED);
                    rDrive2.setPower(MIN_SPEED);
                } else {
                    // runs when neither right nor left sensor sees the line
                    lDrive1.setPower(MIN_SPEED);
                    lDrive2.setPower(MIN_SPEED);
                    rDrive1.setPower(MIN_SPEED);
                    rDrive2.setPower(MIN_SPEED);
                }
            }

            telemetry.addData("rLight", rODSensor.getRawLightDetected());
            telemetry.addData("lLight", lODSensor.getRawLightDetected());
            telemetry.update();
        }
    }

    public boolean rODFoundLine() { // checks right OD sensor for light greater than 0.21
        return rODSensor.getRawLightDetected() > 0.21;
    }

    public boolean lODFoundLine() { // checks left OD sensor for light greater than 0.21
        return lODSensor.getRawLightDetected() > 0.21;
    }

    public void runOpMode() throws InterruptedException {
        driveToLine();
    }

}

