package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Autonomous(name="hehexdev_stuff", group="LinearOpMode")
public class dev_AdjustToLine extends LinearOpMode {

    DcMotor sweeper;
    Servo lButton;
    Servo rButton;
    Servo hopper;
    DcMotor catapult;
    DcMotor lDrive1;
    DcMotor lDrive2;
    DcMotor rDrive1;
    DcMotor rDrive2;
    ColorSensor color;
    GyroSensor gyroSensor;
    TouchSensor touch;
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
        lDrive2.setDirection(DcMotor.Direction.REVERSE);
        rDrive1.setDirection(DcMotor.Direction.REVERSE);
        lDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rDrive2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();
        lDrive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rDrive1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rDrive2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        driveToLine();
    }
}
