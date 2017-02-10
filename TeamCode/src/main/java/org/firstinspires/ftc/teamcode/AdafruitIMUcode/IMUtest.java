package org.firstinspires.ftc.teamcode.AdafruitIMUcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "IMU Test", group = "TeleOp")

public class IMUtest extends LinearOpMode
{
    //Adafruit IMU
    bno055driver imu;

    @Override
    public void runOpMode()
    {
        //Init everything
        runInitSequence();

        //Wait for the start button to be pressed.
        waitForStart();

        while(opModeIsActive())
        {
            telemetry.addLine("Yaw: " + imu.getAngles()[0]);
            telemetry.addLine("Pitch: " + imu.getAngles()[1]);
            telemetry.addLine("Roll: " + imu.getAngles()[2]);
            telemetry.update();
        }
    }

    public void runInitSequence()
    {
        imu = new bno055driver("INSERT IMU NAME FROM CONFIG FILE HERE", hardwareMap);
    }
}
