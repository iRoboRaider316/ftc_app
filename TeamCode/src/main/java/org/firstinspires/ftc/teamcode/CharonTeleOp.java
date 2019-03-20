package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="Charon Teleop", group="OpMode")
public class CharonTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        CharonBase charon = new CharonBase(this);
        telemetry.addData("Initialized Successfully!", "");
        telemetry.update();
        waitForStart();

        while(opModeIsActive()) {
            charon.updateCollectHopperM();
            charon.updateCollectSpinnerM();
            charon.updateDumpS();
            charon.updateExtendM();
            charon.updateLiftM();
            charon.updateLocks();

            charon.updateDriveTrain();
        }
    }
} // IT'S BYTE-SIZED :D