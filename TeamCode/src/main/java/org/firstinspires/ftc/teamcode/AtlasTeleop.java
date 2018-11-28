package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="AtlasTeleop", group="OpMode")
public class AtlasTeleop extends OpMode {

    /*DcMotor lfDriveM;
    DcMotor lbDriveM;
    DcMotor rfDriveM;
    DcMotor rbDriveM;*/
    DcMotor liftM;
    DcMotor extendM;
    DcMotor collectFlipperM;
    DcMotor collectSpinnerM;
    Servo dumpS;

    @Override
    public void init() {
        /*lfDriveM = hardwareMap.dcMotor.get("lfDriveM");
        lbDriveM = hardwareMap.dcMotor.get("lfDriveM");
        rfDriveM = hardwareMap.dcMotor.get("lfDriveM");
        rbDriveM = hardwareMap.dcMotor.get("lfDriveM");*/
        liftM = hardwareMap.dcMotor.get("liftM");
        extendM = hardwareMap.dcMotor.get("extendM");
        collectFlipperM = hardwareMap.dcMotor.get("collectFlipperM");
        collectSpinnerM = hardwareMap.dcMotor.get("collectSpinnerM");
        dumpS = hardwareMap.servo.get("dumpS");
        telemetry.addData("", "Successfully Initialized!");
    }

    @Override
    public void loop() {
        liftM.setPower(gamepad2.right_stick_y);
        dumpS.setPosition(gamepad2.right_bumper ? 1 : 0);

        extendM.setPower(gamepad2.left_stick_y);
        collectFlipperM.setPower(gamepad2.right_trigger - gamepad2.left_trigger);
        collectSpinnerM.setPower(gamepad2.dpad_up ? 1 : gamepad2.dpad_down ? -1 : 0);

    }
}
