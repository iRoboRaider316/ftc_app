package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="ButtonPush",group="LinearOpMode")

public class dev_ButtonPush extends LinearOpMode{

    DcMotor Hand;

    public void ButtonPush(double power, long time) throws InterruptedException {
        Hand.setPower(power);
        sleep(time);
    }

    public void runOpMode() throws InterruptedException {
        Hand = hardwareMap.dcMotor.get("Hand");
        waitForStart();
        ButtonPush(0.9, 1000);
        ButtonPush(-0.9, 1000);
    }
}

