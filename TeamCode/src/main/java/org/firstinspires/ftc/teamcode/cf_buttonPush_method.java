package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class cf_buttonPush_method extends LinearOpMode{

    Servo lButton;
    Servo rButton;

    public void lButtonPush() throws InterruptedException {
        lButton.setPosition(0.5);
        sleep(2000);
        lButton.setPosition(0);
    }

    public void rButtonPush() throws InterruptedException {
        rButton.setPosition(0.5);
        sleep(2000);
        rButton.setPosition(0);
    }

    public void runOpMode() throws InterruptedException {

        lButton = hardwareMap.servo.get("lButton");
        rButton = hardwareMap.servo.get("lButton");

        waitForStart();

        lButtonPush();
        rButtonPush();
    }
}
