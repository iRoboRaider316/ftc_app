package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class cf_buttonPush extends LinearOpMode{

    Servo lHand;
    Servo rHand;

    public void lButtonPush() throws InterruptedException {
        lHand.setPosition(0.5);
        sleep(1000);
    }

    public void rButtonPush() throws InterruptedException {
        rHand.setPosition(0.5);
        sleep(1000);
    }

    public void runOpMode() {}
}
