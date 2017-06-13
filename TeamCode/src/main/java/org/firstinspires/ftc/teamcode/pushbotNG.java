package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="pushbotNG", group="Opmode")

public class pushbotNG extends OpMode {
    public DcMotor l1, r1;
//    private Servo sweeper, wiper;

    public void init() {

        l1 = hardwareMap.dcMotor.get("l1");
        r1 = hardwareMap.dcMotor.get("r1");

        //l1.setDirection(DcMotor.Direction.REVERSE);

//        sweeper = hardwareMap.servo.get("sweeper");
//        wiper = hardwareMap.servo.get("wiper");
    }

    public void loop() {

        setDriveMotorSpeeds(gamepad1.left_stick_y, gamepad1.right_stick_y);
//        l1.setPower(-gamepad1.left_stick_y/4);
//        r1.setPower(gamepad1.right_stick_y/4);
//        if (gamepad1.x) {
//            engageWiper();
//        } else {
//            disengageWiper();
//        }
//
//        if (gamepad1.y) {
//            engageSweeper();
//        } else {
//            disengageSweeper();
//        }

    }

    public void setDriveMotorSpeeds(double left, double right) {
        l1.setPower(-left/4);
        r1.setPower(right/4);
    }

//    public void engageWiper() {
//        wiper.setPosition(1.0);
//    }
//
//    public void disengageWiper() {
//        wiper.setPosition(0.0);
//    }
//
//    public void engageSweeper() {
//        sweeper.setPosition(1.0);
//    }
//
//    public void disengageSweeper() {
//        sweeper.setPosition(0.0);
//    }
}
