package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
@Autonomous(name="cf_button_push", group="LinearOpMode")
public class cf_buttonPush extends LinearOpMode{

    Servo lHand;
    Servo rHand;
    DcMotor lDrive1;
    DcMotor lDrive2;
    DcMotor rDrive1;
    DcMotor rDrive2;
    ColorSensor color;

    public void buttonPush() throws InterruptedException {

        color.enableLed(false);
        //if the beacon is red
        if (color.red() > color.blue()){
            //insert code here
            lButtonPush();
            telemetry.addData("Color is red",color.red());
            sleep(1000);
        }
        //if the beacon is blue
        else if (color.blue() > color.red()) {
            rDrive1.setPower(0.3);            // at that point, the robot stops...
            rDrive2.setPower(0.3);
            lDrive1.setPower(0.3);
            lDrive2.setPower(0.3);
            sleep(750);
            rDrive1.setPower(0);            // at that point, the robot stops...
            rDrive2.setPower(0);
            lDrive1.setPower(0);
            lDrive2.setPower(0);
            sleep(500);
            lButtonPush();
            telemetry.addData("Color is blue", color.blue());
            sleep(1000);
        }

    }

    public void lButtonPush() throws InterruptedException {
        lHand.setPosition(0.5);
        sleep(1000);
    }

    public void rButtonPush() throws InterruptedException {
        rHand.setPosition(0.5);
        sleep(1000);
    }

    public void runOpMode() throws InterruptedException {
        color = hardwareMap.colorSensor.get("color");
        lHand = hardwareMap.servo.get("lButton");
        rHand = hardwareMap.servo.get("lButton");
        rDrive1 = hardwareMap.dcMotor.get("rDrive1");
        rDrive2 = hardwareMap.dcMotor.get("rDrive2");
        lDrive1 = hardwareMap.dcMotor.get("lDrive1");
        lDrive2 = hardwareMap.dcMotor.get("lDrive2");

        waitForStart();
        telemetry.addData("Color is red",color.red());
        telemetry.addData("Color is blue", color.blue());
        buttonPush();
        sleep(3000);
    }
}
