package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Autonomous(name="cf_recognize_color", group="LinearOpMode")
@Disabled
public class cf_recognize_color extends OpMode{

    ColorSensor color;
    Servo hopper;
    DcMotor catapult;
    DcMotor sweeper;
    DcMotor lDrive1;
    DcMotor lDrive2;
    DcMotor rDrive1;
    DcMotor rDrive2;
    Servo lButton;
    Servo rButton;
    TouchSensor touch;

    public void recognizeColor() {

        color.enableLed(true);
        //if the beacon is red
        if (color.red() > color.blue()){
            //insert code here
            telemetry.addData("Color is red",color.red());
            updateTelemetry(telemetry);
        }
        //if the beacon is blue
        else if (color.blue() > color.red()) {
            //insert code here
            telemetry.addData("Color is blue",color.blue());
            updateTelemetry(telemetry);
        }
    }

    public void init() {
        color = hardwareMap.colorSensor.get("color");
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
    }


    public void loop() {

        recognizeColor();
        updateTelemetry(telemetry);

    }

}

