package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

@Autonomous(name="cf_recognize_color", group="LinearOpMode")
//@Disabled

public class cf_recognize_color extends LinearOpMode {

    ColorSensor color;

    public void recognizeColor() {

        //if the beacon is red
        if (color.red() > color.blue()){
            //insert code here
            telemetry.addLine("Color is red");
            telemetry.addData("Red", color.red());
            telemetry.addData("Blue", color.blue());
        }
        //if the beacon is blue
        else if (color.blue() > color.red()) {
            //insert code here
            telemetry.addLine("Color is blue");
            telemetry.addData("Red", color.red());
            telemetry.addData("Blue", color.blue());
        }
        else {
            telemetry.addData("Red", color.red());
            telemetry.addData("Blue", color.blue());
        }
    }

    public void runOpMode() {
        color = hardwareMap.colorSensor.get("color");
        color.enableLed(false);
        waitForStart();
        color.enableLed(true);
        while(opModeIsActive()){
            recognizeColor();
            updateTelemetry(telemetry);
        }
        color.enableLed(false);
    }

}

