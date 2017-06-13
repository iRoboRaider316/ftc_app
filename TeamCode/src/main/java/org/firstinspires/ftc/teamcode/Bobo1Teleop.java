package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by rahm on 6/6/17.
 */

@TeleOp(name="Bobo1Teleop", group="Opmode")
@Disabled
public class Bobo1Teleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        Bobo1 bobo1 = new Bobo1(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {

            // Set drive power with gamepad sticks
            bobo1.setDriveMotorSpeeds(-gamepad1.left_stick_y, -gamepad1.right_stick_y);

            // Control server with x button
            if (gamepad1.x) {
                bobo1.engageWiper();
            } else {
                bobo1.disengageWiper();
            }

            if (gamepad1.y) {
                bobo1.engageSweeper();
            } else {
                bobo1.disengageSweeper();
            }

        }

        bobo1.fullStop();
    }
}
