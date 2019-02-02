package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Cheerios Teleop (Real Deal)", group="Opmode")
public class Atlas_teleopClass extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        AtlasClass base = new AtlasClass(hardwareMap, telemetry, gamepad1, gamepad2);

        telemetry.addData("", "Successfully Initialized!");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            base.updateDriveTrain();
            base.updateLiftM();
            base.updateExtendM();
            base.updateCollectFlipperM();
            base.updateCollectSpinnerM();

            telemetry.addData("Status Updates", "Brought to you by Gucci Gang :)");
            telemetry.addData(" - target", base.target);
            telemetry.addData(" - current pos", base.liftM.getCurrentPosition());
            telemetry.addData(" - first", base.first);
            telemetry.update();
        }
    }
}