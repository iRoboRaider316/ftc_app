package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous (name="Charon Autonomous",group="Opmode")
public class CharonAutonomous extends LinearOpMode {

    @Override
    public void runOpMode() {
        CharonBase charon = new CharonBase(this);
        charon.initAuto(hardwareMap);
        charon.selection(this);
        waitForStart();
        charon.detectSamples();
        telemetry.addData("Gold Position: ", "" + charon.goldPosition);
        telemetry.addData("Gold Numeric Position:", charon.goldPos);
        telemetry.update();

        // Get off the lander :P
        charon.Drop();
        sleep(charon.wait);
        charon.collectHopperM.setPower(-0.5);
        charon.encoderDriveForward(5, 0, -1);
        charon.encoderDriveForward(1, 0, 1);
        charon.encoderDriveForward(12, -0.8, 0);

        charon.liftM.setPower(0);
        sleep(100);
        charon.encoderDriveRight(4, 0.8, 0);
        charon.collectHopperM.setPower(0);

        if (charon.side) {
            if(charon.follow){
                charon.avoidPartner();
            }
            else if (charon.two) {
                charon.toBlockTwo();
            } else {
                charon.toBlockPark();
            }
        }
        else {
            if (charon.follow) {
                charon.followPartner();
            } else {
                if (charon.marker) {
                    charon.toBlockCraterMarker();
                } else {
                    charon.toBlockCraterPark();
                }
            }
        }
    }
}