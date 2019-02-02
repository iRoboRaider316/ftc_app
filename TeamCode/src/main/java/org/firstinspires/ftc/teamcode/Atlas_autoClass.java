package org.firstinspires.ftc.teamcode;
import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@Autonomous(name="Cheerios Auto (Real Deal)", group="OpMode")

public class Atlas_autoClass extends LinearOpMode {


    public void runOpMode() throws InterruptedException {

        AtlasClass base = new AtlasClass(hardwareMap, telemetry, gamepad1, gamepad2);
        base.initAuto(hardwareMap);

        base.selection(this);
        waitForStart();

        base.detector.enable(); // Start the detector!
        sleep(2000);
        base.goldAligned = base.detector.getAligned();
        base.goldFound = base.detector.isFound();

//
//        //used to be 180
//        //If the mineral is inside of a 153 pixel range on the left side of the screen
//        if (detector.getYPosition() > 247 && detector.getYPosition() < 400) {
//        pos = 1; //Position 1 is center
//        telemetry.addData("Y Pos" , detector.getYPosition()); //Print Gold Y position.
//        telemetry.update();
//
//    }//If the mineral is inside of a 193 pixel range on the right side on the screen
//    else if (detector.getYPosition() < 247  && detector.getYPosition() > 50) {
//        pos = 2;
//        telemetry.addData("Y Pos" , detector.getYPosition()); //Print Gold Y position.
//        telemetry.update();
//    }//If the mineral is off the screen
//    else if (detector.getYPosition() == 0) {
//        pos = 0; //Position 0 is left, where we cannot see the mineral
//        telemetry.addData("Y Pos" , detector.getYPosition()); //Print Gold Y position.
//        telemetry.update();
//
//    }
//    else pos = 1; //If something messes up, default to center position



        base.detector.disable();


        if (base.goldFound) { // gold is center or right
            if (base.goldAligned) { // gold is right
                base.goldPosition = "right";
                base.pos = 2;
            }
            else { // gold is center
                base.goldPosition = "center";
                base.pos = 1;
            }
        }
        else { // gold is left
            base.goldPosition = "left";
            base.pos = 0;
        }
        telemetry.addData("Gold Position: ", "" + base.goldPosition);
        //telemetry.update();


        telemetry.addData("pos:", base.pos);
        telemetry.update();

//pos = 0;


//
//        encoderDriveForward(2, 0, 0.5);
//
//
//        encoderDriveRight(13, 0.5, 0);
//
//        liftM.setPower(0);
//
        sleep(base.wait);

        if(base.side) {
            if(base.two) {
                base.toBlockTwo();
            }
            else if (!base.two){
                base.toBlockPark();
            }
        }

        else if(!base.side) {
            if(base.marker) {
                base.toBlockCraterMarker();
            }
            else if(!base.marker){
                base.toBlockCraterPark();
            }
        }
    }
}