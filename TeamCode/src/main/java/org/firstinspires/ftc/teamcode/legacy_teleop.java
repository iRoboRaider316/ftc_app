package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DigitalChannel;
/**
 * Created by A horde of cheezy puffs TM on 6/14/17.
 */
@TeleOp (name="legacy_teleop",group="Opmode")
public class legacy_teleop extends OpMode {
    private DcMotor lfDriveM, rfDriveM, lbDriveM, rbDriveM, glyphLiftM, relicTurnM, relicExtendM;  //Left front drive, right front drive, left back drive, right back drive.
    private Servo ltGlyphS, rtGlyphS, lbGlyphS, rbGlyphS, jewelExtendS, jewelHitS, relicGrabberS, relicLowerS;
    private CRServo glyphSlideS, flipS;
    private DigitalChannel sensorDigitalMagnetic;
    private ElapsedTime runtime = new ElapsedTime();
    private boolean topajar = true;
    private boolean bottomajar = true;
    private boolean bothajar = false;
    private boolean flipUp = true;
    private boolean both = false;
    private int bothToggle = 0;
    private boolean top = false;
    private int topToggle = 0;
    private boolean bottom = false;
    private int bottomToggle = 0;
    private double relicGrab = 1;
    private double relicRelease = 0;
    private double relicDown = 0.9;
    private double relicRaise = 0.3;
    private double relicSlow1 = 0.2;
    private double relicSlow2 = 0.35;
    private double speedFactor = .7;
    private int controlMode = 1;
    private String relicToggle = "inOn";
    private String relicArmToggle = "inOn";
    private double pExtendArm = .3; //Partially extend the jewel arm.
    private double extendArm = .55; //Fully extend the jewel arm.
    private double retractArm = 0; //Bring jewel arm back to the robot.
    private double hitCenter = .5;   //jewelHitS returns to center variable
    private double hitLeft = 0;      //jewelHitS Hit left jewel variable
    private double hitPLeft = .25;   //Protects arm from falling in case of power outages.
    private double hitRight = 1;     //jewelHitS Hits right jewel variable
    boolean first = false;
    ElapsedTime flipper = new ElapsedTime();
    private enum SlippyState { Idle, LookingMagnet, FoundMagnet }
    private enum BothGlyphs { Idle, grab, open, ajar }
    private enum TopGlyphs { Idle, grab, open, ajar }
    private enum BottomGlyphs { Idle, grab, open, ajar }
    private enum FlippiBoi { Idle, Goingup, Goingdown }
    private SlippyState slippystate = SlippyState.Idle;
    private BothGlyphs bothglyphs = BothGlyphs.Idle;
    private TopGlyphs topglyphs = TopGlyphs.Idle;
    private BottomGlyphs bottomglyphs = BottomGlyphs.Idle;
    private FlippiBoi flippiboi = FlippiBoi.Idle;

    public void init () {
        lfDriveM = hardwareMap.dcMotor.get("lfDriveM");       //Left front drive, Hub 1, port 2
        lfDriveM.setPower(0);
        lbDriveM = hardwareMap.dcMotor.get("lbDriveM");       //Left back drive, Hub 1, port 3
        lbDriveM.setPower(0);
        rfDriveM = hardwareMap.dcMotor.get("rfDriveM");       //Right front drive, Hub 1, port 1
        rfDriveM.setPower(0);
        rbDriveM = hardwareMap.dcMotor.get("rbDriveM");       //Right back drive, Hub 1, port 0

        rbDriveM.setPower(0);
        glyphLiftM = hardwareMap.dcMotor.get("glyphLiftM");   //Lift motor, Hub 2, port 3
        glyphLiftM.setPower(0);
        ltGlyphS = hardwareMap.servo.get("LTGlyphS");     //Left servo arm, Hub 1, port 2
        rtGlyphS = hardwareMap.servo.get("RTGlyphS");     //Right servo arm, Hub 2, port 1
        lbGlyphS = hardwareMap.servo.get("LBGlyphS");     //Left servo arm, Hub 1, port 2
        rbGlyphS = hardwareMap.servo.get("RBGlyphS");     //Right servo arm, Hub 2, port 1
        glyphSlideS = hardwareMap.crservo.get("glyphSlideS");
        glyphSlideS.setPower(0);
        flipS = hardwareMap.crservo.get("flipS");
        flipS.setPower(0);
        jewelExtendS = hardwareMap.servo.get("jewelExtendS");   //Hub 3 Servo 0
        jewelExtendS.setPosition(retractArm);
        jewelHitS = hardwareMap.servo.get("jewelHitS");     //Hub 2 Servo 4
        relicTurnM = hardwareMap.dcMotor.get("relicTurnM");       //Relic Turn Motor, Hub 2, Port 0
        relicTurnM.setPower(0);
        relicExtendM = hardwareMap.dcMotor.get("relicExtendM");   //Relic Extend Motor, Hub 2, port 1 20
        relicExtendM.setPower(0);
        relicGrabberS = hardwareMap.servo.get("relicGrabberS");
        relicLowerS = hardwareMap.servo.get("relicLiftS");
        relicTurnM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        relicExtendM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lfDriveM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lbDriveM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rfDriveM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rbDriveM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        glyphLiftM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        sensorDigitalMagnetic = hardwareMap.get(DigitalChannel.class, "SensorDigitalMagnetic");
        sensorDigitalMagnetic.setMode(DigitalChannel.Mode.INPUT);
    }

    @Override
    public void start() {
        runtime.reset();

        ltGlyphS.setPosition(0.7);
        rtGlyphS.setPosition(0.3);
        lbGlyphS.setPosition(0.3);
        rbGlyphS.setPosition(0.7);

        jewelExtendS.setPosition(0);
        jewelHitS.setPosition(0.25);
    }

    public void loop() {
        // Set some variables for use in Glyph Grabbing
        boolean aUp = !gamepad2.a;
        boolean bUp = !gamepad2.b;
        boolean xUp = !gamepad2.x;
        boolean yUp = !gamepad2.y;

        if(gamepad1.right_bumper) {
            speedFactor = 1;
        } else {
            speedFactor = .65;
        }
        if(gamepad1.a) {            //Hitting the "a" button will allow the drive train to run at full capacity.
            controlMode = 1;
        }
        else if(gamepad1.y) {     //Hitting the "y" button will cut the drive train's power by 50%.
            controlMode = 2;
        }
///DRIVER CODE
        switch(controlMode) {
            case 1:
                lfDriveM.setPower((-gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y)) * speedFactor); //exponential scale algorithm
                lbDriveM.setPower((-gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y)) * speedFactor); //tank drive
                rfDriveM.setPower((gamepad1.right_stick_y * Math.abs(gamepad1.right_stick_y)) * speedFactor);
                rbDriveM.setPower((gamepad1.right_stick_y * Math.abs(gamepad1.right_stick_y)) * speedFactor);
                break;
            case 2:
                lfDriveM.setPower((-gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y)) * speedFactor); //exponential scale algorithm
                lbDriveM.setPower((-gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y)) * speedFactor); //tank drive
                rfDriveM.setPower((gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y)) * speedFactor);
                rbDriveM.setPower((gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y)) * speedFactor);
                break;
            default:
                lfDriveM.setPower((-gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y)) * speedFactor); //exponential scale algorithm
                lbDriveM.setPower((-gamepad1.left_stick_y * Math.abs(gamepad1.left_stick_y)) * speedFactor); //tank drive
                rfDriveM.setPower((gamepad1.right_stick_y * Math.abs(gamepad1.right_stick_y)) * speedFactor);
                rbDriveM.setPower((gamepad1.right_stick_y * Math.abs(gamepad1.right_stick_y)) * speedFactor);
                break;
        }
        if (gamepad1.dpad_up || gamepad1.dpad_down) {
            gamepad1.left_stick_y = gamepad1.right_stick_y;
        }
///OPERATOR CODE
        relicExtendM.setPower(-gamepad2.right_stick_y);
        relicTurnM.setPower(-gamepad2.left_stick_x/5);
        if (gamepad2.left_stick_x >= 0) {
            first = true;
        }

        if (gamepad2.dpad_right) {
            relicGrabberS.setPosition(relicSlow1);
        } else if (gamepad2.dpad_left) {
            relicGrabberS.setPosition(relicSlow2);
        }

        if (gamepad2.right_trigger > 0.01 && gamepad2.y){
            relicGrabberS.setPosition(gamepad2.right_trigger);
        }
        if(gamepad2.left_trigger > 0.01 && gamepad2.y){
            relicLowerS.setPosition(gamepad2.left_trigger);
        }
        if (gamepad2.right_bumper && relicToggle == "inOn") {
            relicToggle = "outOff";
        } else if (relicToggle == "outOff" && !gamepad2.right_bumper) {
            relicToggle = "outOn";
            relicGrabberS.setPosition(relicRelease);
        } else if (gamepad2.right_bumper && relicToggle == "outOn") {
            relicToggle = "inOff";
        } else if (relicToggle == "inOff" && !gamepad2.right_bumper) {
            relicToggle = "inOn";
            relicGrabberS.setPosition(relicGrab);
        }

        if (gamepad2.left_bumper && relicArmToggle == "inOn") {
            relicArmToggle = "outOff";
        } else if (relicArmToggle == "outOff" && !gamepad2.left_bumper) {
            relicArmToggle = "outOn";
            relicLowerS.setPosition(relicDown);
        } else if (gamepad2.left_bumper && relicArmToggle == "outOn") {
            relicArmToggle = "inOff";
        } else if (relicArmToggle == "inOff" && !gamepad2.left_bumper) {
            relicArmToggle = "inOn";
            relicLowerS.setPosition(relicRaise);
        }

        if (gamepad2.dpad_up) {
            if(relicLowerS.getPosition() == relicDown && relicDown > 0) {
                relicDown -= 0.01;
                relicLowerS.setPosition(relicDown);
                try { Thread.sleep(100); }
                catch (InterruptedException exc) { Thread.currentThread().interrupt(); }
            } else if(relicLowerS.getPosition() != relicDown) glyphLiftM.setPower(1);
        } else if (gamepad2.dpad_down) {
            if(relicLowerS.getPosition() == relicDown && relicDown < 1) {
                relicDown += 0.01;
                relicLowerS.setPosition(relicDown);
                try { Thread.sleep(100); }
                catch (InterruptedException exc) { Thread.currentThread().interrupt(); }
            } else if(relicLowerS.getPosition() != relicDown) glyphLiftM.setPower(-1);
        } else {
            glyphLiftM.setPower(0);
        }

        if (gamepad2.dpad_up) {
            glyphLiftM.setPower(1);
        } else if (gamepad2.dpad_down) {
            glyphLiftM.setPower(-1);
        } else {
            glyphLiftM.setPower(0);
        }

        // ----------- Glyph Grab System (Both sets) -----------
        if(!gamepad2.b && !bUp){
            both = true;
        }
        else if(gamepad2.x || gamepad2.y || gamepad2.a){
            both = false;
            bothToggle = 0;
        }
        if (!gamepad2.b && both && !bUp){
            if (bothToggle == 0){
                bothToggle = 1;
            } else if (bothToggle == 1 && !bothajar){
                bothToggle = 2;
                bothajar = true;
            } else if (bothToggle == 1 && bothajar){
                bothToggle = 0;
                bothajar = false;
            } else if (bothToggle == 2){
                bothToggle = 1;
            }
            telemetry.addData("Toggle State", bothToggle);
            telemetry.update();
            if (bothToggle > 2) bothToggle = 0;
            bothglyphs = bothToggle == 0 ? BothGlyphs.open :
                         bothToggle == 1 ? BothGlyphs.ajar : BothGlyphs.grab;

            // Now power the servos!
            switch (bothglyphs){
                case ajar:
                    ltGlyphS.setPosition(0.7);
                    rtGlyphS.setPosition(0.3);
                    lbGlyphS.setPosition(0.3);
                    rbGlyphS.setPosition(0.7);
                    break;
                case grab:
                    ltGlyphS.setPosition(0);
                    rtGlyphS.setPosition(1);
                    lbGlyphS.setPosition(1);
                    rbGlyphS.setPosition(0);
                    break;
                case open:
                    ltGlyphS.setPosition(1);
                    rtGlyphS.setPosition(0);
                    lbGlyphS.setPosition(0);
                    rbGlyphS.setPosition(1);
                    break;
            }
        }

        // ----------- Glyph Grab System (Top set only) -----------
        if((!gamepad2.y && !yUp)){
            top = true;
        } else if(gamepad2.x || gamepad2.b || gamepad2.a){
            top = false;
            topToggle = 0;
        }
        if (!gamepad2.y && !yUp && top){
            if (topToggle == 0){
                topToggle = 1;
            } else if (topToggle == 1 && !topajar){
                topToggle = 2;
                topajar = true;
            } else if (topToggle == 1 && topajar){
                topToggle = 0;
                topajar = false;
            } else if (topToggle == 2){
                topToggle = 1;
            }
            if (topToggle > 2) {
                topToggle = 0;
            }

            if (topToggle > 2) topToggle = 0;
            topglyphs = topToggle == 0 ? TopGlyphs.open :
                        topToggle == 1 ? TopGlyphs.ajar : TopGlyphs.grab;

            // Now power the servos!
            switch (topglyphs){
                case ajar:
                    ltGlyphS.setPosition(0.7);
                    rtGlyphS.setPosition(0.3);
                    break;
                case grab:
                    ltGlyphS.setPosition(1);
                    rtGlyphS.setPosition(0);
                    break;
                case open:
                    ltGlyphS.setPosition(0);
                    rtGlyphS.setPosition(1);
                    break;
            }
        }

        // ----------- Glyph Grab System (Bottom set only) -----------
        if(!gamepad2.a && !aUp){
            bottom = true;
            //bottomToggle = 0;
        } else if(gamepad2.x || gamepad2.b || gamepad2.y){
            bottom = false;
            bottomToggle = 0;
        }
        if (!gamepad2.a && !aUp && bottom){
            if (bottomToggle == 0){
                bottomToggle = 1;
            } else if (bottomToggle == 1 && !bottomajar){
                bottomToggle = 2;
                bottomajar = true;
            } else if (bottomToggle == 1 && bottomajar){
                bottomToggle = 0;
                bottomajar = false;
            } else if (bottomToggle == 2){
                bottomToggle = 1;
            }

            if (bottomToggle > 2) bothToggle = 0;
            bottomglyphs = bottomToggle == 0 ? BottomGlyphs.open :
                           bottomToggle == 1 ? BottomGlyphs.ajar : BottomGlyphs.grab;

            // Now power the servos!
            switch (bottomglyphs){
                case ajar:
                    lbGlyphS.setPosition(0.3);
                    rbGlyphS.setPosition(0.7);
                    break;
                case grab:
                    lbGlyphS.setPosition(1);
                    rbGlyphS.setPosition(0);
                    break;
                case open:
                    lbGlyphS.setPosition(0);
                    rbGlyphS.setPosition(1);
                    break;
            }

        }

        // ----------- Glyph Flipper System -----------
        if (!gamepad2.x && !xUp && flipUp){
            flipper.reset();
            flipUp = false;
            flippiboi = FlippiBoi.Goingup;
        } else if (!gamepad2.x && !xUp && !flipUp){
            flipper.reset();
            flipUp = true;
            flippiboi = FlippiBoi.Goingdown;
        } else if (flipper.milliseconds() > 1000){
            flippiboi = FlippiBoi.Idle;
        }
        switch (flippiboi){
            case Idle:
                flipS.setPower(0);
                break;
            case Goingdown:
                flipS.setDirection(DcMotorSimple.Direction.FORWARD);
                flipS.setPower(1);
                break;
            case Goingup:
                flipS.setDirection(DcMotorSimple.Direction.REVERSE);
                flipS.setPower(1);
                break;
        }

        // ----------- Glyph Slider System -----------
        switch (slippystate){
            case Idle:              //The Idle case is the default case
                if ((gamepad2.right_trigger < 0.1 && gamepad2.left_trigger > 0.1) || gamepad1.right_trigger < 0.1 && gamepad1.left_trigger > 0.1){
                    glyphSlideS.setDirection(DcMotorSimple.Direction.FORWARD);
                    glyphSlideS.setPower(1);
                    //If the right trigger is pressed, the timing belt moves to the right
                }
                else if ((gamepad2.right_trigger > 0.1 && gamepad2.left_trigger < 0.1) || gamepad1.right_trigger > 0.1 && gamepad1.left_trigger < 0.1) {
                    glyphSlideS.setDirection(DcMotorSimple.Direction.REVERSE);
                    glyphSlideS.setPower(1);
                    //If the left trigger is pressed, the timing belt moves to the left
                }
                else if ((gamepad2.right_trigger > 0.1 && gamepad2.left_trigger > 0.1) || gamepad1.y) {
                    if (glyphSlideS.getDirection() == DcMotorSimple.Direction.FORWARD) {
                        glyphSlideS.setDirection(DcMotorSimple.Direction.REVERSE);
                        //if both triggers (operator) or the y button (driver) is pressed, the timing
                        // belt moves the opposite direction of whatever direction it last moved
                    } else {
                        glyphSlideS.setDirection(DcMotorSimple.Direction.FORWARD);
                    }
                    slippystate = SlippyState.FoundMagnet;
                    //if both triggers (operator) or the y button (driver) is pressed, then switch to
                    //the Looking case
                }
                else {
                    glyphSlideS.setPower(0);
                }
                break;
            case LookingMagnet:    //The timing belt is moving until the magnetic sensor is triggered
                glyphSlideS.setPower(1);
                if (!sensorDigitalMagnetic.getState()) {
                    slippystate = SlippyState.FoundMagnet;
                    //If the sensor is triggered, switch to Found case
                }
                if ((gamepad2.right_trigger > 0.1) !=(gamepad2.left_trigger >0.1) || gamepad1.y) {
                    glyphSlideS.setPower(0);
                    slippystate = SlippyState.Idle;
                    //if either triggers (operator) or the y button (driver) is pressed, stop
                    //searching for the sensor and return to Idle case
                }
                break;
            case FoundMagnet:
                glyphSlideS.setPower(0);
                slippystate = SlippyState.Idle;
                //Once the magnetic sensor has been triggered, don't move the timing belt and return
                //to Idle case
                break;
        }
    }
}