package org.firstinspires.ftc.teamcode;

//import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static android.drm.DrmStore.Playback.STOP;
import static android.net.NetworkInfo.DetailedState.IDLE;


//import org.firstinspires.ftc.robotcore.external.Telemetry;
@Autonomous(name="snedHelpBOYS", group="Opmode")
@Disabled

public class CharonBase extends LinearOpMode {

    //================== ROBOT DEVICES ====================
    private GoldAlignDetector detector;     // Phone Camera
    private DcMotor lf, rf, lb, rb;         // Drive Train Motors
            DcMotor liftM;                  // Delivery/Latching Lift
    private DcMotor extendM;                // Collection Extension
    private DcMotor collectSpinnerM;        // Double Rubber Band Spinner (DRBS)
            DcMotor collectHopperM;         // DRBS Hopper
    private Servo dumpS;                    // Delivery Dump
    private CRServo lock, clampL, clampR;   // Dehanging Locks
    private BNO055IMU imu;                  // IMU Gyro itself
    private Orientation angles;             // IMU Gyro's Orienting
    private ColorSensor liftStager;         // Lift's Color Strip Reader

    //================= TIMERS =================
    private ElapsedTime timerDrive = new ElapsedTime();
    private ElapsedTime timerDehang = new ElapsedTime();
    private ElapsedTime repeaterLiftTuner = new ElapsedTime();
    private ElapsedTime repeaterLiftStages = new ElapsedTime();
    private ElapsedTime repeaterLiftIdler = new ElapsedTime();

    //================= VARIABLES ===============
            int wait = 0;
            int goldPos;
    private int count = 0;
    private int tQ = 1;
    private int tP = -1;

    private double angle;
    private double angleTest[] = new double[10];
    private double gyroCorrect;
    private double speed;
    private double rightX;
    private double sum;
    private double hopperPowerScale = 1;

            boolean side, marker, two, follow;
    private boolean goldAligned = false;
    private boolean goldFound = false;
    private boolean shouldHang;
    private boolean first = true;

    String goldPosition;

    // ================ CONSTANTS ============
    private int antiCatchMin = -250;
    private int antiCatchMax = -460;
    private int redMargin = 160;

    private enum LiftStage { IDLE, LIFTING, SLOWING, STOP }
    private LiftStage currentStage = LiftStage.IDLE;

    CharonBase(LinearOpMode opMode) {

        this.hardwareMap = opMode.hardwareMap;
        this.telemetry = opMode.telemetry;
        this.gamepad1 = opMode.gamepad1;
        this.gamepad2 = opMode.gamepad2;

        rb = hardwareMap.dcMotor.get("rb");
        rb.setPower(0);
        rb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lb = hardwareMap.dcMotor.get("lb");
        lb.setPower(0);
        lb.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rf = hardwareMap.dcMotor.get("rf");
        rf.setPower(0);
        rf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        lf = hardwareMap.dcMotor.get("lf");
        lf.setPower(0);
        lf.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        collectHopperM = hardwareMap.dcMotor.get("collectHopperM");
        collectHopperM.setPower(0);
        collectHopperM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        liftM = hardwareMap.dcMotor.get("liftM");
        liftM.setPower(0);
        liftM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        extendM = hardwareMap.dcMotor.get("extendM");
        extendM.setPower(0);
        extendM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        extendM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        extendM.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        collectSpinnerM = hardwareMap.dcMotor.get("collectSpinnerM");
        collectSpinnerM.setPower(0);
        collectSpinnerM.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        clampR = hardwareMap.crservo.get("clampR");
        clampL = hardwareMap.crservo.get("clampL");
        liftStager = hardwareMap.get(ColorSensor.class, "color");

        dumpS = hardwareMap.servo.get("dumpS");
        dumpS.setPosition(0);

        lock = hardwareMap.crservo.get("lock");

        BNO055IMU.Parameters parameters_IMU = new BNO055IMU.Parameters();
        parameters_IMU.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters_IMU.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters_IMU.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters_IMU.loggingEnabled = true;
        parameters_IMU.loggingTag = "IMU";
        parameters_IMU.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu = hardwareMap.get(BNO055IMU.class, "imu3");
        imu.initialize(parameters_IMU);
    }

    void initAuto(HardwareMap hardwareMap){
        // Set up detector
        detector = new GoldAlignDetector(); // Create detector
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance()); // Initialize it with the app context and camera
        detector.useDefaults(); // Set detector to use default settings

        // Optional tuning
        detector.alignSize = 150; // How wide (in pixels) is the range in which the gold object will be aligned. (Represented by green bars in the preview)
        detector.alignPosOffset = 250; // How far from center frame to offset this alignment zone.
        detector.downscale = 0.4; // How much to downscale the input frames

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA; // Can also be PERFECT_AREA
        //detector.perfectAreaScorer.perfectArea = 10000; // if using PERFECT_AREA scoring
        detector.maxAreaScorer.weight = 0.005;

        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0; // Ratio adjustment

        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        clampR.setPower(0);
        clampL.setPower(0);
        lock.setPower(0);
        sleep(50);

        goldAligned = detector.getAligned();
        goldFound = detector.isFound();
    }

    private void setDrive(double power) {
        setDriveSides(power, power);
    }

    private void setDriveSides(double lPower, double rPower) {
        setDriveMotors(lPower, rPower, lPower, rPower);
    }

    private void setDriveMotors(double lfPower, double rfPower, double lbPower, double rbPower) {
        lf.setPower(lfPower);
        rf.setPower(-rfPower);
        lb.setPower(lbPower);
        rb.setPower(-rbPower);
    }

    /* =================================================================
    *                   AUTONOMOUS EXCLUSIVE METHODS
    *  =================================================================*/

    // ================= PATHS ======================
     void toBlockTwo(){
        if (goldPos == 0){
            imuTurn(-30);
            encoderDriveForward(36, 0.5, 0.5);

            imuTurn(70);
            encoderDriveForward(20, -0.5, 0.5);
            imuTurn(-90);
            encoderDriveForward(30, -0.5, 0.5);
            encoderDriveRight(5, 0.5, -0.5);
            scoreMarker();

            encoderDriveForward(45, -0.5, -0.5);
            encoderDriveRight(16, 0.5, -0.5);
            encoderDriveForward(30, -0.5, -0.5);

        }

        else if (goldPos == 1){
            encoderDriveForward(54, 0, 0.5);

            imuTurn(-45);
            encoderDriveForward(30, -0.5, 0.5);
            encoderDriveRight(6, 0.5, -0.5);
            scoreMarker();

            encoderDriveForward(45, -0.5, -0.5);

            encoderDriveRight(16, 0.5, -0.5);

            imuTurn(40);

            encoderDriveRight(30, 0, -0.5);

            imuTurn(-55);

            encoderDriveForward(20, -0.5, -0.5);

        }

        else if(goldPos == 2){
            imuTurn(30);
            encoderDriveForward(47, -0.5, 0.5);

            imuTurn(-75);
            encoderDriveForward(10, -0.5, 0.5);
            encoderDriveRight(7, 0.5, -0.5);
            encoderDriveForward(20, 0.5, 0.5);
            scoreMarker();

            encoderDriveForward(45, -0.5, -0.5);
            encoderDriveRight(16, 0.5, -0.5);
            imuTurn(40);
            encoderDriveRight(44, 0, -0.5);
            imuTurn(-55);
            encoderDriveForward(20, -0.5, -0.5);
        }
        dumpS.setPosition(1);
        sleep(1000);
    }

     void toBlockPark(){
        if (goldPos == 0){
            imuTurn(-40);
            encoderDriveForward(38, 0.5, 0.5);

            imuTurn(80);
            encoderDriveForward(20, -0.5, 0.5);
            imuTurn(-90);
            encoderDriveForward(30, -0.5, 0.5);
            encoderDriveRight(5, 0.5, -0.5);
            scoreMarker();

            encoderDriveForward(70, -0.5, -0.5);
        }
        else if (goldPos == 1){
            imuTurn(-15);
            encoderDriveForward(54, 0, 0.5);

            imuTurn(-35);
            driveTime(5000, -0.5, 0.5);
            driveTime(1000, 0.5, -0.5);
            scoreMarker();

            encoderDriveForward(72, -0.5, -0.5);
        }

        else if(goldPos == 2){
            imuTurn(20);
            encoderDriveForward(47, -0.5, 0.5);

            imuTurn(-65);
            driveTime(3000, -0.5, 0.5);
            driveTime(7000, 0.5, -0.5);
            encoderDriveForward(20, 0.5, 0.5);
            scoreMarker();

            encoderDriveForward(70, -0.5, -0.5);
        }
        dumpS.setPosition(1);
        sleep(1000);
    }

     void toBlockCraterPark(){
        if (goldPos == 0){
            imuTurn(-50);
            encoderDriveForward(38, 0.5, 0.5);
        }

        else if (goldPos == 1){
            imuTurn(-10);
            encoderDriveForward(46, 0, 0.5);
        }

        else if(goldPos == 2){
            imuTurn(25);
            encoderDriveForward(50, -0.5, 0.5);
        }
        dumpS.setPosition(1);
        sleep(1000);
    }

     void toBlockCraterMarker(){
        encoderDriveForward(12, 0, 0.5);
        imuTurn(-90);
        encoderDriveForward(48, 1, 0);
        imuTurn(-45);
        driveTime(2000, 0.5, 0.5);
        driveTime(1000, -0.5, -0.5);
        sleep(500);
        encoderDriveForward(42, 0.8, -0.8);
        scoreMarker();

        encoderDriveForward(40, -0.8, 0.8);
        imuTurn(55);

        if      (goldPos == 0) encoderDriveForward(17, -0.5, 0);
        else if (goldPos == 1) encoderDriveForward(40, -0.5, 0);
        else if (goldPos == 2) encoderDriveForward(58, -0.5, 0);

        imuTurn(-85);
        encoderDriveForward(23, 0, 0.5);
        dumpS.setPosition(1);
        sleep(1000);
    }

    void followPartner(){
        if (goldPos == 0){
            imuTurn(-50);
            encoderDriveForward(28, 0.5, 0.5);
            sleep(100);
            encoderDriveRight(6, -0.5, -0.5);
            imuTurn(-45);
            encoderDriveForward(26, 0.5, 0);
        }

        else if (goldPos == 1){
            imuTurn(-10);
            encoderDriveForward(28, 0, 0.5);
            sleep(100);
            encoderDriveRight(11, 0, -0.5);
            imuTurn(-80);
            encoderDriveForward(38, 0.5, 0);
        }

        else if(goldPos == 2){
            imuTurn(20);
            encoderDriveForward(28, -0.5, 0.5);
            sleep(300);
            encoderDriveRight(11, 0.5, -0.5);
            imuTurn(-115);
            encoderDriveForward(50, 0.5, 0);
        }
        imuTurn(-90);
        encoderDriveRight(38, 0, -0.5);
        imuTurn(90);

        if (goldPos == 0){
            encoderDriveRight(6, -0.5, 0);
            imuTurn(-25);
            encoderDriveForward(44, 0.5, 0);
            imuTurn(70);
            encoderDriveForward(20, 0, 0.5);
            scoreMarker();
        }

        else if (goldPos == 1){
            encoderDriveForward(40, 0.5, 0);
            scoreMarker();
        }

        else if(goldPos == 2){
            encoderDriveRight(8, -0.5, 0);
            imuTurn(30);
            encoderDriveForward(40, 0.5, 0);
            imuTurn(-80);
            encoderDriveForward(20, 0.5, 0.5);
            encoderDriveRight(12, -0.5, -0.5);
            encoderDriveRight(26, 0.5, -0.5);
            scoreMarker();
        }
    }

    void avoidPartner(){
        if (goldPos == 0){
            imuTurn(-40);
            encoderDriveForward(53, 0, 0.5);

            imuTurn(80);
            encoderDriveRight(20, -0.5, 0.5);
            scoreMarker();

            dumpS.setPosition(1);
            encoderDriveRight(8, 0.5, 0.5);
            encoderDriveForward(75, 0.5, -0.5);
        } else if (goldPos == 1){
            imuTurn(-10);
            encoderDriveForward(56, 0, 0.5);

            imuTurn(50);
            encoderDriveRight(24, 0.5, 0.5);
            scoreMarker();

            dumpS.setPosition(1);
            encoderDriveRight(8, 0.5, 0.5);
            encoderDriveForward(80, 0.5, -0.5);
        } else if(goldPos == 2){
            imuTurn(20);
            encoderDriveForward(40, 0, 0.5);

            imuTurn(25);
            encoderDriveRight(66, 0.7, 0.7);
            encoderDriveForward(5, 0.5, -0.5);
            scoreMarker();

            dumpS.setPosition(1);
            encoderDriveRight(8, 0.5, 0.5);
            encoderDriveRight(85, 0.5, -0.5);
        }
        dumpS.setPosition(1);
        sleep(1000);
    }

    //never actually run this, just threw it in so that we can call this a linear opMode for telemetry data and controller inputs
    @Override
    public void runOpMode() {
    }

    // ============== AUTONOMOUS BASE METHODS ===============
    void Drop(){
        liftM.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftM.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lock.setPower(1);
        sleep(300);
        clampL.setPower(-1);
        sleep(500);
        clampL.setPower(0);
        lock.setPower(0);
        sleep(600);
        clampR.setPower(1);

        timerDehang.reset();
        while(timerDehang.milliseconds() < 2000 && liftStager.red() < redMargin && !isStopRequested()){
            liftM.setPower(-0.8);
        }

        liftM.setPower(-0.1);
        liftM.setPower(-0.25);
        sleep(100);
    }

    private void scoreMarker(){
        collectHopperM.setPower(0.4);
        sleep(400);
        collectHopperM.setPower(-0.4);
        collectSpinnerM.setPower(-0.5);
        sleep(800);
        collectHopperM.setPower(-0.8);
        sleep(800);
        collectSpinnerM.setPower(0);
        collectHopperM.setPower(0);
        dumpS.setPosition(1);
    }

    void encoderDriveForward(int distance, double x, double y){ //Distance in inches, x and y as direction values, both between 1- and 1
        //reset encoder values to 0
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //Run without encoders so we can get values properly
        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        //Run the loop as long as all of the wheels are at a position less than their target position
        //55 encoder ticks per inch, so convert the inch input to encoder ticks
        while(distance*55 > rf.getCurrentPosition() && -distance*55 < lf.getCurrentPosition() && distance*55 > rb.getCurrentPosition() && -distance*55 < lb.getCurrentPosition() &&!isStopRequested()) {
            //intake gyro values for drift correction
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);
            //calculate speed based off of the values given
            speed = Math.hypot(x, y);
            //calculate direction based off of the values given
            angle = Math.atan2(y, x) + Math.PI / 4;
            //use gyro correction
            angle = angle + angles.firstAngle;
            //Spinning value.  This code drives striaght so it is always zero
            rightX = 0;
            //set motor speeds
            setDriveMotors( (speed * (Math.sin(angle))  + rightX),
                           -(speed * (Math.cos(angle))) - rightX,
                            (speed * (Math.cos(angle))  + rightX),
                           -(speed * (Math.sin(angle))) - rightX);
            //telemetry data for troubleshooting
            telemetry.addData("current pos",rf.getCurrentPosition());
            telemetry.addData("current pos",rb.getCurrentPosition());
            telemetry.addData("current pos",lf.getCurrentPosition());
            telemetry.addData("current pos",lb.getCurrentPosition());
            telemetry.update();
        }
        //exit loop and stop motors
        setDrive(0);
    }

    void encoderDriveRight(int distance, double x, double y){
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while(-distance*55 < rf.getCurrentPosition() && distance*55 > lf.getCurrentPosition() && -distance*55 < rb.getCurrentPosition() && distance*55 > lb.getCurrentPosition()&&!isStopRequested()) {

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

            //0, -0.5 is to drive forward
            //-0.5, 0
            speed = Math.hypot(x, y);
            angle = Math.atan2(y, x) + Math.PI / 4;
            angle = angle + angles.firstAngle;
            rightX = 0;

            setDriveMotors( (speed * (Math.sin(angle))  + rightX),
                           -(speed * (Math.cos(angle))) - rightX,
                            (speed * (Math.cos(angle))  + rightX),
                           -(speed * (Math.sin(angle))) - rightX);

            telemetry.addData("current pos",rf.getCurrentPosition());
            telemetry.addData("current pos",rb.getCurrentPosition());
            telemetry.addData("current pos",lf.getCurrentPosition());
            telemetry.addData("current pos",lb.getCurrentPosition());
            telemetry.update();
        }
        setDrive(0);

    }

    private void driveTime(double millisecond, double x, double y){
        rf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lf.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lf.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while(millisecond > timerDrive.milliseconds() && !isStopRequested()) {

            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

            //0, -0.5 is to drive forward
            //-0.5, 0
            speed = Math.hypot(x, y);
            angle = Math.atan2(y, x) + Math.PI / 4;
            angle = angle + angles.firstAngle;
            rightX = 0;

            setDriveMotors( (speed * (Math.sin(angle))  + rightX),
                           -(speed * (Math.cos(angle))) - rightX,
                            (speed * (Math.cos(angle))  + rightX),
                           -(speed * (Math.sin(angle))) - rightX);

            telemetry.addData("current pos",rf.getCurrentPosition());
            telemetry.addData("current pos",rb.getCurrentPosition());
            telemetry.addData("current pos",lf.getCurrentPosition());
            telemetry.addData("current pos",lb.getCurrentPosition());
            telemetry.update();
        }
        setDrive(0);

    }

    private void imuTurn(double degreesToTurn) {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double currentHeading = -angles.firstAngle + 180;
        double targetHeading = degreesToTurn + currentHeading;

        targetHeading += targetHeading > 360 ? -360 :
                         targetHeading <   0 ?  360 : 0;

        while (Math.abs(degreesToTurn) > 2) {
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
            currentHeading = -angles.firstAngle + 180;
            degreesToTurn = targetHeading - currentHeading;

            double power = Range.clip(Math.signum(degreesToTurn) * (0.25 + (Math.abs(degreesToTurn) / 360)), -1, 1);
            setDriveSides(power, -power);
        }
        setDrive(0);
    }

    void selection(LinearOpMode opMode) {

        telemetry.addData("Press B ", "for depot side");
        telemetry.addData("Press X ", "for crater side");
        telemetry.update();
        while (!opMode.opModeIsActive() && !gamepad1.y) {


            if (gamepad1.b) {
                telemetry.addData("depot ", "side");
                telemetry.addData("press Y to ", "move on");
                telemetry.update();
                side = true;
            } else if (gamepad1.x) {
                telemetry.addData("crater ", "side");
                telemetry.addData("press Y to ", "move on");
                side = false;

                telemetry.update();
            }

        }
        sleep(1000);
        if (side) {
            telemetry.addData("Press B ", "for two block");
            telemetry.addData("Press X ", "for one");
            telemetry.update();
            while (!opMode.opModeIsActive() && !gamepad1.a) {
                if (gamepad1.b) {
                    telemetry.addData("Two ", "block");
                    telemetry.addData("press A to ", "move on");
                    two = true;
                    telemetry.update();
                } else if (gamepad1.x) {
                    telemetry.addData("One ", "block");
                    telemetry.addData("press A to ", "move on");
                    two = false;

                    telemetry.update();
                }
            }
        }
        else if(!side){
            telemetry.addData("Press B ", "for marker");
            telemetry.addData("Press X ", "for no marker");
            telemetry.update();
            while (!opMode.opModeIsActive() && !gamepad1.a) {
                if (gamepad1.b) {
                    telemetry.addData("Yes ", "marker");
                    telemetry.addData("press A to ", "move on");
                    marker = true;
                    telemetry.update();
                } else if (gamepad1.x) {
                    telemetry.addData("No ", "Marker");
                    telemetry.addData("press A to ", "move on");
                    marker = false;

                    telemetry.update();
                }
            }
        }

        telemetry.update();
        while(!opMode.opModeIsActive()){
            if(gamepad1.dpad_up){
                wait = wait + 1000;
                sleep(300);
            }

            else if (gamepad1.dpad_down){
                wait = wait - 1000;
                sleep(300);
                if(wait < 0){
                    wait = 0;
                }

            }

            telemetry.addData("Press ","Dpad up for more wait");
            telemetry.addData("Press ","Dpad down for not more wait");
            telemetry.addData("Press ","Play to start going!");
            telemetry.addData("Waiting for", wait);
            telemetry.update();
        }
    }

    void detectSamples() {
        detector.enable(); // Start the detector!


        sleep(2000);
        goldAligned = detector.getAligned();
        goldFound = detector.isFound();

        detector.disable();


        if (goldFound) { // gold is center or right
            if (goldAligned) { // gold is right
                goldPosition = "right";
                goldPos = 2;
            } else { // gold is center
                goldPosition = "center";
                goldPos = 1;
            }
        } else { // gold is left
            goldPosition = "left";
            goldPos = 0;
        }
    }

    /* =================================================================
     *                   TELEOP EXCLUSIVE METHODS
     *  =================================================================*/

    void updateExtendM() { extendM.setPower(gamepad2.right_stick_y); }

    void updateCollectSpinnerM() {
        collectSpinnerM.setPower(tQ*gamepad2.right_trigger + tP*gamepad2.left_trigger);
    }

    void updateCollectHopperM() {
        if(extendM.getCurrentPosition() < antiCatchMin &&
           extendM.getCurrentPosition() > antiCatchMax)   hopperPowerScale = 0.3;
        else                                              hopperPowerScale = 1;

        // Only give power scaling when powered upward so we can still lower to collect efficiently.
        // We need to check for experimental power swap so we can effectively power scale in the
        // right direction.
        if(tQ == 1) {
            if     (gamepad2.dpad_down)   collectHopperM.setPower(tQ*0.8*hopperPowerScale);
            else if(gamepad2.dpad_up)     collectHopperM.setPower(tP*0.8);
            else                          collectHopperM.setPower(0);
        }
        else {
            if     (gamepad2.dpad_down)   collectHopperM.setPower(tQ*0.8);
            else if(gamepad2.dpad_up)     collectHopperM.setPower(tP*0.8*hopperPowerScale);
            else                          collectHopperM.setPower(0);
        }
    }

    void updateDumpS() {
        if(gamepad2.right_bumper || gamepad1.right_bumper) dumpS.setPosition(1);
        else                                               dumpS.setPosition(0);
    }

    void updateLiftM() {
        if(gamepad2.left_bumper){
            liftM.setPower(-0.2);
        }

        if (gamepad2.x && repeaterLiftStages.milliseconds() > 300){
            shouldHang = true;
            repeaterLiftStages.reset();
        }
        else if (gamepad2.b && repeaterLiftStages.milliseconds() > 300){
            shouldHang = false;
            repeaterLiftStages.reset();
        }

        if(shouldHang) {
            switch(currentStage) {
                case IDLE:
                    currentStage = LiftStage.LIFTING;
                    break;
                case LIFTING:
                    if(liftStager.red() < redMargin) liftM.setPower(-0.9);
                    else currentStage = LiftStage.SLOWING;
                    break;
                case SLOWING:
                    if(liftStager.red() > redMargin)               liftM.setPower(0);
                    else if(liftStager.red() > redMargin && first) liftM.setPower(0);
                    else currentStage = LiftStage.STOP;
                    break;
                case STOP:
                    if(repeaterLiftTuner.milliseconds() > 200 && gamepad2.a){
                        liftM.setPower(0);
                        repeaterLiftTuner.reset();
                    }
                    else if (repeaterLiftTuner.milliseconds() > 200 && gamepad2.y){
                        liftM.setPower(-0.6);
                        repeaterLiftTuner.reset();
                    }
                    else {
                        liftM.setPower(0.25);
                    }
                    if(first){
                        first = false;
                    }

                    break;
            }
        } else {
            currentStage = LiftStage.IDLE;
            if(gamepad2.left_stick_y > 0.1 || gamepad2.left_stick_y < -0.1) {
                liftM.setPower(gamepad2.left_stick_y);
                repeaterLiftIdler.reset();
            }
            else if (gamepad2.left_stick_y == 0 && repeaterLiftIdler.milliseconds() < 500){
                liftM.setPower(0.4);
            }
            else {
                liftM.setPower(gamepad2.left_stick_y);
            }
        }
    }

    void updateLocks() {
        if(gamepad1.right_bumper){
            lock.setPower(1);
            clampL.setPower(-1);
        }
        if(gamepad1.left_bumper){
            clampR.setPower(0);
            clampL.setPower(0);
            lock.setPower(0);
        }
        if(gamepad1.x){
            clampR.setPower(1);
            clampL.setPower(1);
            lock.setPower(1);
        }
    }

    void updateDriveTrain() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS);

        speed = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        angle = Math.atan2(gamepad1.left_stick_y, gamepad1.left_stick_x) + (Math.PI/4);
        rightX = gamepad1.right_stick_x;

        if(rightX == 0){
            if (count < 10) {
                angleTest[count] = angle;
                count++;
            }
            else if (count >= 10){
                sum = (angleTest[1] + angleTest[2] + angleTest[3] + angleTest[4] + angleTest[0] + angleTest[5] + angleTest[6] + angleTest[7] + angleTest[8] + angleTest[9])/10;
                if(sum > angle){
                    gyroCorrect = sum - angle;
                    angle = angle + gyroCorrect;
                }
                else if(angle > sum){
                    gyroCorrect = angle - sum;
                    angle = angle - gyroCorrect;
                }
                count = 0;

            }
        }

        lf.setPower(((speed * (Math.sin(angle)) + rightX)));
        rf.setPower((-(speed * (Math.cos(angle))) + rightX));
        lb.setPower(((speed * (Math.cos(angle)) + rightX)));
        rb.setPower((-(speed * (Math.sin(angle))) + rightX));
        telemetry.addData("Lift Staging", repeaterLiftStages.milliseconds());
        telemetry.addData("gp1 Bumper", gamepad1.right_bumper);
        telemetry.addData("gp2 Bumper", gamepad2.right_bumper);
        telemetry.addData("Extend Pos", extendM.getCurrentPosition());
        telemetry.addData("Hopper Scale", hopperPowerScale);
        telemetry.addData("Lift Idler", repeaterLiftIdler);
        telemetry.update();
    }
}