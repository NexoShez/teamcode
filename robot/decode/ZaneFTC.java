package org.firstinspires.ftc.teamcode.robot.decode;

//import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
//import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.mechanisms.Mecanum;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@TeleOp(name="FTC: Zane' Data Collection", group="TeleOps")
@SuppressWarnings("unused")
public class ZaneFTC extends OpMode {

    // Mecanum
    Mecanum drive;

    DcMotorEx Shooter; // ARTIFACT Shooter
    DcMotor Csl; // ARTIFACT Carousel
    DcMotor Intake; // ARTIFACT Intake

    // Servos
    Servo Push; // Servo push ball into shooter

    // Sensors
    NormalizedColorSensor ShooterS;
    NormalizedColorSensor BackS;
    NormalizedColorSensor FrontS;

    NormalizedRGBA SS, BS, FS;
    enum ArtifactColors {
        PURPLE,
        GREEN,
        UNK
    }

    enum Goals {
        RED,
        BLUE,
        UNI
    }

    Goals goal;

    AprilTagWebcam apriltags;
    IMU imu;

    // Misc Variables
    /**
     * First - Shooter<p>
     * Second -  Back<p>
     * Third - Front<p>
     * <p> - <p>
     0 - none<p>
     1 - GRN<p>
     2 - PRPL<p>
     */
    static int[] arts = {0,0,0};
    String obsk = "";
    double maxShootDist = 220;
    double factor = .025;
    double  dShooterPwr;
    boolean dIsShooter = false;
    boolean singleOp = true;
    int shoTime; //timer to keep the shooter runing if the camera repeadedly loses track of the april tag
    double endGame;
    boolean isEndGame;
    boolean db1 = false, db2 = false, cs = false, shu = false, shoo = false;
    double dSPDefault = 0.0;
    int cAngle = 0;
    Telemetry.Item cam;
    Telemetry.Item cslEncoder;
    Telemetry.Item shtrPwr;
    Telemetry.Item shtrStatus;
    Telemetry.Item tagstuff, tagMode;
    Telemetry.Item shU,shO;

    @Override
    public void init() {
        //Add Telemetry Values
        telemetry.setAutoClear(true);

        ShooterS = hardwareMap.get(NormalizedColorSensor.class, "ShooterS");
        BackS = hardwareMap.get(NormalizedColorSensor.class, "BackS");
        FrontS = hardwareMap.get(NormalizedColorSensor.class, "FrontS");

        ShooterS.setGain(20);
        FrontS.setGain(20);
        BackS.setGain(20);

        SS = ShooterS.getNormalizedColors();
        BS = BackS.getNormalizedColors();
        FS = FrontS.getNormalizedColors();

        Shooter = hardwareMap.get(DcMotorEx.class, "Shooter");
        Csl = hardwareMap.get(DcMotor.class, "Csl");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Push = hardwareMap.get(Servo.class, "Push");

        Shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        drive = new Mecanum();

        Csl.setTargetPosition(0);

        Csl.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        apriltags = new AprilTagWebcam();
        apriltags.init(hardwareMap, telemetry, "Shooter Cam");

        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);
        drive.init(hardwareMap, telemetry, imu);
    }

    public void start() {
        endGame = 90;
    }

    public void init_loop() {

        telemetry.addData("Panels IP:", "192.168.43.1:8001");
        if (singleOp) {
            telemetry.addLine("\n\nHello IDEA Teams! This is currently in a single operator mode, meaning only 1 controller will control the robot (Controller #1)\n\nControls:\nL Stick = Drive\nR Stick (X) = Rotate"+
                    "\nShoot ARTIFACT = Hold Right Bumper\nPush Purple ARTIFACT Into Shooter = X Button\nPush Green ARTIFACT Into Shooter = Y Button"+
                    "Rotate Carousel Clockwise = DPAD Up\nRotate Carousel Counter-Clockwise = DPAD Down");
        } else {
            telemetry.addLine("\n\nHello IDEA Teams! This is currently in a double operator mode (Driver-Manipulator)\n\nDriver Controls:\nL Stick = Drive\nR Stick (X) = Rotate" +
                    "\nShoot ARTIFACT = Hold Right Bumper\n\nManipulator Controls:"+
                    "\nPush Purple ARTIFACT Into Shooter = X Button\nPush Green ARTIFACT Into Shooter = Y Button" +
                    "Rotate Carousel Clockwise = DPAD Up\nRotate Carousel Counter-Clockwise = DPAD Down\n\n");
        }

        goal = Goals.UNI;
        tagMode = telemetry.addData("How does the shooter act? (Press X to Switch): ", goal);

        if (gamepad1.x) {
            switch (goal) {
                case RED:
                    goal = Goals.BLUE;
                    break;
                case UNI:
                    goal = Goals.RED;
                    break;
                default:
                    goal = Goals.UNI;
                    break;
            }
        }

        cslEncoder = telemetry.addData("Carousel Position: ", Csl.getCurrentPosition());
        cam = telemetry.addData("dist to tag",dShooterPwr);
        shtrPwr = telemetry.addData("Shooter Power (0-10, Default 7): ", dShooterPwr*10);
//        shtrRPM = telemetry.addData("Shooter RPM", ((60)/(Shooter.getVelocity()));
        shtrStatus = telemetry.addData("Is Shooter Spinning?: ", dIsShooter);
        tagstuff = telemetry.addData("Shooter Factor: ", factor);
        shU = telemetry.addData("Shooter on by driver?: ", shu);
        shO = telemetry.addData("Shooter ready?: ", shoo);

        shU.setValue(shu);
        shO.setValue(shoo);
        cslEncoder.setValue(Csl.getCurrentPosition());
        telemetry.update();
        apriltags.update();
    }

    public void loop() {
        telemetry.addData("Panels IP:", "192.168.43.1:8001");
        telemetry.addData("Timer:", getRuntime());
        telemetry.addData("power",dSPDefault);
        cslEncoder = telemetry.addData("Carousel Position: ", Csl.getCurrentPosition());
        shtrPwr = telemetry.addData("Shooter Power (0-10, Default 7): ", dShooterPwr * 10);
//        shtrRPM = telemetry.addData("Shooter RPM", ((60)/(Shooter.getVelocity()));
        shtrStatus = telemetry.addData("Is Shooter Spinning?: ", dIsShooter);
        tagstuff = telemetry.addData("Shooter Factor: ", factor);
        shU = telemetry.addData("Shooter on by driver?: ", shu);
        shO = telemetry.addData("Shooter ready?: ", shoo);

        final double y = -gamepad1.left_stick_y;
        final double x = gamepad1.left_stick_x;
        final double rx = gamepad1.right_stick_x;
        final boolean fixed = gamepad1.left_bumper;
        final boolean intake = gamepad1.right_bumper;
        final double dSPDown = gamepad1.left_trigger;
        final double dSPUp = gamepad1.right_trigger;

        boolean mIntake = gamepad2.left_bumper;
        boolean mPrpl = gamepad2.x;
        boolean mGrn = gamepad2.y;
        boolean mIndex = gamepad2.right_bumper;
        boolean mCsl = gamepad2.dpad_up;
        boolean mCslB = gamepad2.dpad_down;

        AprilTagDetection redG = apriltags.getTagbyID(24);
        AprilTagDetection blueG = apriltags.getTagbyID(20);

        if (singleOp) {
            mIntake = gamepad1.left_bumper;
            mPrpl = gamepad1.x;
            mGrn = gamepad1.y;
            mIndex = gamepad1.right_stick_button;
            mCsl = gamepad1.dpad_up;
            mCslB = gamepad1.dpad_down;
        }

        if (gamepad1.start) {
            imu.resetYaw();
            Csl.setTargetPosition(0);
            shoo = true;
        }

        if (mPrpl || mGrn) {
            Push.setPosition(3);
        } else {
            Push.setPosition(0);
        }

        if (endGame <= getRuntime() && !isEndGame) {
            gamepad1.rumble(1, 0, 3);
            gamepad2.rumbleBlips(3);
            isEndGame = true;
        }

        if (mCsl && !cs) {
            shoo = false;
            shu = false;
            cs = true;
            Csl.setPower(.25);
            cAngle+=(360/3);
            Csl.setTargetPosition(cAngle);
            if (Math.round(Csl.getCurrentPosition())== cAngle) {
                Csl.setPower(0);
            }
        }

        if (!Csl.isBusy()) {
            cs = false;
            shoo = true;
        }
//        if (mCslB && !cs) {
//            int pos;
//            shoo = false;
//            shu = false;
//            cs = true;
//            Csl.setPower(.25);
//            pos = Csl.getCurrentPosition() - 94;
//            Csl.setTargetPosition(Csl.getCurrentPosition() - 94);
//            if (Csl.getCurrentPosition() == pos) {
//                Csl.setPower(0);
//            }
//        }

        cslEncoder.setValue(Csl.getCurrentPosition());

        shu = intake && shoo;

        shtrPwr.setValue(dShooterPwr);

        if ((((DistanceSensor) BackS).getDistance(DistanceUnit.CM) <= 2.5) && (((DistanceSensor) FrontS).getDistance(DistanceUnit.CM) <= 3.5) && (((DistanceSensor) FrontS).getDistance(DistanceUnit.CM) >= 3)) {
            shoo = false;
            shu = false;
            cs = true;
            Csl.setTargetPosition(Csl.getCurrentPosition()-94);
            if (Csl.isBusy()) {
                cs = false;
                shoo = true;
            }
        }

        if (apriltags.getTagRange(redG) > apriltags.getTagRange(blueG) && (shu && shoo) && !(goal == Goals.BLUE)) {
            if (apriltags.getTagRange(redG) >= 10) {
                dShooterPwr = factor * apriltags.getTagRange(redG);
            }
        } else if (apriltags.getTagRange(redG) < apriltags.getTagRange(blueG) && (shu && shoo) && !(goal == Goals.RED)){
            if (apriltags.getTagRange(blueG) >= 10) {
                dShooterPwr = factor * apriltags.getTagRange(blueG);
            }
        }

        shU.setValue(shu);
        shO.setValue(shoo);

        Shooter.setPower(dShooterPwr);
        telemetry.update();
        apriltags.update();


        drive.drive(y,x,rx,2);
    }

    public ArtifactColors getArtifactColor(NormalizedRGBA color) {
        float normR, normG, normB;
        normR=color.red / color.alpha;
        normG=color.green / color.alpha;
        normB=color.blue / color.alpha;

        if (normR > 0 && normG > 0 && normB > 0) {
            return ArtifactColors.PURPLE;
        } else if (normR > 1 && normG > 0 && normB > 0) {
            return ArtifactColors.GREEN;
        }

        return ArtifactColors.UNK;
    }
}
