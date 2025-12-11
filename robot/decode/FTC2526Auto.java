package org.firstinspires.ftc.teamcode.robot.decode;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.draw;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.drawOnlyCurrent;
import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import androidx.annotation.NonNull;

import com.bylazar.configurables.PanelsConfigurables;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.*;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebcam;
import org.firstinspires.ftc.teamcode.mechanisms.Mecanum;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.Tuning;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Autonomous(name = "FTC 2025-2026 DECODE Auto", group="Autos")
public class FTC2526Auto extends OpMode {

    AprilTagWebcam apriltag = new AprilTagWebcam();

    ElapsedTime runtime = new ElapsedTime();

    Mecanum drive;

    DcMotorEx Shooter; // ARTIFACT Shooter
    DcMotorEx Csl; // ARTIFACT Carousel
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

    IMU imu;

    enum State {
        drive,
        rotate,
        shoot,
        spinCSL,
        spinCSLB,
        rev,
        none
    }

    enum Motif {
        GPP,
        PGP,
        PPG
    }

    double ShooterPwr;

    Telemetry.Item cmotif, cstate;
    PathChain forward, leave;
    int path;
    Timer pathT, actT, opT;

    AprilTagDetection red = apriltag.getTagbyID(24);
    AprilTagDetection blue = apriltag.getTagbyID(20);

    AprilTagDetection mGPP = apriltag.getTagbyID(21);
    AprilTagDetection mPGP = apriltag.getTagbyID(22);
    AprilTagDetection mPPG = apriltag.getTagbyID(23);

    boolean selMotif;
    double factor = .015;

    int cslpos = 96;
    int weirdpidthingbecauseimlazy = 35;

    boolean[] dbs = {
            false,
            false
    };

    boolean doneSpinning= false;

    Follower follower;

    public ArtifactColors getArtifactColor(@NonNull NormalizedRGBA color) {
        float normR, normG, normB;
        normR=color.red / color.alpha;
        normG=color.green / color.alpha;
        normB=color.blue / color.alpha;

        if (normR > .097 && normG > 0.097 && normB > 0.137) {
            return ArtifactColors.PURPLE;
        } else if (normR > 0.065 && normG > 0.160 && normB > 0.115) {
            return ArtifactColors.GREEN;
        }

        return ArtifactColors.UNK;
    }

    State state;
    Motif motif;

    Telemetry.Item goalA;

    enum Goals {
        RED,
        BLUE
    }

    Goals goal;

    @Override
    public void init() {
        pathT = new Timer();
        opT = new Timer();
        actT = new Timer();
        opT.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        PanelsConfigurables.INSTANCE.refreshClass(this);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));


        //Add Telemetry Values
        telemetry.setAutoClear(false);

        ShooterS = hardwareMap.get(NormalizedColorSensor.class, "ShooterS");
        BackS = hardwareMap.get(NormalizedColorSensor.class, "BackS");
        FrontS = hardwareMap.get(NormalizedColorSensor.class, "FrontS");

        ShooterS.setGain(15);
        FrontS.setGain(15);
        BackS.setGain(15);

        SS = ShooterS.getNormalizedColors();
        BS = BackS.getNormalizedColors();
        FS = FrontS.getNormalizedColors();

        Shooter = hardwareMap.get(DcMotorEx.class, "Shooter");
        Csl = hardwareMap.get(DcMotorEx.class, "Csl");
        Intake = hardwareMap.get(DcMotor.class, "Intake");
        Push = hardwareMap.get(Servo.class, "Push");

        Shooter.setDirection(DcMotorSimple.Direction.REVERSE);

        drive = new Mecanum();

        Csl.setTargetPosition(0);
        Csl.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(weirdpidthingbecauseimlazy,0,.175,0, MotorControlAlgorithm.PIDF));
        Csl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Csl.setPositionPIDFCoefficients(weirdpidthingbecauseimlazy);

        Csl.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        apriltag.init(hardwareMap, telemetry, "Shooter Cam");
        state = State.none;
        goal = Goals.RED;

        goalA = telemetry.addData("What Alliance are you on? (X to Swap): ", goal);

        cmotif = telemetry.addData("current motif: ", motif);
        cstate = telemetry.addData("current state: ", state);

        telemetry.update();
        apriltag.update();
    }

    public void init_loop() {

        follower.update();
//        Tuning.drawOnlyCurrent();

        if (apriltag.isMetadata(mGPP) && !selMotif) {
            motif = Motif.GPP;
            selMotif=true;
        } else if (apriltag.isMetadata(mPGP) && !selMotif) {
            motif = Motif.PGP;
            selMotif=true;
        } else if (apriltag.isMetadata(mPPG) && !selMotif) {
            motif = Motif.PPG;
            selMotif=true;
        } else if (!selMotif){
            motif = Motif.GPP;
        }

        if (gamepad1.x) {
            switch (goal) {
                case RED:
                    buildPaths(goal);
                    break;
                case BLUE:
                    buildPaths(goal);
                    break;
                default:
                    buildPaths(Goals.RED);
                    break;
            }
        }

        goalA.setValue(goal);

        telemetry.update();
        apriltag.update();
    }

    public void start() {
        runtime.reset();
        opT.resetTimer();
        setPath(0);
    }

    @Override
    public void loop() {

        if (apriltag.isMetadata(mGPP) && !selMotif) {
            motif = Motif.GPP;
            selMotif=true;
        } else if (apriltag.isMetadata(mPGP) && !selMotif) {
            motif = Motif.PGP;
            selMotif=true;
        } else if (apriltag.isMetadata(mPPG) && !selMotif) {
            motif = Motif.PPG;
            selMotif=true;
        } else if (!selMotif){
            motif = Motif.GPP;
        }

        boolean hasGreen = false;

        cmotif.setValue(motif);

        follower.update();
//        draw();

        if (apriltag.getTagRange(red) > apriltag.getTagRange(blue)) {
            ShooterPwr = factor * apriltag.getTagRange(red);
        } else if (apriltag.getTagRange(red) < apriltag.getTagRange(blue)){
            ShooterPwr = factor * apriltag.getTagRange(blue);
        } else {
            ShooterPwr = 0;
        }

        Shooter.setPower(ShooterPwr);

        switch (motif) {
            case GPP:
                if (getArtifactColor(ShooterS.getNormalizedColors()) == ArtifactColors.GREEN && !hasGreen) {
                    hasGreen = true;

                    if ((!follower.isBusy()) && (state != State.shoot)) {
                        actT.resetTimer();
                        if (actT.getElapsedTimeSeconds() == .5) {
                            state = State.shoot;
                        }
                        if (doneSpinning) {
                            doneSpinning = false;

                            state = State.spinCSL;

                            actT.resetTimer();
                            if (actT.getElapsedTimeSeconds() == .5 && !Csl.isBusy()) {
                                state = State.shoot;
                            }
                            if (doneSpinning) {
                                doneSpinning = false;

                                state = State.spinCSL;
                            }
                        }
                    }
                } else if (getArtifactColor(BackS.getNormalizedColors()) == ArtifactColors.GREEN && !hasGreen) {
                    hasGreen = true;
                    state = State.spinCSL;
                    if (doneSpinning) {
                        state = State.shoot;
                    }
                }else if (getArtifactColor(FrontS.getNormalizedColors()) == ArtifactColors.GREEN && !hasGreen) {
                    hasGreen = true;

                    state = State.spinCSLB;
                    if (doneSpinning) {
                        state = State.shoot;
                    }
                } else {
                    state = State.spinCSL;
                    if (doneSpinning) {
                        state = State.shoot;
                    }
                }

                break;
            case PGP:

                break;
            case PPG:

                break;
            default:
                motif = Motif.PGP;
                break;
        }

        cstate.setValue(state);
        switch (state) {
            case shoot:
                if (!dbs[0]) {
                    dbs[0] = true;
                    Shooter.setVelocity(ShooterPwr);

                    actT.resetTimer();

                    if (actT.getElapsedTimeSeconds() < 5) {
                        Push.setPosition(3);

                        actT.resetTimer();

                        if (actT.getElapsedTimeSeconds() < 1.5) {
                            Push.setPosition(0);

                            runtime.reset();
                        }

                    }

                    Shooter.setVelocity(0);
                    dbs[0]=false;
                    doneSpinning = true;
                }
                break;
            case spinCSLB:
                Push.setPosition(0);
                if (Csl.getCurrentPosition() == 0 || Csl.getCurrentPosition() <= 284) {
                    Csl.setTargetPosition(192);

                    if (Csl.getCurrentPosition() < 94) {
                        doneSpinning = true;
                    }
                } else if (Csl.getCurrentPosition() <= 95 & !(Csl.getCurrentPosition() < 190)) {
                    Csl.setTargetPosition(0);

                    if (Csl.getCurrentPosition() < 190) {
                        doneSpinning = true;
                    }
                } else if (Csl.getCurrentPosition() <= 190 & !(Csl.getCurrentPosition() < 284)) {
                    Csl.setTargetPosition(96);

                    if (Csl.getCurrentPosition() < -2) {
                        doneSpinning = true;
                    }
                }
                break;
            case spinCSL:
                if (Csl.getCurrentPosition() == 0 || Csl.getCurrentPosition() <= 284) {
                    Csl.setTargetPosition(96);

                    if (Csl.getCurrentPosition() < 94) {
                        doneSpinning = true;
                    }
                } else if (Csl.getCurrentPosition() <= 95 & !(Csl.getCurrentPosition() < 190)) {
                    Csl.setTargetPosition(192);

                    if (Csl.getCurrentPosition() < 190) {
                        doneSpinning = true;
                    }
                } else if (Csl.getCurrentPosition() <= 190 & !(Csl.getCurrentPosition() < 284)) {
                  Csl.setTargetPosition(0);

                  if (Csl.getCurrentPosition() < -2) {
                      doneSpinning = true;
                  }
                }
                break;
        }

        telemetry.update();

        apriltag.update();
    }

    public void buildPaths(Goals g) {
        follower.activateAllPIDFs();

        if (g == Goals.RED) {
            forward = follower.pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(88, 8), new Pose(60, 14))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(50))
                    .build()
            ;

            leave = follower.pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(60, 14), new Pose(72, 36))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .build()
            ;
        } else {
            forward = follower.pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(55, 8), new Pose(80, 14))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(120))
                    .build()
            ;

            leave = follower.pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(80, 14), new Pose(72, 36))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .build()
            ;
        }
    }
    public void autonomousPathUpdate() {
        switch (path) {
            case 0:
                if (!follower.isBusy()) {
                    follower.followPath(forward);
//                    setPath(-1);
                }
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(leave);
//                    setPath(-1);
                }
                break;
        }
    }

    public void setPath(int state) {
        path = state;
        pathT.resetTimer();
    }
}
