package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(40)
            .forwardZeroPowerAcceleration(-5041.814754124957)
            .lateralZeroPowerAcceleration(-4630.832527725512)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0))
            .headingPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.01, 0))
            ;
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("Fr")
            .rightRearMotorName("Br")
            .leftRearMotorName("Bl")
            .leftFrontMotorName("Fl")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(428.5879842011498)
            .yVelocity(270.92406925493003)
            ;

    public static DriveEncoderConstants localizer = new DriveEncoderConstants()
            .rightFrontMotorName("Fr")
            .rightRearMotorName("Br")
            .leftRearMotorName("Bl")
            .leftFrontMotorName("Fl")
            .leftFrontEncoderDirection(Encoder.FORWARD)
            .leftRearEncoderDirection(Encoder.FORWARD)
            .rightFrontEncoderDirection(Encoder.FORWARD)
            .rightRearEncoderDirection(Encoder.FORWARD)
            .robotWidth(18)
            .robotLength(16)
            .forwardTicksToInches(0.027523658580937155)
            .strafeTicksToInches(-0.7035642767208512)
            .turnTicksToInches(-4.54282988172469)
            ;

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .driveEncoderLocalizer(localizer)
                .build();
    }
}
