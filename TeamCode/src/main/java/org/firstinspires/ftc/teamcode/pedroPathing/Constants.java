package org.firstinspires.ftc.teamcode.pedroPathing;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {

    final static double LB_PER_KG = 2.205;
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(22 / LB_PER_KG)
            .forwardZeroPowerAcceleration(-33.27)
            .lateralZeroPowerAcceleration(-56.73)
            .translationalPIDFCoefficients(new PIDFCoefficients(
                    0.1,
                    0,
                    0.01,
                    0.045))
            // 0.2, 0, 0.02, 0.035
            .headingPIDFCoefficients(new PIDFCoefficients(
                    1.5,
                    0,
                    0.05,
                    0.045))
            // 1.5, 0, 0.05, 0.035
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(
                    0.02,
                    0,
                    0.0002,
                    0.6,
                    0.001));

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("front_right")
            .rightRearMotorName("back_right")
            .leftRearMotorName("back_left")
            .leftFrontMotorName("front_left")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(80.45)
            .yVelocity(65.52);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(0)  // Y (sideways) offset of the forward pod
            .strafePodX(-7)  // X (up and down) offset of the strafe pod
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.99,
            100,
            1.1,
            1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
