package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import org.firstinspires.ftc.teamcode.pedroPathing.otosAprilTagLocalizer;
import com.pedropathing.ftc.localization.constants.OTOSConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.otosAprilTagLocalizer;
import com.pedropathing.ftc.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import java.lang.reflect.Array;
import java.util.Arrays;
import java.util.List;

@Configurable
public class Constants {

    static boolean dualPID = false;

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(9) // kg
            .forwardZeroPowerAcceleration(-90.44761588635672)
            .lateralZeroPowerAcceleration(-85.62064150812581)
            .centripetalScaling(0.0009)

            .translationalPIDFCoefficients(new PIDFCoefficients(0.09, 0, 0.009, 0.04))
            .headingPIDFCoefficients(new PIDFCoefficients(2, 0.0, 0.02, 0.04))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.01,0.0,0.0006,0.6,0.04))

            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.09, 0, 0.009, 0.04))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.5, 0.3, 0.07, 0.06))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.01,0.0,0.0006,0.6,0.04))

            .useSecondaryTranslationalPIDF(dualPID)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(dualPID);


    public static ThreeWheelIMUConstants ThreeWheelImuLocalizerConstants = new ThreeWheelIMUConstants()
            .forwardTicksToInches(-0.0030552567962607207)
            .strafeTicksToInches(-0.003941643869723979)
            .turnTicksToInches(-0.003941643869723979)
            .leftPodY(7.5)
            .rightPodY(-7.5)
            .strafePodX(-4.5)
            .leftEncoder_HardwareMapName("lf")
            .rightEncoder_HardwareMapName("rf")
            .strafeEncoder_HardwareMapName("lr")
            .leftEncoderDirection(Encoder.FORWARD)
            .rightEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.REVERSE)
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.DOWN, RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));

    public static otosAprilTagConstants otosLocalizerConstants = new otosAprilTagConstants()
            .otosHardwareMapName("otos")
            .linearUnit(DistanceUnit.INCH)
            .angleUnit(AngleUnit.RADIANS)
            .offset(new SparkFunOTOS.Pose2D(-6.1, 4.9, Math.toRadians(90)))
            .linearScalar(0.98432063)
            .angularScalar(0.97384378);

    static double[] linearScalars = { /* forward */ 1.0722153945249597, 1.0653532159999999, 1.0936420914317, 1.076549329021827 /* lateral */ };
    static double[] angularScalars = {0.9644738778513654, 0.9524473898383984, 0.9524473898383984, 0.9644312981031047};

    public static OTOSConstants otosNormalLocalizerConstants = new OTOSConstants()
            .hardwareMapName("otos")
            .linearUnit(DistanceUnit.INCH)
            .angleUnit(AngleUnit.RADIANS)
            .linearScalar(getAverage(linearScalars))
            .angularScalar(getAverage(angularScalars))
            .offset(new SparkFunOTOS.Pose2D(-1.75, -4.75, Math.toRadians(90)));

    public static PathConstraints pathConstraints = new PathConstraints(
            0.99,
            100,
            1,
            1);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rf")
            .rightRearMotorName("rr")
            .leftRearMotorName("lr")
            .leftFrontMotorName("lf")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(67.59403258796752)
            .yVelocity(55.55520846149115);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                //.setLocalizer(new otosAprilTagLocalizer(hardwareMap, otosLocalizerConstants))
                .OTOSLocalizer(otosNormalLocalizerConstants)
                //.threeWheelIMULocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }

    private static double getAverage(double[] list){
        double total = 0;
        for(double number : list){
            total += number;
        }
        return total/list.length;
    }
}
