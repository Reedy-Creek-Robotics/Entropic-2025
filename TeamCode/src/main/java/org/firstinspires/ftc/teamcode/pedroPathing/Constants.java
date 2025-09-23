package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.OTOSConstants;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
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

@Configurable
public class Constants {

    static boolean dualPID = false;

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(6.9) // kg
            .forwardZeroPowerAcceleration(-60.03045132454115) // 3 Wheel
            .lateralZeroPowerAcceleration(-77.63231532212639)
            .centripetalScaling(0.0009)

            .translationalPIDFCoefficients(new PIDFCoefficients(0.09, 0, 0.009, 0.04))
            .headingPIDFCoefficients(new PIDFCoefficients(2.5, 0.0, 0.02, 0.04))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.01,0.0,0.0006,0.6,0.04))

            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.09, 0, 0.009, 0.04))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.5, 0.3, 0.07, 0.06))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.01,0.0,0.0006,0.6,0.04))

            .useSecondaryTranslationalPIDF(dualPID)
            .useSecondaryHeadingPIDF(dualPID)
            .useSecondaryDrivePIDF(dualPID);




    public static ThreeWheelIMUConstants ThreeWheelImuLocalizerConstants = new ThreeWheelIMUConstants()
            .forwardTicksToInches(-0.0030552567962607207)
            .strafeTicksToInches(-0.003941643869723979)
            .turnTicksToInches(-0.003941643869723979)
    public static otosAprilTagConstants otosLocalizerConstants = new otosAprilTagConstants()
            .angleUnit(AngleUnit.RADIANS)
            .linearUnit(DistanceUnit.INCH)
            .angularScalar(1)
            .linearScalar(1);

    public static ThreeWheelIMUConstants localizerConstants = new ThreeWheelIMUConstants()
            .forwardTicksToInches(.003)
            .strafeTicksToInches(.003)
            .turnTicksToInches(.003)
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

    public static OTOSConstants otosLocalizerConstants = new OTOSConstants()
            .hardwareMapName("otos")
            .linearUnit(DistanceUnit.INCH)
            .angleUnit(AngleUnit.RADIANS)
            .offset(new SparkFunOTOS.Pose2D(-6.1, 4.9, Math.toRadians(90)))
            .linearScalar(0.98432063)
            .angularScalar(0.97384378);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.99,
            100,
            1,
            1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .setLocalizer(new otosAprilTagLocalizer(hardwareMap, otosLocalizerConstants))
                //.threeWheelIMULocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }

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
            .xVelocity(58.092132328063485) // 3 Wheel
            .yVelocity(45.65561850239912);
            /*.xVelocity(38.35723156065454) // OTOS
            .yVelocity(37.20200906588337);*/
}
