package org.firstinspires.ftc.teamcode.pedro;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.OTOSConstants;
import com.pedropathing.ftc.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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

    public static double[] linearScalars = {0.9760748986073785, 0.9718011578691315, 0.9813496831245394, 0.9893696285289747};
    public static double[] angularScalars = {0.9905234043196346, 0.987627529876274, 0.9892674385255185, 0.9897305787120929};

    public static OTOSConstants otosNormalLocalizerConstants = new OTOSConstants()
            .hardwareMapName("otos")
            .linearUnit(DistanceUnit.INCH)
            .angleUnit(AngleUnit.RADIANS)
            .linearScalar(getAverage(linearScalars))
            .angularScalar(getAverage(angularScalars))
            .offset(new SparkFunOTOS.Pose2D(-2.75, 3, Math.toRadians(180)));

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

    public static double getAverage(double[] list){
        double total = 0;
        for(double number : list){
            total += number;
        }
        return total/list.length;
    }
}
