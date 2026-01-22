package org.firstinspires.ftc.teamcode.pedro;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
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
            .forwardZeroPowerAcceleration(-39.928058898966064)
            .lateralZeroPowerAcceleration(-82.48594579185915)
            .centripetalScaling(0.0009)

            .translationalPIDFCoefficients(new PIDFCoefficients(0.09, 0, 0.009, 0.04))
            .headingPIDFCoefficients(new PIDFCoefficients(1.5, 0.0, 0.001, 0.00))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.005,0.0,0.00005,0.6,0.005))

            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.09, 0, 0.009, 0.04))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.5, 0.3, 0.07, 0.06))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.01,0.0,0.0006,0.6,0.04))

            .useSecondaryTranslationalPIDF(dualPID)
            .useSecondaryHeadingPIDF(false)
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

    public static double[] seLinearScalars = {0.9789450036755697, 0.9713286068563092, 0.984250938654841, 0.967096238198983};
    public static double[] seAngularScalars = {0.9911039824022719, 0.994369789112311, 0.9939128037010939, 0.9948119413054476};

    public static double seLinearScalar = getAverage(seLinearScalars);
    public static double seAngularScalar = getAverage(seAngularScalars);

    public static OTOSConstants otosNormalLocalizerConstants = new OTOSConstants()
            .hardwareMapName("otos")
            .linearUnit(DistanceUnit.INCH)
            .angleUnit(AngleUnit.RADIANS)
            .linearScalar(seLinearScalar)
            .angularScalar(seAngularScalar)
            .offset(new SparkFunOTOS.Pose2D(-2.75, 3, Math.toRadians(180)));

    public static DriveEncoderConstants driveEncoderConstants = new DriveEncoderConstants()
            .rightFrontMotorName("rf")
            .rightRearMotorName("rr")
            .leftRearMotorName("lr")
            .leftFrontMotorName("lf")
            .leftFrontEncoderDirection(Encoder.REVERSE)
            .leftRearEncoderDirection(Encoder.REVERSE)
            .rightFrontEncoderDirection(Encoder.FORWARD)
            .rightRearEncoderDirection(Encoder.FORWARD)
            .robotWidth(9.500)
            .robotLength(7.875)
            .forwardTicksToInches(0.021956045405421835 * 0.3943661972)
            .strafeTicksToInches((0.05284855454518251 * 0.3943661972) * 0.4528301887)
            .turnTicksToInches((0.01772821447097184 * 0.3943661972) * 2.6111920708)
            ;



    public static PathConstraints pathConstraints = new PathConstraints(
            0.99,
            100,
            1,
            1);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(0.5)
            .rightFrontMotorName("rf")
            .rightRearMotorName("rr")
            .leftRearMotorName("lr")
            .leftFrontMotorName("lf")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(81.78650863527312)
            .yVelocity(65.83266370878444);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                //.setLocalizer(new otosAprilTagLocalizer(hardwareMap, otosLocalizerConstants))
                .OTOSLocalizer(otosNormalLocalizerConstants)
                //.threeWheelIMULocalizer(localizerConstants)
//                .driveEncoderLocalizer(driveEncoderConstants)
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
