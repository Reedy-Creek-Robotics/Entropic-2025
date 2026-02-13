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
import com.pedropathing.ftc.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.Rev9AxisImuOrientationOnRobot;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.teamcode.custom.OtosImuConstants;
import org.firstinspires.ftc.teamcode.custom.OtosImuLocalizer;

@Configurable
public class Constants {

    static boolean dualPID = false;

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.34) // kg
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
            .IMU_Orientation(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP));

    public static double[] seLinearScalars = {1.0010764494377409, 1.0133037345530145, 1.0005626816936104, 1.01993009954486};
    public static double[] seAngularScalars = {0.9978221801411064, 0.997928858342879, 0.9971521533437077, 0.9980355593572452};

    public static double seLinearScalar = getAverage(seLinearScalars);
    public static double seAngularScalar = getAverage(seAngularScalars);

    public static double[] forgeKnockoffLinearScalars = {0};
    public static double[] forgeKnockoffAngularScalars = {0};

   public static double forgeKnockoffLinearScalar = getAverage(forgeKnockoffLinearScalars);
   public static double forgeKnockoffAngularScalar = getAverage(forgeKnockoffAngularScalars);

    public static double[] houseKnockoffLinearScalars = {0};
    public static double[] houseKnockoffAngularScalars = {0};

    public static double houseKnockoffLinearScalar = getAverage(houseKnockoffLinearScalars);
    public static double houseKnockoffAngularScalar = getAverage(houseKnockoffAngularScalars);

    public static OtosImuConstants otosImuConstants = new OtosImuConstants()
            .otosHardwareMapName("otos")
//            .imuHardwareMapName("extImu")
//            .imuParameters(new IMU.Parameters(new Rev9AxisImuOrientationOnRobot(Rev9AxisImuOrientationOnRobot.LogoFacingDirection.LEFT, Rev9AxisImuOrientationOnRobot.I2cPortFacingDirection.FORWARD)))
            .imuHardwareMapName("imu")
            .imuParameters(new IMU.Parameters(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.LEFT, RevHubOrientationOnRobot.UsbFacingDirection.UP)))
            .linearUnit(DistanceUnit.INCH)
            .angleUnit(AngleUnit.RADIANS)
            .linearScalar(seLinearScalar)
//            .angularScalar(seAngularScalar)
            .offset(new SparkFunOTOS.Pose2D(2.7, 2.34, Math.toRadians(180)));

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
            .maxPower(1)
            .rightFrontMotorName("rf")
            .rightRearMotorName("rr")
            .leftRearMotorName("lr")
            .leftFrontMotorName("lf")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(81.78650863527312)
            .yVelocity(65.83266370878444)
//            .useVoltageCompensation(true) TODO try this out???
            ;

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                //.setLocalizer(new otosAprilTagLocalizer(hardwareMap, otosLocalizerConstants))
                .setLocalizer(new OtosImuLocalizer(hardwareMap, otosImuConstants))
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
