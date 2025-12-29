package org.firstinspires.ftc.teamcode.components;

import static org.firstinspires.ftc.teamcode.components.RobotDescriptor.DriveTuner;
import static org.firstinspires.ftc.teamcode.components.RobotDescriptor.OdometryTuner;
import static org.firstinspires.ftc.teamcode.pedro.Constants.getAverage;
import static org.firstinspires.ftc.teamcode.pedro.Constants.linearScalars;
import static org.firstinspires.ftc.teamcode.pedro.Constants.angularScalars;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.geometry.Heading;
import org.firstinspires.ftc.teamcode.util.DriveUtil;
import org.firstinspires.ftc.teamcode.util.EmptyObjectUtil;
import org.firstinspires.ftc.teamcode.util.HardwareUtil;
import org.firstinspires.ftc.teamcode.util.LogCatUtil;

import java.util.Arrays;
import java.util.List;

@SuppressLint("DefaultLocale")
public class DriveTrain extends BaseComponent {

    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;

    private SparkFunOTOS otos;
    private VoltageSensor batteryVoltageSensor;

    private LogCatUtil log;
    private HardwareUtil hardwareUtil;

    private Robot robot;

    public static DriveTuner driveTuner;
    public static OdometryTuner odometryTuner;

    SparkFunOTOS.Pose2D curPose;

    public DriveTrain(RobotContext context, Robot robot) {
        super(context);

        this.robot = robot;

        log = new LogCatUtil("DriveTrain");
        hardwareUtil = new HardwareUtil(log, hardwareMap);

        driveTuner = descriptor.DRIVE_TUNER;
        odometryTuner = descriptor.ODOMETRY_TUNER;

        //this.context.localizer = new StandardTrackingWheelLocalizer(hardwareMap, lastTrackingEncPositions, lastTrackingEncVels, odometryTuner);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        
        otos = hardwareUtil.getOtos("otos");
    }

    @Override
    public void init() {
        super.init();
        
        
        leftFront = hardwareUtil.getMotorEx("lf");
        leftRear = hardwareUtil.getMotorEx("lr");
        rightRear = hardwareUtil.getMotorEx("rr");
        rightFront = hardwareUtil.getMotorEx("rf");

        leftFront.setDirection(DcMotorEx.Direction.REVERSE);
        leftRear.setDirection(DcMotorEx.Direction.REVERSE);
        rightRear.setDirection(DcMotorEx.Direction.FORWARD);
        rightFront.setDirection(DcMotorEx.Direction.FORWARD);

        motors = Arrays.asList(leftFront, leftRear, rightRear, rightFront);

        for (DcMotorEx motor : motors) {
            MotorConfigurationType motorConfigurationType = motor.getMotorType().clone();
            motorConfigurationType.setAchieveableMaxRPMFraction(1.0);
            motor.setMotorType(motorConfigurationType);
        }


        if (driveTuner.runUsingEncoder) {
            setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        if (driveTuner.runUsingEncoder && driveTuner.driveMotorVeloPid != null) {
            setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, driveTuner.driveMotorVeloPid);
        }

        otos.setAngularUnit(AngleUnit.RADIANS);
        otos.setLinearUnit(DistanceUnit.INCH);

        // For example, if
        // the sensor is mounted 5 inches to the left (negative X) and 10 inches
        // forward (positive Y) of the center of the robot, and mounted 90 degrees
        // clockwise (negative rotation) from the robot's orientation, the offset
        // would be {-5, 10, -90}. These can be any value, even the angle can be
        // tweaked slightly to compensate for imperfect mounting (eg. 1.3 degrees).
        otos.setOffset(new SparkFunOTOS.Pose2D(-2.75, 3, Math.toRadians(180)));

        otos.setLinearScalar(getAverage(linearScalars));
        otos.setAngularScalar(getAverage(angularScalars));
        otos.calibrateImu();
    }

    @Override
    public void update() {
        curPose = otos.getPosition();
        telemetry.addData("otos x", curPose.x);
        telemetry.addData("otos y", curPose.y);
        telemetry.addData("otos h", Math.toDegrees(curPose.h));
    }

    public DriveTrain(RobotContext context){
        this(context, null);
    }


    public void setMode(DcMotor.RunMode runMode) {
        for (DcMotorEx motor : motors) {
            motor.setMode(runMode);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        for (DcMotorEx motor : motors) {
            motor.setZeroPowerBehavior(zeroPowerBehavior);
        }
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );

        for (DcMotorEx motor : motors) {
            motor.setPIDFCoefficients(runMode, compensatedCoefficients);
        }
    }

    public SparkFunOTOS getOtos(){
        return otos;
    }

    public void drive(double drive, double strafe, double turn, double speedFactor) {
        DriveUtil.MotorPowers motorPowers = context.driveUtil.calculateWheelPowerForDrive(drive, strafe, turn, speedFactor);

        leftFront.setPower(motorPowers.frontLeft);
        leftRear.setPower(motorPowers.backLeft);
        rightFront.setPower(motorPowers.frontRight);
        rightRear.setPower(motorPowers.backRight);
    }

    public void drive(double drive, double strafe, double turn) {
        drive(drive, strafe, turn, 1);
    }

    public void driverRelative(double drive, double strafe, double turn) {

    }

}
