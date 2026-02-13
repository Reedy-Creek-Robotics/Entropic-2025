package org.firstinspires.ftc.teamcode.components;

import static org.firstinspires.ftc.teamcode.components.RobotDescriptor.DriveTuner;
import static org.firstinspires.ftc.teamcode.components.RobotDescriptor.OdometryTuner;

import android.annotation.SuppressLint;
import android.app.Notification;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.geometry.Heading;
import org.firstinspires.ftc.teamcode.pedro.Constants;
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

    private Follower follower;

    private VoltageSensor batteryVoltageSensor;

    private LogCatUtil log;
    private HardwareUtil hardwareUtil;

    private Robot robot;

    private SparkFunOTOS otos;

    public static DriveTuner driveTuner;
    public static OdometryTuner odometryTuner;

    Pose curPose;

    public DriveTrain(RobotContext context, Robot robot) {
        super(context);

        this.robot = robot;

        log = new LogCatUtil("DriveTrain");
        hardwareUtil = new HardwareUtil(log, hardwareMap);

        driveTuner = descriptor.DRIVE_TUNER;
        odometryTuner = descriptor.ODOMETRY_TUNER;

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        leftFront = hardwareUtil.getMotorEx("lf");
        leftRear = hardwareUtil.getMotorEx("lr");
        rightRear = hardwareUtil.getMotorEx("rr");
        rightFront = hardwareUtil.getMotorEx("rf");

        otos = hardwareUtil.getOtos("otos");

        follower = Constants.createFollower(hardwareMap);
    }

    @Override
    public void init() {
        super.init();

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
    }

    @Override
    public void update() {
        curPose = robot.getPose();
    }

    @Override
    public void addTelemetry(){
        telemetry.addLine(String.format("XYH %6.2f %6.2f %6.2f  (inch) (degree)", curPose.getX(), curPose.getY(), Math.toDegrees(curPose.getHeading())));
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

    public Follower getFollower(){
        return follower;
    }

    public void setPos(Pose newPose){
        follower.setPose(newPose);
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

    public void nothingForTime(double timeMs){
        robot.executeCommand(new NothingForTime(timeMs));
    }

    private class NothingForTime implements Command {

        double time;
        ElapsedTime timer;

        public NothingForTime(double timeMs){
            this.time = timeMs;
        }

        @Override
        public void start(){
            timer = new ElapsedTime();
        }

        @Override
        public void stop() {

        }

        @Override
        public boolean update() {
            return timer.milliseconds() > time;
        }
    }
}
