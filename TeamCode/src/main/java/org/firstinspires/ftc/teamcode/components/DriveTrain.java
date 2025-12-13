package org.firstinspires.ftc.teamcode.components;

import static org.firstinspires.ftc.teamcode.components.RobotDescriptor.DriveTuner;
import static org.firstinspires.ftc.teamcode.components.RobotDescriptor.OdometryTuner;

import android.annotation.SuppressLint;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.teamcode.geometry.Heading;
import org.firstinspires.ftc.teamcode.util.DriveUtil;
import org.firstinspires.ftc.teamcode.util.EmptyObjectUtil;
import org.firstinspires.ftc.teamcode.util.LogCatUtil;

import java.util.Arrays;
import java.util.List;

@SuppressLint("DefaultLocale")
public class DriveTrain extends BaseComponent {
    @Override
    public void update() {
    }

    private DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private List<DcMotorEx> motors;

    private SparkFunOTOS otos;
    private VoltageSensor batteryVoltageSensor;

    private LogCatUtil log;

    private Robot robot;

    public static DriveTuner driveTuner;
    public static OdometryTuner odometryTuner;

    public DriveTrain(RobotContext context, Robot robot) {
        super(context);

        this.robot = robot;

        log = new LogCatUtil("DriveTrain");

        driveTuner = descriptor.DRIVE_TUNER;
        odometryTuner = descriptor.ODOMETRY_TUNER;

        //this.context.localizer = new StandardTrackingWheelLocalizer(hardwareMap, lastTrackingEncPositions, lastTrackingEncVels, odometryTuner);

        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        try{
            otos = hardwareMap.get(SparkFunOTOS.class, "otos");
        } catch (Exception e) {
            log.error("Device \"otos\" not found in hardware map. Defaulting to empty DcMotorEx object.");
            log.error(e.getMessage());
            otos = EmptyObjectUtil.getEmptySparkFunOTOS();
        }

        leftFront = hardwareMap.get(DcMotorEx.class, "lf");
        leftRear = hardwareMap.get(DcMotorEx.class, "lr");
        rightRear = hardwareMap.get(DcMotorEx.class, "rr");
        rightFront = hardwareMap.get(DcMotorEx.class, "rf");

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

    public void driverRelative(double drive, double strafe, double turn) {

    }

}
