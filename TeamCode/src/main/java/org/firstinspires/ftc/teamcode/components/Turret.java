package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import java.util.Arrays;
import java.util.List;

import org.firstinspires.ftc.teamcode.util.ArrayUtil;

public class Turret extends BaseComponent{

    /**
     * In rpm
     */
    static int baseMotorSpeed = 1150;
    static double baseTicksPerDeg =  300.0 / 360; //TODO: Change this
    static double drivePulleyTeeth = 24;
    static double turretPulleyTeeth = 134;

    static double gearRatio = turretPulleyTeeth / drivePulleyTeeth;

    /**
     * In rpm
     */
    static int[] speeds = {30, 43, 60, 84, 117, 223, 312, 435, 1150, 1620, 6000};
    /**
     * In kg.cm
     */
    static double[] torques = {250.0, 185.0, 133.2, 93.6, 68.4, 38.0, 24.3, 18.7, 7.9, 5.4, 1.5};
    static double[] ticksPerRevs = {5281.1, 3895.9, 2786.2, 1993.6, 1425.1, 751.8, 537.7, 384.5, 145.1, 103.8, 28.0};

    static int motorType = ArrayUtil.findIndexOfItem(speeds, baseMotorSpeed);
    /**
     * In kg.cm
     */
    static double baseMotorTorque = torques[motorType];
    static double baseTicksPerRev = ticksPerRevs[motorType];

    /**
     * Effective speed accounting for the gear ratio <br> Measured in RPM
     */
    static double effectiveSpeed = baseMotorSpeed / gearRatio;
    /**
     * Effective torque accounting for the gear ratio <br> Measured in kg.cm
     */
    static double effectiveTorque = baseMotorSpeed * gearRatio;
    static double effectiveTicksPerRev = baseTicksPerRev * gearRatio;


    /**
     * Configured like the motor, with it's base stats (rpm, tps, torque) <br> points to "turret" hardware map
     */
    DcMotorEx turretMotor;
    /**
     * Configured like the Lazy Suzan, with it's effective stats (rpm, tps, torque) <br> points to "turret" hardware map
     */
    DcMotorEx turretLazySuzan;
    double toleranceDeg = 5;

    public Turret(RobotContext context) {
        super(context);

        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turretLazySuzan = turretMotor;

        MotorConfigurationType motorConfiguration = turretMotor.getMotorType().clone();
        motorConfiguration.setAchieveableMaxRPMFraction(1.0);
        motorConfiguration.setMaxRPM(baseMotorSpeed);
        motorConfiguration.setTicksPerRev(baseTicksPerRev);

        MotorConfigurationType lazySuzanConfiguration = turretLazySuzan.getMotorType().clone();
        motorConfiguration.setAchieveableMaxRPMFraction(1.0);
        motorConfiguration.setMaxRPM(effectiveSpeed);
        motorConfiguration.setTicksPerRev(effectiveTicksPerRev);

        turretMotor.setMotorType(motorConfiguration);
        turretLazySuzan.setMotorType(lazySuzanConfiguration);

        turretMotor.setTargetPositionTolerance((int) (baseTicksPerDeg * toleranceDeg));
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void update(){
        telemetry.addData("Turret Pos:", getPositionTicks());
    }

    public void goToDeg(double targetDeg) {
        turretMotor.setTargetPosition((int) (targetDeg * baseTicksPerDeg));
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public int getPositionTicks(){
        return turretMotor.getCurrentPosition();
    }

    public double getPositionDeg(){
        return getPositionTicks() / baseTicksPerDeg;
    }


}
