package org.firstinspires.ftc.teamcode.components;

import android.util.Log;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS.Pose2D;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import org.firstinspires.ftc.teamcode.util.EmptyObjectUtil;

import java.util.Arrays;
import java.util.List;

import org.firstinspires.ftc.teamcode.util.ArrayUtil;

public class Turret extends BaseComponent{

    static String logTag = "Component-Turret";

    // Must be at least 360 degrees
    static double maxHeading = 360 * 2;
    static double minHeading = -360 * 2;

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
    static double effectiveTicksPerDeg = effectiveTicksPerRev / 360;

    static Pose2D redTag = new Pose2D(31.875, 58.5, 234);
    static Pose2D blueTag = new Pose2D(0, 0, 144); //ToDo update to match coordinate system

    Pose2D otosPos = new Pose2D();

    /**
     * Configured like the motor, with it's base stats (rpm, tps, torque) <br> points to "turret" hardware map
     */
    DcMotorEx turretMotor;
    /**
     * Configured like the Lazy Suzan, with it's effective stats (rpm, tps, torque) <br> points to "turret" hardware map
     */
    double toleranceDeg = 5;
    double targetPos = 0;

    /**
     * false - red (tag 24)<br>
     * true - blue (tag 20)
     */
    boolean alliance = false;


    public Turret(RobotContext context) {
        super(context);

        this.alliance = context.alliance;

        try {
            turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        }finally {
            Log.println(Log.ERROR, logTag, "Device \"turret\" not found in hardware map. Defaulting to empty motor object.");
            turretMotor = EmptyObjectUtil.getEmptyMotorEx();
        }

        MotorConfigurationType motorConfiguration = turretMotor.getMotorType().clone();
        motorConfiguration.setAchieveableMaxRPMFraction(1.0);
        motorConfiguration.setMaxRPM(baseMotorSpeed);
        motorConfiguration.setTicksPerRev(baseTicksPerRev);

        turretMotor.setMotorType(motorConfiguration);

        turretMotor.setTargetPositionTolerance((int) (baseTicksPerDeg * toleranceDeg));
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void update(){
        otosAutoAim();
        movePid();
        telemetry.addData("Turret Pos:", getPositionTicks());
    }

    private void setTargetDegrees(double degrees){
        //if(degrees < 0) degrees = 360 + degrees;
        degrees = degrees + getPositionDegrees();
        degrees = -degrees /*+ getPositionDegrees()*/;

        while(degrees > maxHeading){
            degrees = degrees - 360;
            Log.println(Log.WARN, "turret", "over max heading");
        }

        while(degrees < minHeading){
            degrees = degrees + 360;
            Log.println(Log.WARN, "turret", "under min heading");
        }

        targetPos = degrees * effectiveTicksPerDeg;
    }

    private double getPositionDegrees(){
        return turretMotor.getCurrentPosition() / effectiveTicksPerDeg;
    }

    public int getPositionTicks(){
        return turretMotor.getCurrentPosition();
    }

    private void otosAutoAim(){
        // Calculates the theta using tanh function
        double theta = Math.toDegrees(Math.tanh((alliance ? blueTag.x : redTag.x - otosPos.x) / (alliance ? blueTag.y : redTag.y - otosPos.y)));
        // We subtract the theta from the heading to account for robot rotation.
        setTargetDegrees(-otosPos.h - theta);
        telemetry.addData("theta", theta);
        telemetry.addData("delta deg", -otosPos.h - theta);
    }

    private void moveRtp(){

    }

    private void movePid(){
        //turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (Math.abs(turretMotor.getCurrentPosition() - targetPos) <= 3) {
            turretMotor.setPower(0);
        } else if (Math.abs(turretMotor.getCurrentPosition() - targetPos) <= 25) {
            turretMotor.setPower(turretMotor.getCurrentPosition() < (int) targetPos ? 0.2 : -0.2);
        } else {
            turretMotor.setPower(turretMotor.getCurrentPosition() < (int) targetPos ? 1 : -1);
        }
    }

}
