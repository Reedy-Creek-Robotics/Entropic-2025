package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Shooter extends BaseComponent {

    DcMotorEx shooter;
    int velocityTolerance = 50;
    ElapsedTime shootTimer;
    private int setVelocity;

    public Shooter(RobotContext context) {
        super(context);

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
    }

    @Override
    public void init() {
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shootTimer = new ElapsedTime();
    }

    public void setMode(DcMotor.RunMode runMode) {
        shooter.setMode(runMode);

    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        shooter.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );

        shooter.setPIDFCoefficients(runMode, compensatedCoefficients);
    }

    public void setPIDFCoefficients(PIDFCoefficients coefficients) {
        setPIDFCoefficients(shooter.getMode(), coefficients);
    }

    public void setVelocity(int velocity){
        setVelocity = velocity;
        shooter.setVelocity(setVelocity);
    }

    @Override
    public boolean isBusy() {
        // If the shooter velocity is outside of the tolerance, reset the timer.
        if(shooter.getVelocity() < setVelocity - velocityTolerance && shooter.getVelocity() > setVelocity + velocityTolerance) {
            shootTimer.reset();
        }

        // If the velocity is within the tolerance for 750ms, return false (not busy)
        return !(shootTimer.milliseconds() > 750);
    }
}
