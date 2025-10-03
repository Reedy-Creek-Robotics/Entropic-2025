package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Shooter extends BaseComponent {

    DcMotorEx shooter;
    int velocityTolerance = 50;
    int stabilizationTime = 750;
    int holdVelocity = 1600;
    ElapsedTime shootTimer;
    private int setVelocity;

    public Shooter(RobotContext context) {
        super(context);
    }

    @Override
    public void init() {
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");

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

    public void holdVelocity(){
        shooter.setVelocity(holdVelocity);
    }

    @Override
    public boolean isBusy() {
        // If the shooter velocity is outside of the tolerance, reset the timer.
        if(shooter.getVelocity() < setVelocity - velocityTolerance && shooter.getVelocity() > setVelocity + velocityTolerance) {
            shootTimer.reset();
        }

        // If the velocity is within the tolerance for stabilizationTime milliseconds, return false (not busy)
        return shootTimer.milliseconds() < stabilizationTime;
    }

    public void spinToVelocity(int velocity){
        executeCommand(new SpinToVelocity(velocity));
    }

    public void shootAtVelocity(int velocity){
        executeCommand(new ShootAtVelocity(velocity));
    }

    public class SpinToVelocity implements Command{

        int velocity;

        public SpinToVelocity(int velocity){
            this.velocity = velocity;
        }

        @Override
        public void start() {
            setVelocity(velocity);
        }

        @Override
        public void stop() {

        }

        @Override
        public boolean update() {
            return isBusy();
        }
    }

    public class ShootAtVelocity implements Command{

        int velocity;

        public ShootAtVelocity(int velocity){
            this.velocity = velocity;
        }

        @Override
        public void start() {
            setVelocity(velocity);
        }

        @Override
        public void stop() {
            //ToDo Add functionality to make transfer push artifact into shooter.
        }

        @Override
        public boolean update() {
            return isBusy();
        }
    }
}
