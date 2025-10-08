package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Shooter extends BaseComponent {

    DcMotorEx shooter;

    double degPerTick = 12.8571428571;

    int velocityTolerance = 50;
    int stabilizationTime = 750;
    int holdVelocity = 1600;
    private int setVelocity;

    ElapsedTime shootTimer;

    String shooterHardwareName;

    public Shooter(RobotContext context, String shooterHardwareName) {
        super(context);
        this.shooterHardwareName = shooterHardwareName;
        shooter = hardwareMap.get(DcMotorEx.class, shooterHardwareName);
    }

    @Override
    public void init() {

        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shootTimer = new ElapsedTime();
    }

    @Override
    public void update() {
        telemetry.addData("Velocity", shooter.getVelocity());
        telemetry.addData("Target", setVelocity);
        telemetry.addData("Shooter Current (from Shooter)", shooter.getCurrent(CurrentUnit.AMPS));
    }

    public double velocityTicksToDegrees(int ticks) {return ticks * degPerTick;}

    /**
     * Sets the hold velocity of the shooter motor
     * <p>Hold velocity is the velocity the shooter motor spins at when not shooting</p>
     * @param holdVelocity Hold velocity to set
     */
    public void setHoldVelocity(int holdVelocity){
        this.holdVelocity = holdVelocity;
    }

    /**
     * @return Returns the shooter DcMotorEx to use outside of the component, for example, for getting the current in the Transfer component.
     */
    protected DcMotorEx getShooter(){
        return shooter;
    }

    /**
     * Sets the run mode of the shooter motor
     * @param runMode Run mode to set the shooter motor to
     */
    public void setMode(DcMotor.RunMode runMode) {
        shooter.setMode(runMode);

    }

    /**
     * Sets the zero power behavior of the shooter motor
     * @param zeroPowerBehavior Zero power behavior to set the shooter motor to
     */
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        shooter.setZeroPowerBehavior(zeroPowerBehavior);
    }

    /**
     * Sets PIDF Coefficients of the shooter motor
     * <p>Feedforward value is compensated based on battery voltage</p>
     * @param runMode Run Mode to set the shooter motor to
     * @param coefficients PIDF Coefficients to set on the shooter motor
     */
    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );

        shooter.setPIDFCoefficients(runMode, compensatedCoefficients);
    }
    /**
     * Sets PIDF Coefficients of the shooter motor
     * <p>Feedforward value is compensated based on battery voltage</p>
     * @param coefficients PIDF Coefficients to set on the shooter motor
     */
    public void setPIDFCoefficients(PIDFCoefficients coefficients) {
        setPIDFCoefficients(shooter.getMode(), coefficients);
    }

    /*
        ~1700 tps from 43in
        ~1640 tps from 46in
        ~1540 tps from 63in
        ~1600 tps from 77in
        ~1580 tps from 95in
        ~1720 tps from 123in
        ~1780 tps from 144in
     */
    public int velocityFromDistance(int distance){
//        if(distance < 40) return 0;
//        if(distance > 150) return 0;
//
//        if(distance >= 40 && distance <= 50)  return 1680;
//        if(distance  > 50 && distance <= 65)  return 1560;
//        if(distance  > 65 && distance <= 95)  return 1600;
//        if(distance  > 95 && distance <= 125) return 1700;
//        if(distance  > 125)                   return 1780;
//
//        return 0;

        switch(distance) {
            case 0:
                return 0;
            case 1:
                return 1;
            case 2:
                return 2;
        }
        return 0;
    }

    /**
     * Set velocity for the shooter motor to spin at
     * @param velocity Target velocity for the shooter motor
     */
    public void setVelocity(int velocity){
        setVelocity = velocity;
        shooter.setVelocity(setVelocity);
    }

    /**
     *
     * @return current velocity of the shooter motor
     */
    public double getVelocity(){
        return shooter.getVelocity();
    }

    /**
     * Sets the shooter motor to run at it's hold velocity
     * <p>Hold velocity is the velocity the shooter motor spins at when not shooting</p>
     * <p>See setHoldVelocity( int )</p>
     */
    public void holdVelocity(){
        shooter.setVelocity(holdVelocity);
        setVelocity = holdVelocity;
    }

    @Override
    public boolean isBusy() {
        // If the shooter velocity is outside of the tolerance, reset the timer.
        if(shooter.getVelocity(AngleUnit.DEGREES) < setVelocity - velocityTolerance && shooter.getVelocity(AngleUnit.DEGREES) > setVelocity + velocityTolerance) {
            shootTimer.reset();
        }

        // If the velocity is within the tolerance for stabilizationTime milliseconds, return false (not busy)
        return shootTimer.milliseconds() < stabilizationTime;
    }

    /**
     * Spins shooter motor up to given velocity
     * @param velocity Velocity to set
     */
    public void spinToVelocity(int velocity){
        executeCommand(new SpinToVelocity(velocity));
    }

    public void shootAtDistance(int distance){
        int velocity;
        if((velocity = velocityFromDistance(distance)) == 0){
            velocity = 1600;
        }
        shootAtVelocity(velocity);
    }

    public double getShooterCurrent(){
        return shooter.getCurrent(CurrentUnit.AMPS);
    }

    /**
     * Spins shooter motor up to given velocity and shoots after it reaches that velocity
     * @param velocity Velocity to set
     */
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

        }

        @Override
        public boolean update() {
            return isBusy();
        }
    }
}
