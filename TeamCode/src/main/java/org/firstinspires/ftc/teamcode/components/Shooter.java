package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS.Pose2D;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.DistanceUtil;
import org.firstinspires.ftc.teamcode.util.HardwareUtil;
import org.firstinspires.ftc.teamcode.util.LogCatUtil;

import java.util.Collections;
import java.util.Comparator;
import java.util.Dictionary;
import java.util.Enumeration;
import java.util.Hashtable;
import java.util.List;

/**
 * Depends on:
 * DriveTrain
 */
public class Shooter extends BaseComponent {

    DcMotorEx shooter;

    double degPerTick = 12.8571428571;

    static Dictionary<Double, Integer> speeds = new Hashtable<>();

    int velocityTolerance = 60;
    int stabilizationTime = 750;
    int holdVelocity = 1600;
    private int setVelocity;

    private Robot robot;

    ElapsedTime shootTimer;
    
    VoltageSensor batteryVoltageSensor;
    
    LogCatUtil log;
    HardwareUtil hardwareUtil;

    SparkFunOTOS otos;

    Double distanceToTag;

    Pose2D goalPosition;

    public Shooter(RobotContext context, Robot robot) {
        super(context);
        log = new LogCatUtil("shooter");
        hardwareUtil = new HardwareUtil(log, hardwareMap);
        this.robot = robot;

        shooter = hardwareUtil.getMotorEx("shooter");
    }

    @Override
    public void init() {
        
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        shootTimer = new ElapsedTime();

        otos = robot.getDriveTrain().getOtos();

        goalPosition = context.alliance ? Turret.blueGoal : Turret.redGoal;

        // MAKE SURE THESE ARE IN ORDER FROM LOWEST TO HIGHEST DISTANCE
        speeds.put(40.0, 1640);
        speeds.put(50.0, 1500);
        speeds.put(60.0, 1480);
        speeds.put(70.0, 1480);
        speeds.put(80.0, 1520);
        speeds.put(90.0, 1520);
        speeds.put(100.0, 1560);
        speeds.put(110.0, 1640);
        speeds.put(120.0, 1600);
        speeds.put(130.0, 1640);
        speeds.put(140.0, 1780);
    }

    @Override
    public void update() {
        telemetry.addData("Velocity", shooter.getVelocity());
        telemetry.addData("Target", shooter.getPower());

        distanceToTag = DistanceUtil.distanceBetween(otos.getPosition(), goalPosition);
        setVelocity(velocityFromDistance(distanceToTag));
    }

    public double velocityTicksToDegrees(int ticks) {
        return ticks * degPerTick;
    }

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

    public void setDirection(DcMotorSimple.Direction direction) {
        shooter.setDirection(direction);
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

    public int velocityFromDistance(double distance){
        return speeds.get(findClosestByBiSearch(speeds.keys(), distance));
    }

    public double findClosestByBiSearch(Enumeration<Double> sortedNumbers, double target) {
        return findClosestByBiSearch(Collections.list(sortedNumbers), target);
    }

    public double findClosestByBiSearch(List<Double> sortedNumbers, double target) {
        double first = sortedNumbers.get(0);
        if (target <= first) {
            return first;
        }

        double last = sortedNumbers.get(sortedNumbers.size() - 1);
        if (target >= last) {
            return last;
        }

        int pos = Collections.binarySearch(sortedNumbers, target);
        if (pos > 0) {
            return sortedNumbers.get(pos);
        }
        int insertPos = -(pos + 1);
        double pre = sortedNumbers.get(insertPos - 1);
        double after = sortedNumbers.get(insertPos);

        return Math.abs(pre - target) <= Math.abs(after - target) ? pre : after;
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
        if(shooter.getVelocity() < setVelocity - velocityTolerance && shooter.getVelocity() > setVelocity + velocityTolerance) {
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