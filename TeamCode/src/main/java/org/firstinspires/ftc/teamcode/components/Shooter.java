package org.firstinspires.ftc.teamcode.components;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS.Pose2D;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
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

    static Dictionary<Integer, Integer> speeds = new Hashtable<>();

    int velocityTolerance = 60;
    int stabilizationTime = 750;
    int holdVelocity = 1600;

    double shotCurrent = 4;

    private int setVelocity;

    boolean autoSpeed = true;

    private final Robot robot;

    ElapsedTime shootTimer;

    Follower follower;
    
    VoltageSensor batteryVoltageSensor;
    
    LogCatUtil log;
    HardwareUtil hardwareUtil;

    Double distanceToTag;

    Pose goalPosition;

    public Shooter(RobotContext context, Robot robot) {
        super(context);
        log = new LogCatUtil("shooter");
        hardwareUtil = new HardwareUtil(log, hardwareMap);
        this.robot = robot;

        shooter = hardwareUtil.getMotorEx("shooter");
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    @Override
    public void init() {
        
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        setPIDFCoefficients(new PIDFCoefficients(600, 3, 0, 0, MotorControlAlgorithm.PIDF));
        shootTimer = new ElapsedTime();

        // All distances were measured to the april tag, 18in is added for the distance to the corner from the tag
        speeds.put(46+18, 1260);
        speeds.put(56+18, 1260);
        speeds.put(66+18, 1200);
        speeds.put(76+18, 1240);
        speeds.put(86+18, 1280);
        speeds.put(96+18, 1300);
        speeds.put(116+18, 1380);
        speeds.put(126+18, 1440);
        speeds.put(134+18, 1480);

        follower = robot.getDriveTrain().getFollower();
    }

    @Override
    public void update() {
        goalPosition = context.alliance ? Turret.blueGoal : Turret.redGoal;
        telemetry.addLine(String.format("Shooter Velocity: %4d / %4d  (tick) | Ready: %b",
                (int) shooter.getVelocity(),
                setVelocity,
                isBusy()));
        telemetry.addData("Distance", follower.getPose().distanceFrom(goalPosition));

        if(autoSpeed) {
            distanceToTag = follower.getPose().distanceFrom(goalPosition);
            setVelocity(velocityFromDistance(distanceToTag));
        }
        telemetry.addData("Shooter Current", getShooterCurrent());
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
        //log.debug("distance : " + distance + " | speed : " + speeds.get(findClosestByStream(speeds.keys(), distance)) + " | real : " + shooter.getVelocity());
//        return speeds.get(findClosestByStream(speeds.keys(), distance));
        if (distance < 46) {
            distance = 46;
        }
        // linear search through speeds values to find the 2 neighbouring values
        int lowerBound = -1;
        int upperBound = -1;
        Enumeration<Integer> measuredDistances = speeds.keys();
        while (measuredDistances.hasMoreElements()) {
            int measuredDistance = measuredDistances.nextElement();
            if (distance >= measuredDistance) {
                lowerBound = measuredDistance;
                if (measuredDistances.hasMoreElements()) {
                    upperBound = measuredDistances.nextElement();
                } else {
                    return 0;
                }
                break;
            }
        }
        // check if we actually found a speed value
        if (lowerBound == -1) {
            return 0;
        }

        // interpolate values with point slope
        double slope = (double) (speeds.get(upperBound) - speeds.get(lowerBound)) / (upperBound - lowerBound);
        double expectedTPS = slope * (distance - lowerBound) + speeds.get(lowerBound);
        return (int) expectedTPS;
    }

    private int findClosestByStream(Enumeration<Integer> sortedNumbers, double target) {
        return findClosestByStream(Collections.list(sortedNumbers), target);
    }

    private int findClosestByStream(List<Integer> numbers, double target) {
        return numbers.stream()
                .min(Comparator.comparingInt(o -> (int) Math.abs(o - target)))
                .get();
    }

    public boolean setAutoSpeed(boolean autoSpeed){
        return this.autoSpeed = autoSpeed;
    }

    public boolean getAutoSpeed(){
        return autoSpeed;
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