package org.firstinspires.ftc.teamcode.components;

import android.annotation.SuppressLint;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.HardwareUtil;
import org.firstinspires.ftc.teamcode.util.LogCatUtil;

import java.util.Arrays;
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
public class DualShooter extends BaseComponent {

    DcMotorEx shooter1;
    DcMotorEx shooter2;
    List<DcMotorEx> shooters;

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

    Double distanceToGoal;

    Pose goalPosition;

    public DualShooter(RobotContext context, Robot robot) {
        super(context);
        log = new LogCatUtil("DualShooter");
        hardwareUtil = new HardwareUtil(log, hardwareMap);
        this.robot = robot;

        shooter1 = hardwareUtil.getMotorEx("shooter1");
        shooter2 = hardwareUtil.getMotorEx("shooter2");
        shooters = Arrays.asList(shooter1, shooter2);
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
    }

    @Override
    public void init() {
        for(DcMotorEx shooter : shooters) {
            shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            setPIDFCoefficients(shooter, new PIDFCoefficients(600, 3, 0, 0, MotorControlAlgorithm.PIDF));
        }

        setDirection(shooter1, DcMotorSimple.Direction.REVERSE);
        setDirection(shooter2, DcMotorSimple.Direction.FORWARD);

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

    @SuppressLint("DefaultLocale")
    @Override
    public void update() {
        // 0.2ms
        goalPosition = robot.getTurret().getAlliance() ? Turret.blueGoal : Turret.redGoal;

        if(autoSpeed) {
//             effectively nothing?
            distanceToGoal = robot.getPose().distanceFrom(goalPosition);
//             5-10ms
            setVelocity(velocityFromDistance(distanceToGoal));
        }
    }

    public double velocityTicksToDegrees(int ticks) {
        return ticks * degPerTick;
    }

    @Override
    public void addTelemetry(){
        // 5ms
        telemetry.addLine(String.format("Shooter Velocity: %4d / %4d  (tick) | Ready: %b",
                (int) getVelocity(),
                setVelocity,
                isBusy()));
        telemetry.addData("Distance", robot.getPose().distanceFrom(goalPosition));
        // 3ms
        telemetry.addData("Shooter 1 Current", getShooter1Current());
        telemetry.addData("Shooter 2 Current", getShooter2Current());
        telemetry.addData("Shooters Current", getCombinedCurrent());
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
    protected DcMotorEx getShooter1(){
        return shooter1;
    }

    /**
     * @return Returns the shooter DcMotorEx to use outside of the component, for example, for getting the current in the Transfer component.
     */
    protected DcMotorEx getShooter2(){
        return shooter2;
    }

    protected List<DcMotorEx> getShooters(){
        return shooters;
    }

    /**
     * Sets the run mode of the shooter motor
     * @param runMode Run mode to set the shooter motor to
     */
    public void setMode(DcMotorEx motor, DcMotor.RunMode runMode) {
        motor.setMode(runMode);
    }

    public void setDirection(DcMotorSimple.Direction direction) {
        for(DcMotorEx shooter : shooters){
            setDirection(shooter, direction);
        }
    }

    public void setDirection(DcMotorEx motor, DcMotorSimple.Direction direction) {
        motor.setDirection(direction);
    }

    public void setPIDFCoefficients(PIDFCoefficients pidf){
        for(DcMotorEx shooter : shooters){
            setPIDFCoefficients(shooter, pidf);
        }
    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior){
        for(DcMotorEx shooter : shooters){
            shooter.setZeroPowerBehavior(behavior);
        }
    }

    /**
     * Sets PIDF Coefficients of the shooter motor
     * <p>Feedforward value is compensated based on battery voltage</p>
     * @param runMode Run Mode to set the shooter motor to
     * @param coefficients PIDF Coefficients to set on the shooter motor
     */
    public void setPIDFCoefficients(DcMotorEx motor, DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );

        motor.setPIDFCoefficients(runMode, compensatedCoefficients);
    }
    /**
     * Sets PIDF Coefficients of the shooter motor
     * <p>Feedforward value is compensated based on battery voltage</p>
     * @param coefficients PIDF Coefficients to set on the shooter motor
     */
    public void setPIDFCoefficients(DcMotorEx motor, PIDFCoefficients coefficients) {
        setPIDFCoefficients(motor, motor.getMode(), coefficients);
    }

    public int velocityFromDistance(double distance){
//        log.debug("distance : " + distance + " | speed : " + speeds.get(findClosestByStream(speeds.keys(), distance)) + " | real : " + shooter.getVelocity());
//         return speeds.get(findClosestByStream(speeds.keys(), distance));

        if (distance < 46+18) {
            distance = 46+18;
        }

        // search through speeds values to find the 2 neighbouring values
        int lowerBound = findClosestSmallerByStream(speeds.keys(), distance);
        int upperBound = findClosestLargerByStream(speeds.keys(), distance);

        // if the bot is exactly on a distance (or beyond the boundaries), return that speed to prevent division by 0
        if (lowerBound == upperBound) {
            return speeds.get(lowerBound);
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
    private int findClosestSmallerByStream(Enumeration<Integer> sortedNumbers, double target) {
        return findClosestSmallerByStream(Collections.list(sortedNumbers), target);
    }
    private int findClosestSmallerByStream(List<Integer> numbers, double target) {
        // returns the largest value less than the target, or the smallest value if none exists
        if (target < Collections.min(numbers)) {
            return Collections.min(numbers);
        }
        return numbers.stream()
                .max(Comparator.comparingInt(o -> (int) o <= target ? (int) o : Integer.MIN_VALUE))
                .get();
    }
    private int findClosestLargerByStream(Enumeration<Integer> sortedNumbers, double target) {
        return findClosestLargerByStream(Collections.list(sortedNumbers), target);
    }
    private int findClosestLargerByStream(List<Integer> numbers, double target) {
        // returns the smallest value greater than the target, or the largest value if none exists
        if (target > Collections.max(numbers)) {
            return Collections.max(numbers);
        }
        return numbers.stream()
                .min(Comparator.comparingInt(o -> (int) o >= target ? (int) o : Integer.MAX_VALUE))
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
        for(DcMotorEx shooter : shooters) {
            shooter.setVelocity(setVelocity);
        }
    }

    /**
     *
     * @return current velocity of the shooter motor
     */
    public double getVelocity(){
        return (shooter1.getVelocity() + shooter2.getVelocity()) / 2;
    }

    /**
     * Sets the shooter motor to run at it's hold velocity
     * <p>Hold velocity is the velocity the shooter motor spins at when not shooting</p>
     * <p>See setHoldVelocity( int )</p>
     */
    public void holdVelocity(){
        setVelocity(holdVelocity);
    }

    @Override
    public boolean isBusy() {
        // If the velocity is within the tolerance for stabilizationTime milliseconds, return false (not busy)
//        log.debug("Current Velocity: " + shooter.getVelocity());
//        log.debug("Set Velocity: " + setVelocity);
//        log.debug("Velocity Tolerance: " + velocityTolerance);
//
//        log.debug("is Busy: " + (shooter.getVelocity() < setVelocity - velocityTolerance && shooter.getVelocity() > setVelocity + velocityTolerance));
//        log.debug("Jonathan is Smarter: " + String.valueOf((shooter.getVelocity() < (setVelocity - velocityTolerance)) && (shooter.getVelocity() > (setVelocity + velocityTolerance))));
//
//        log.debug("Auto Speed: " + autoSpeed);

        return (getVelocity() < (setVelocity - velocityTolerance)) && (getVelocity() > (setVelocity + velocityTolerance));

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

    public double getShooter1Current(){
        return shooter1.getCurrent(CurrentUnit.AMPS);
    }

    public double getShooter2Current(){
        return shooter2.getCurrent(CurrentUnit.AMPS);
    }

    public double getCombinedCurrent(){
        return getShooter1Current() + getShooter2Current();
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