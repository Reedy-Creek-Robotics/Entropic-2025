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

    static Dictionary<Integer, Integer> tunedSpeedValues = new Hashtable<>();

    Dictionary<Integer, Integer> speeds = new Hashtable<>();
    int lookupTableInterval = 1; //inches PLEASE DON'T CHANGE THIS STUFF WILL BREAK

    int velocityTolerance = 60;
    int stabilizationTime = 750;
    int holdVelocity = 1600;

    double shotCurrent = 4;

    private int setVelocity;

    boolean autoSpeed = true;

    private final Robot robot;

    ElapsedTime shootTimer;
    
    VoltageSensor batteryVoltageSensor;
    
    LogCatUtil log;
    HardwareUtil hardwareUtil;

    Double distanceToGoal;

    Pose goalPosition;

    public Shooter(RobotContext context, Robot robot) {
        super(context);
        log = new LogCatUtil("Shooter");
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
        tunedSpeedValues.put(46+18, 1260);
        tunedSpeedValues.put(56+18, 1260);
        tunedSpeedValues.put(66+18, 1200);
        tunedSpeedValues.put(76+18, 1240);
        tunedSpeedValues.put(86+18, 1280);
        tunedSpeedValues.put(96+18, 1300);
        tunedSpeedValues.put(116+18, 1380);
        tunedSpeedValues.put(126+18, 1440);
        tunedSpeedValues.put(134+18, 1480);

        createSpeedDictionary();
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void update() {
        // 0.2ms
        goalPosition = robot.getTurret().getTargetGoal();

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
                (int) shooter.getVelocity(),
                setVelocity,
                isBusy()));
        telemetry.addData("Distance", robot.getPose().distanceFrom(goalPosition));
        // 3ms
        telemetry.addData("Shooter Current", getShooterCurrent());
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
        if(distance < 0) distance = 0;
        if(distance > 204) distance = 204;

        return speeds.get(distance);
    }

    private void createSpeedDictionary(){
        for(int i = 0; i <= 204; i += lookupTableInterval) {
            int distance = i;
            //        log.debug("distance : " + distance + " | speed : " + speeds.get(findClosestByStream(speeds.keys(), distance)) + " | real : " + shooter.getVelocity());
            //        return speeds.get(findClosestByStream(speeds.keys(), distance));

            if (distance < tunedSpeedValues.keys().asIterator().next()) {
                distance = tunedSpeedValues.keys().asIterator().next();
            }

            // search through speeds values to find the 2 neighbouring values
            int lowerBound = findClosestSmallerByStream(tunedSpeedValues.keys(), distance);
            int upperBound = findClosestLargerByStream(tunedSpeedValues.keys(), distance);

            // if the bot is exactly on a distance (or beyond the boundaries), return that speed to prevent division by 0
            if (lowerBound == upperBound) {
                speeds.put(i, lowerBound);
                continue;
            }

            // interpolate values with point slope
            double slope = (double) (tunedSpeedValues.get(upperBound) - tunedSpeedValues.get(lowerBound)) / (upperBound - lowerBound);
            double expectedTPS = slope * (distance - lowerBound) + tunedSpeedValues.get(lowerBound);

            speeds.put(i, (int) expectedTPS);
        }
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
        // If the velocity is within the tolerance for stabilizationTime milliseconds, return false (not busy)
//        log.debug("Current Velocity: " + shooter.getVelocity());
//        log.debug("Set Velocity: " + setVelocity);
//        log.debug("Velocity Tolerance: " + velocityTolerance);
//
//        log.debug("is Busy: " + (shooter.getVelocity() < setVelocity - velocityTolerance && shooter.getVelocity() > setVelocity + velocityTolerance));
//        log.debug("Jonathan is Smarter: " + String.valueOf((shooter.getVelocity() < (setVelocity - velocityTolerance)) && (shooter.getVelocity() > (setVelocity + velocityTolerance))));
//
//        log.debug("Auto Speed: " + autoSpeed);

        return (shooter.getVelocity() < (setVelocity - velocityTolerance)) && (shooter.getVelocity() > (setVelocity + velocityTolerance));

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