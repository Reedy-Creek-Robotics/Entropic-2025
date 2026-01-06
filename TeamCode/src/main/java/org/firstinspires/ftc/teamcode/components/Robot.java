package org.firstinspires.ftc.teamcode.components;

import android.annotation.SuppressLint;


import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.ErrorUtil;
import org.firstinspires.ftc.teamcode.util.FileUtil;
import org.firstinspires.ftc.teamcode.util.TelemetryHolder;

import java.util.List;

public class Robot extends BaseComponent{
    private static final double VOLTAGE_WARNING_THRESHOLD = 12.0;

    private List<LynxModule> lynxModules;

    // START COMPONENTS
    private DriveTrain driveTrain;
    private Turret turret;
    private Shooter shooter;
    private Transfer transfer;
    private Endoscope endoscope;
    private Intake intake;
    // END COMPONENTS

    private int updateCount;
    private ElapsedTime initTime;
    private ElapsedTime firstUpdateTime;

    private Follower follower;

    public Robot(OpMode opMode){
        this(opMode, false);
    }

    public Robot(OpMode opMode, boolean alliance) {
        super(createRobotContext(opMode, alliance));

        this.lynxModules = hardwareMap.getAll(LynxModule.class);

        // START COMPONENTS
        driveTrain = new DriveTrain(context, this);
        turret = new Turret(context, this);
        shooter = new Shooter(context, this);
        transfer = new Transfer(context, this);
        endoscope = new Endoscope(context, this);
        intake = new Intake(context, this);
        // END COMPONENTS

        addSubComponents(driveTrain, intake, turret, shooter, transfer, endoscope);
    }

    public RobotContext getRobotContext() {
        return context;
    }

    @Override
    public void init() {
        super.init();

        follower = driveTrain.getFollower();

        double voltage = computeBatteryVoltage();
        if (voltage < VOLTAGE_WARNING_THRESHOLD) {
            telemetry.addLine("LOW BATTERY WARNING");
            telemetry.addLine("My battery is low and it's getting dark -Opportunity");
        }

        // Set the caching mode for reading values from Lynx components to manual. This means that when reading values
        // like motor positions, the code will grab all values at once instead of one at a time. It will also keep
        // these values and not update them until a manual call is made to clear the cache. We do this once per loop
        // in the Robot's update method.

        for (LynxModule module : lynxModules) {
            //module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        telemetry.addLine("Robot is initialized");
        telemetry.update();

        initTime = new ElapsedTime();
    }

    @SuppressLint("DefaultLocale")
    @Override
    public void update() {
        if (updateCount == 0) {
            firstUpdateTime = new ElapsedTime();
            onStart();
        }

        // Compute and print the updates per second
        computeUpdatesPerSecond();

        // Clear the bulk cache so that new values will be read for each component
        for (LynxModule lynxModule : lynxModules) {
            lynxModule.clearBulkCache();
        }

        follower.update();

        // Allow all the subcomponents to do their work.
        super.update();

        // Update telemetry once per iteration after all components have been called.
        telemetry.update(ftcTelemetry);
    }

    public void savePositionToDisk() {
        savePositionToDisk("robot-position");
    }

    public void savePositionToDisk(String filename) {

    }

    public void loadPositionFromDisk() {
        loadPositionFromDisk("robot-position");
    }

    public void loadPositionFromDisk(String filename) {
        List<String> lines = FileUtil.readLines(filename);
        if (!lines.isEmpty()) {
            try {
                if (lines.size() != 5) {
                    throw new IllegalArgumentException("Expected 4 lines but found [" + lines.size() + "]");
                }


            } catch (Exception e) {
                telemetry.addData("Error loading position", ErrorUtil.convertToString(e));
            }

            // Now that the position has been consumed, remove the file
            FileUtil.removeFile(filename);
        }
    }

    @SuppressLint("DefaultLocale")
    private void computeUpdatesPerSecond() {
        updateCount++;

        double updatesPerSecond = updateCount / firstUpdateTime.seconds();
        telemetry.addData("Updates / sec", String.format("%.1f", updatesPerSecond));
    }

    public void onStart() {
        ftcTelemetry.clear();
    }

    /**
     * false - red (tag 24)<br>
     * true - blue (tag 20)
     */
    public static RobotContext createRobotContext(OpMode opMode, boolean alliance){
        return new RobotContext(
                opMode,
                new RobotDescriptor(),
                alliance
        );
    }

    public static RobotContext createRobotContext(OpMode opMode){
        return createRobotContext(opMode, true);
    }

    public void waitForCommandsToFinish() {
        waitForCommandsToFinish(Double.MAX_VALUE);
    }

    public void waitForCommandsToFinish(double maxTime) {
        // While the components are busy trying to execute a command, keep looping and giving
        // each of them a chance to update.
        ElapsedTime time = new ElapsedTime();
        while (!isStopRequested() && isBusy() && time.seconds() < maxTime) {
            update();
        }
    }

    public DriveTrain getDriveTrain() {
        return driveTrain;
    }
    public Turret getTurret() {
        return turret;
    }
    public Shooter getShooter() {
        return shooter;
    }
    public Transfer getTransfer(){
        return transfer;
    }
    public Intake getIntake(){
        return intake;
    }
    public Endoscope getEndoscope(){
        return endoscope;
    }
    public TelemetryManager getTelemetry(){
        return telemetry;
    }
    public Pose getPose(){
        return follower.getPose();
    }

    private double computeBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        }
        return result;
    }
}
