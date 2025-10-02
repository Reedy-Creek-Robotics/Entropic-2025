package org.firstinspires.ftc.teamcode.components;

import android.annotation.SuppressLint;


import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AprilTagTest;
import org.firstinspires.ftc.teamcode.util.ErrorUtil;
import org.firstinspires.ftc.teamcode.util.FileUtil;
import org.firstinspires.ftc.teamcode.util.TelemetryHolder;

import java.util.List;

public class Robot extends BaseComponent {
    private static final double VOLTAGE_WARNING_THRESHOLD = 12.0;

    private List<LynxModule> lynxModules;

    private DriveTrain driveTrain;
    private AprilTag aprilTag;

    private int updateCount;
    private ElapsedTime initTime;
    private ElapsedTime firstUpdateTime;

    public Robot(OpMode opMode) {
        super(createRobotContext(opMode));

        this.lynxModules = hardwareMap.getAll(LynxModule.class);

        driveTrain = new DriveTrain(context);
        aprilTag = new AprilTag(context);

        addSubComponents(driveTrain, aprilTag);

        TelemetryHolder.telemetry = telemetry;
    }

    public RobotContext getRobotContext() {
        return context;
    }

    @Override
    public void init() {
        super.init();

        double voltage = computeBatteryVoltage();
        if (voltage < VOLTAGE_WARNING_THRESHOLD) {
            telemetry.log().add("LOW BATTERY WARNING");
            telemetry.log().add("My battery is low and it's getting dark -Opportunity");
        }

        // Set the caching mode for reading values from Lynx components to manual. This means that when reading values
        // like motor positions, the code will grab all values at once instead of one at a time. It will also keep
        // these values and not update them until a manual call is made to clear the cache. We do this once per loop
        // in the Robot's update method.

        for (LynxModule module : lynxModules) {
            //module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        telemetry.log().add("Robot is initialized");
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

        // Allow all the subcomponents to do their work.
        super.update();

        // Update telemetry once per iteration after all components have been called.
        telemetry.update();

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
                telemetry.log().add("Error loading position: " + ErrorUtil.convertToString(e));
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
        // todo: Commenting this out for now since we no longer need to worry about moving the turrent from a diagonal
        // todo: start position to the front.  We may need to revisit this since the turret won't "lock" in place
        // todo: until the slide moves above a specific height.
        //slide.moveToHeight(TRAVEL);

        telemetry.log().clear();
    }

    public static RobotContext createRobotContext(OpMode opMode) {
        return createRobotContext(opMode, -1);
    }
    public static RobotContext createRobotContext(OpMode opMode, int alliance) {
        return new RobotContext(
                opMode,
                new RobotDescriptor(),
                alliance
        );
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
