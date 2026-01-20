package org.firstinspires.ftc.teamcode.components;

import android.annotation.SuppressLint;


import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.ErrorUtil;
import org.firstinspires.ftc.teamcode.util.FileUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class Robot extends BaseComponent{
    private static final double VOLTAGE_WARNING_THRESHOLD = 12.0;

    private List<LynxModule> lynxModules;

    // START COMPONENTS
    private DriveTrain driveTrain;
    private Turret turret;
    private Shooter shooter;
    private UpgradedTransfer upgradedTransfer;
    private Endoscope endoscope;
    private Intake intake;
    // END COMPONENTS

    private int updateCount;
    private ElapsedTime initTime;
    private ElapsedTime firstUpdateTime;

    private Follower follower;

    private Pose curPose = new Pose();

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
        upgradedTransfer = new UpgradedTransfer(context, this);
        endoscope = new Endoscope(context, this);
        intake = new Intake(context, this);
        // END COMPONENTS

        addSubComponents(driveTrain, intake, upgradedTransfer, turret, shooter, endoscope);
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
        curPose = follower.getPose();

        // Allow all the subcomponents to do their work.
        super.update();

        // Update telemetry once per iteration after all components have been called.
        telemetry.update(ftcTelemetry);
    }

    public Pose getPose(){
        return curPose;
    }

    public boolean getUseTelemetry(){
        return useTelemetry;
    }

    public void setUseTelemetry(boolean useTelemetry){
        this.useTelemetry = useTelemetry;
    }

    public void saveStateToDisk() {
        saveStateToDisk("robot-state");
    }

    public void saveStateToDisk(String filename) {
        saveStateToDisk(filename, new RobotState(
                follower.getPose(),
                turret.getPositionTicks(),
                0, //ToDo get pattern from AMS
                new int[] {}, //ToDo get balls from AMS
                context.alliance
        ));
    }

    public void saveStateToDisk(RobotState state) {
        saveStateToDisk("robot-state", state);
    }

    public void saveStateToDisk(String filename, RobotState state) {
        FileUtil.writeLines(
                filename,
                state.posX(),
                state.posY(),
                state.posH(),
                state.alliance(),
                state.posT(),
                state.pattern(),
                state.balls()
        );
    }

    public void loadStateFromDisk() {
        loadStateFromDisk("robot-state");
    }

    public void loadStateFromDisk(String filename) {
        List<String> lines = FileUtil.readLines(filename);
        if (!lines.isEmpty()) {
            try {
                if (lines.size() != 7) {
                    throw new IllegalArgumentException("Expected 7 lines but found [" + lines.size() + "]");
                }
            } catch (Exception e) {
                telemetry.addData("Error loading robot state", ErrorUtil.convertToString(e));
            }

            double posX = 0;
            double posY = 0;
            double posH = 0;
            boolean alliance = false;
            int posT = 0;
            int pattern = 0;
            int[] balls = {};
            String line = "";
            for(int i = 0; i < lines.size(); i++){
                line = lines.get(i);
                switch(i){
                    case 0:
                        posX = Double.parseDouble(line);
                        break;
                    case 1:
                        posY = Double.parseDouble(line);
                        break;
                    case 2:
                        posH = Double.parseDouble(line);
                        break;
                    case 3:
                        alliance = Boolean.parseBoolean(line);
                        break;
                    case 4:
                        posT = Integer.parseInt(line);
                        break;
//                    case 5:
//                        pattern = Integer.parseInt(line);
//                        break;
//                    case 6:
//                        balls = Arrays.stream(line.split(","))
//                                .mapToInt(Integer::parseInt)
//                                .toArray();
                    }
                }

            follower.setPose(new Pose(
                    posX,
                    posY,
                    posH
            ));
            context.alliance = alliance;
            turret.setPositionTicks(posT);
            //ToDo: Jonathan, add logic here to get pattern and balls into the ball management system
        }

            // Now that the position has been consumed, remove the file
            FileUtil.removeFile(filename);
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
    public UpgradedTransfer getTransfer(){
        return upgradedTransfer;
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
    public void setPose(Pose pose){
        follower.setPose(pose);
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



    public static class RobotState {
        Pose pose;
        boolean alliance;
        int turretPos;
        /**
         * 0 - N/A<br>
         * 1 - GPP<br>
         * 2 - PGP<br>
         * 3 - PPG<br>
         */
        int pattern;
        int[] balls;

        public RobotState(Pose pose, int turretPos, int pattern, int[] balls, boolean alliance){
            this.pose = pose;
            this.alliance = alliance;
            this.turretPos = turretPos;
            this.pattern = pattern;
            this.balls = balls;
        }

        public double posX(){
            return pose.getX();
        }

        public double posY(){
            return pose.getY();
        }

        public double posH(){
            return pose.getHeading();
        }

        public boolean alliance(){
            return alliance;
        }

        public int posT(){
            return turretPos;
        }

        public int pattern(){
            return pattern;
        }

        public int[] balls() {
            return balls;
        }
    }
}
