package org.firstinspires.ftc.teamcode.components;

import android.annotation.SuppressLint;


import androidx.annotation.NonNull;
import androidx.annotation.Nullable;

import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.field.FieldManager;
import com.bylazar.field.PanelsField;
import com.bylazar.field.Style;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.PoseHistory;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.ErrorUtil;
import org.firstinspires.ftc.teamcode.util.FileUtil;
import org.firstinspires.ftc.teamcode.util.LogCatUtil;
import org.firstinspires.ftc.teamcode.util.log.DataLogger;
import org.firstinspires.ftc.teamcode.util.log.MotorDataLog;
import org.firstinspires.ftc.vision.VisionPortal;

import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;

public class Robot extends BaseComponent{
    private static final double VOLTAGE_WARNING_THRESHOLD = 12.0;
    
    MotorDataLog motorDataLog;

    private List<LynxModule> lynxModules;

    // START COMPONENTS
    private DriveTrain driveTrain;
    private Turret turret;
    private DualShooter shooter;
    private UpgradedTransfer transfer;
    private Endoscope endoscope;
    private Lighthouse lighthouse;
    private Intake intake;
    // END COMPONENTS

    private int[] liveViewContainerIds;

    private int updateCount;
    private ElapsedTime initTime;
    private ElapsedTime firstUpdateTime;

    @IgnoreConfigurable
    private static Follower follower;

    private LogCatUtil log;

    private Pose curPose = new Pose();

    double motorCurrent = 0;

    private List<HardwareDevice> devices;
    private List<DcMotorEx> motors;

    @IgnoreConfigurable
    private static PoseHistory poseHistory;


    public Robot(OpMode opMode){
        this(opMode, false);
    }

    public Robot(OpMode opMode, boolean alliance){
        this(opMode, alliance, null);
    }

    public Robot(OpMode opMode, boolean alliance, @Nullable Component... components) {
        super(createRobotContext(opMode, alliance));

        liveViewContainerIds = VisionPortal.makeMultiPortalView(2, VisionPortal.MultiPortalLayout.VERTICAL);

        log = new LogCatUtil("Robot");

        @SuppressLint("SimpleDateFormat") String timeStamp = new SimpleDateFormat("MM\\dd-HH:mm").format(new java.util.Date());
        motorDataLog = new MotorDataLog("MCD-"+timeStamp);

        this.lynxModules = hardwareMap.getAll(LynxModule.class);

        if(components == null) {
            // START COMPONENTS
            driveTrain = new DriveTrain(context, this);
            turret = new Turret(context, this);
            shooter = new DualShooter(context, this);
            transfer = new UpgradedTransfer(context, this);
            endoscope = new Endoscope(context, this, liveViewContainerIds[0]);
            lighthouse = new Lighthouse(context, this, liveViewContainerIds[1]);
            intake = new Intake(context, this);
            // END COMPONENTS

            addSubComponents(driveTrain, turret, shooter, transfer, endoscope, /*lighthouse, */intake);
        }else{
            for(Component component : components){
                if(component instanceof DriveTrain){
                    driveTrain = (DriveTrain) component;
                }else if(component instanceof Turret){
                    turret = (Turret) component;
                }else if(component instanceof DualShooter){
                    shooter = (DualShooter) component;
                }else if(component instanceof UpgradedTransfer){
                    transfer = (UpgradedTransfer) component;
                }else if(component instanceof Endoscope){
                    endoscope = (Endoscope) component;
                }else if(component instanceof Lighthouse){
                    lighthouse = (Lighthouse) component;
                }else if(component instanceof Intake){
                    intake = (Intake) component;
                }
            }

            addSubComponents(components);
        }
    }

    public RobotContext getRobotContext() {
        return context;
    }

    @Override
    public void init() {
        super.init();

        follower = driveTrain.getFollower();
        Drawing.init();

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

//        while(
//                !Thread.currentThread().isInterrupted() ||
//                getTurret().getVisionPortal() == null ||
//                getEndoscope().getVisionPortal() == null ||
//                getTurret().getVisionPortal().getCameraState()    != VisionPortal.CameraState.STREAMING ||
//                getEndoscope().getVisionPortal().getCameraState() != VisionPortal.CameraState.STREAMING)
//        {
//            telemetry.addLine("Waiting on cameras...");
//            telemetry.update();
//        }

        telemetry.addLine("Robot is initialized");
        telemetry.update();

        initTime = new ElapsedTime();

//        devices = new ArrayList<>();
//        motors = hardwareMap.getAll(DcMotorEx.class);
//        for (HardwareDevice device : hardwareMap) {
//            devices.add(device);
//        }
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

        if(useTelemetry){
            poseHistory = follower.getPoseHistory();
            drawCurrentAndHistory();
        }


        // Allow all the subcomponents to do their work.
        super.update();

        // Update telemetry once per iteration after all components have been called.
        telemetry.update(ftcTelemetry);

//        logCurrents(); //Takes a long time to execute, leave commented when not needed
    }
    
    private void logCurrents(){
        motorCurrent = 0;

        for(DataLogger.GenericField field : motorDataLog.fields){
            field.set(-1);
        }

        for(DcMotorEx motor : motors){
            String motorName = hardwareMap.getNamesOf(motor).iterator().next();

            switch (motorName) {
                case "shooter":
                    motorDataLog.shooterCurrent.set(motor.getCurrent(CurrentUnit.AMPS));
                    break;
                case "turret":
                    motorDataLog.turretCurrent.set(motor.getCurrent(CurrentUnit.AMPS));
                    break;
                case "intake":
                    motorDataLog.intakeCurrent.set(motor.getCurrent(CurrentUnit.AMPS));
                    break;
                case "lf":
                    motorDataLog.lfCurrent.set(motor.getCurrent(CurrentUnit.AMPS));
                    break;
                case "lr":
                    motorDataLog.lrCurrent.set(motor.getCurrent(CurrentUnit.AMPS));
                    break;
                case "rf":
                    motorDataLog.rfCurrent.set(motor.getCurrent(CurrentUnit.AMPS));
                    break;
                case "rr":
                    motorDataLog.rrCurrent.set(motor.getCurrent(CurrentUnit.AMPS));
            }
            
            motorCurrent += motor.getCurrent(CurrentUnit.AMPS);
        }

        motorDataLog.batteryVoltage.set(computeBatteryVoltage());
        motorDataLog.totalMotorCurrent.set(motorCurrent);

        motorDataLog.writeLine();
    }

    public Pose getPose(){
        return curPose;
    }

    public double getVelocity(){
        return follower.getVelocity().getMagnitude();
    }
    public double getAngVelocity(){
        return follower.getAngularVelocity();
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
                context.getPattern(),
                context.getBallsClassified(),
//                context.getAlliance()
                turret.getAlliance()
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
                state.posT()
//                state.pattern(),
//                state.balls()
        );
    }

    public void loadStateFromDisk() {
        loadStateFromDisk("robot-state");
    }

    public void loadStateFromDisk(String filename) {
        List<String> lines = FileUtil.readLines(filename);

        // if there is no file, do nothing
        if (lines.isEmpty()) {
            return;
        }

        try {
            if (lines.size() != 5) {
                throw new IllegalArgumentException("Expected 5 lines but found [" + lines.size() + "]");
            }

            this.curPose = new Pose (
                    Double.parseDouble(lines.get(0)),
                    Double.parseDouble(lines.get(1)),
                    Double.parseDouble(lines.get(2)));

            boolean alliance = Boolean.parseBoolean(lines.get(3));
            context.setAlliance(alliance);
            turret.setAlliance(alliance);

            turret.setPositionTicks(Integer.parseInt(lines.get(4)));

//            context.setPattern(Integer.parseInt(lines.get(5)));
//            context.setBallsClassified(Integer.parseInt(lines.get(6)));

        } catch (Exception e) {
            telemetry.addData("Error loading robot state", ErrorUtil.convertToString(e));
            telemetry.addData("File contents", lines.toString());
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
    public DualShooter getShooter() {
        return shooter;
    }
    public UpgradedTransfer getTransfer(){
        return transfer;
    }
    public Intake getIntake(){
        return intake;
    }
    public Endoscope getEndoscope(){
        return endoscope;
    }
    public Lighthouse getLighthouse(){
        return lighthouse;
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

    public static void drawCurrent() {
        try {
            Drawing.drawRobot(follower.getPose());
            Drawing.sendPacket();
        } catch (Exception e) {
            throw new RuntimeException("Drawing failed " + e);
        }
    }

    public static void drawCurrentAndHistory() {
        Drawing.drawPoseHistory(poseHistory);
        drawCurrent();
    }

    public static class RobotState {
        Pose pose;
        /**
         * false - Red<br>
         * true - Blue
         */
        boolean alliance;
        int turretPos;
        /**
         * 0 - N/A<br>
         * 1 - GPP<br>
         * 2 - PGP<br>
         * 3 - PPG<br>
         */
        int pattern;
        /** the number of balls in the classifier*/
        int balls;

        public RobotState(Pose pose, int turretPos, int pattern, int balls, boolean alliance){
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

        public int balls() {
            return balls;
        }
    }
}

class Drawing {
    public static final double ROBOT_RADIUS = 17/2.0; // woah
    private static final FieldManager panelsField = PanelsField.INSTANCE.getField();

    private static final Style robotLook = new Style(
            "", "#3F51B5", 0.75
    );
    private static final Style historyLook = new Style(
            "", "#4CAF50", 0.25
    );

    /**
     * This prepares Panels Field for using Pedro Offsets
     */
    public static void init() {
        panelsField.setOffsets(PanelsField.INSTANCE.getPresets().getPEDRO_PATHING());
    }

    /**
     * This draws everything that will be used in the Follower's telemetryDebug() method. This takes
     * a Follower as an input, so an instance of the DashbaordDrawingHandler class is not needed.
     *
     * @param follower Pedro Follower instance.
     */
    public static void drawDebug(Follower follower) {
        if (follower.getCurrentPath() != null) {
            drawPath(follower.getCurrentPath(), robotLook);
            Pose closestPoint = follower.getPointFromPath(follower.getCurrentPath().getClosestPointTValue());
            drawRobot(new Pose(closestPoint.getX(), closestPoint.getY(), follower.getCurrentPath().getHeadingGoal(follower.getCurrentPath().getClosestPointTValue())), robotLook);
        }
        drawPoseHistory(follower.getPoseHistory(), historyLook);
        drawRobot(follower.getPose(), historyLook);

        sendPacket();
    }

    /**
     * This draws a robot at a specified Pose with a specified
     * look. The heading is represented as a line.
     *
     * @param pose  the Pose to draw the robot at
     * @param style the parameters used to draw the robot with
     */
    public static void drawRobot(Pose pose, Style style) {
        if (pose == null || Double.isNaN(pose.getX()) || Double.isNaN(pose.getY()) || Double.isNaN(pose.getHeading())) {
            return;
        }

        panelsField.setStyle(style);
        panelsField.moveCursor(pose.getX(), pose.getY());
        panelsField.circle(ROBOT_RADIUS);

        Vector v = pose.getHeadingAsUnitVector();
        v.setMagnitude(v.getMagnitude() * ROBOT_RADIUS);
        double x1 = pose.getX() + v.getXComponent() / 2, y1 = pose.getY() + v.getYComponent() / 2;
        double x2 = pose.getX() + v.getXComponent(), y2 = pose.getY() + v.getYComponent();

        panelsField.setStyle(style);
        panelsField.moveCursor(x1, y1);
        panelsField.line(x2, y2);
    }

    /**
     * This draws a robot at a specified Pose. The heading is represented as a line.
     *
     * @param pose the Pose to draw the robot at
     */
    public static void drawRobot(Pose pose) {
        drawRobot(pose, robotLook);
    }

    /**
     * This draws a Path with a specified look.
     *
     * @param path  the Path to draw
     * @param style the parameters used to draw the Path with
     */
    public static void drawPath(Path path, Style style) {
        double[][] points = path.getPanelsDrawingPoints();

        for (int i = 0; i < points[0].length; i++) {
            for (int j = 0; j < points.length; j++) {
                if (Double.isNaN(points[j][i])) {
                    points[j][i] = 0;
                }
            }
        }

        panelsField.setStyle(style);
        panelsField.moveCursor(points[0][0], points[0][1]);
        panelsField.line(points[1][0], points[1][1]);
    }

    /**
     * This draws all the Paths in a PathChain with a
     * specified look.
     *
     * @param pathChain the PathChain to draw
     * @param style     the parameters used to draw the PathChain with
     */
    public static void drawPath(PathChain pathChain, Style style) {
        for (int i = 0; i < pathChain.size(); i++) {
            drawPath(pathChain.getPath(i), style);
        }
    }

    /**
     * This draws the pose history of the robot.
     *
     * @param poseTracker the PoseHistory to get the pose history from
     * @param style       the parameters used to draw the pose history with
     */
    public static void drawPoseHistory(PoseHistory poseTracker, Style style) {
        panelsField.setStyle(style);

        int size = poseTracker.getXPositionsArray().length;
        for (int i = 0; i < size - 1; i++) {

            panelsField.moveCursor(poseTracker.getXPositionsArray()[i], poseTracker.getYPositionsArray()[i]);
            panelsField.line(poseTracker.getXPositionsArray()[i + 1], poseTracker.getYPositionsArray()[i + 1]);
        }
    }

    /**
     * This draws the pose history of the robot.
     *
     * @param poseTracker the PoseHistory to get the pose history from
     */
    public static void drawPoseHistory(PoseHistory poseTracker) {
        drawPoseHistory(poseTracker, historyLook);
    }

    /**
     * This tries to send the current packet to FTControl Panels.
     */
    public static void sendPacket() {
        panelsField.update();
    }
}
