package org.firstinspires.ftc.teamcode.components;

import android.util.Size;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS.Pose2D;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.util.ArrayUtil;
import org.firstinspires.ftc.teamcode.util.HardwareUtil;
import org.firstinspires.ftc.teamcode.util.LogCatUtil;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Configurable
public class Turret extends BaseComponent{

    LogCatUtil log;
    HardwareUtil hardwareUtil;

    /**
     * Will be appended to the prefix defined in LogCatUtil
     */
    static String logTag = "Turret";

    // Must be at least 360 degrees
    static double maxHeading = 30;
    static double minHeading = -180;

    static double fx = 595.21, fy = 595.21, cx = 984.515, cy = 599.035; //TODO: Fix these
    /**
     * When using OTOS and April Tag Only<br>
     * If the center of the tag is not within +- tagTolerance, then the localizer will use the OTOS instead
     */
    static int tagTolerance = 25;

    /**
     * In rpm
     */
    static int baseMotorSpeed = 1150;
    static double drivePulleyTeeth = 24;
    static double turretPulleyTeeth = 134;

    static double gearRatio = turretPulleyTeeth / drivePulleyTeeth;

    /**In rpm*/
    static int[] speeds          = {30,     43,     60,     84,     117,    223,   312,   435,   1150,  1620,  6000};/**In kg.cm*/
    static double[] torques      = {250.0,  185.0,  133.2,  93.6,   68.4,   38.0,  24.3,  18.7,  7.9,   5.4,   1.5};
    static double[] ticksPerRevs = {5281.1, 3895.9, 2786.2, 1993.6, 1425.1, 751.8, 537.7, 384.5, 145.1, 103.8, 28.0};

    static int motorType = ArrayUtil.findIndexOfItem(speeds, baseMotorSpeed);

    static double baseTicksPerDeg =  ticksPerRevs[motorType] / 360;
    /**
     * In kg.cm
     */
    static double baseMotorTorque = torques[motorType];
    static double baseTicksPerRev = ticksPerRevs[motorType];

    /**
     * Effective speed accounting for the gear ratio <br> Measured in RPM
     */
    static double effectiveSpeed = baseMotorSpeed / gearRatio;
    /**
     * Effective torque accounting for the gear ratio <br> Measured in kg.cm
     */
    static double effectiveTorque = baseMotorSpeed * gearRatio;
    static double effectiveTicksPerRev = /*baseTicksPerRev * gearRatio*/ 824;
    static double effectiveTicksPerDeg = effectiveTicksPerRev / 360;

    //206 ticks per 90 deg
    //824 ticks per 360 deg

    static Pose2D redGoal = new Pose2D(144, 144, 0);
    static Pose2D blueGoal = new Pose2D(144, 0, 0);

    Pose curPos = new Pose();

    /**
     * Configured like the motor, with it's base stats (rpm, tps, torque) <br> points to "turret" hardware map
     */
    DcMotorEx turretMotor;
    Follower follower;

    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, -8.423, 14.97, 0);

    AprilTagProcessor aprilTag;
    VisionPortal visionPortal;

    AprilTagDetection tag;
    List<AprilTagDetection> detectionList;

    Size cameraRes = new Size(1280, 720);

    double previousBearing;

    double toleranceDeg = 5;
    double targetPos = 0;

    Robot robot;

    /**
     * 0 - otos only<br>
     * 1 - tag only - stop if no tag<br>
     * 2 - tag when found - otos when no tag<br>
     * 3 - otos with periodic relocalization from tag
     */
    static int autoAimMethod = 0;

    /**
     * 0 - run to position<br>
     * 1 - rough 'pid' like system
     */
    static int moveMethod = 1;

    /**
     * false - red (tag 24)<br>
     * true - blue (tag 20)
     */
    boolean alliance = false;


    public Turret(RobotContext context, Robot robot) {
        super(context);

        log = new LogCatUtil(logTag);

        hardwareUtil = new HardwareUtil(log, hardwareMap);

        this.alliance = context.alliance;

        //initAprilTag();

        turretMotor = hardwareUtil.getMotorEx("turret");

        this.robot = robot;
    }

    @Override
    public void init() {
        super.init();

        MotorConfigurationType motorConfiguration = turretMotor.getMotorType().clone();
        motorConfiguration.setAchieveableMaxRPMFraction(1.0);
        motorConfiguration.setMaxRPM(baseMotorSpeed);
        motorConfiguration.setTicksPerRev(baseTicksPerRev);

        turretMotor.setMotorType(motorConfiguration);

        turretMotor.setTargetPositionTolerance((int) (baseTicksPerDeg * toleranceDeg));
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        follower = robot.getDriveTrain().getFollower();
    }

    @Override
    public void update(){
        curPos = robot.getPose();
        telemetry.addData("Turret Pose", curPos);

        switch(autoAimMethod){
            case 0:
                otosAutoAim();
                break;
            case 1:
                //tagAutoAim();
                break;
            case 2:
                //tagOtosAutoAim();
                break;
            case 3:
                //otosRelocalizeAutoAim();
                break;
        }

        switch(moveMethod){
            case 0:
                moveRtp();
                break;
            case 1:
                movePid();
                break;
        }

        telemetry.addData("Turret Pos", getPositionTicks());
    }

    private void setTargetDegrees(double degrees){

        while(degrees > maxHeading){
            degrees = maxHeading;
            log.warn("target over max heading");
        }

        while(degrees < minHeading){
            degrees = minHeading;
            log.warn("target under min heading");
        }

        targetPos = degrees * effectiveTicksPerDeg;
    }

    private double getPositionDegrees(){
        return turretMotor.getCurrentPosition() / effectiveTicksPerDeg;
    }

    public int getPositionTicks(){
        return turretMotor.getCurrentPosition();
    }

    public void resetEncoder(){
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void otosAutoAim(){
        // Calculates the theta using the tanh function
        double theta = Math.tanh(((alliance ? blueGoal.y : redGoal.y) - curPos.getY()) / ((alliance ? blueGoal.x : redGoal.x) - curPos.getX()));
        log.debug("theta : " + theta + " | degrees : " + Math.toDegrees(curPos.getHeading() - theta));
        // We subtract the theta from the heading to account for robot rotation.
        setTargetDegrees(Math.toDegrees(curPos.getHeading() - theta));
    }

    private void tagAutoAim(){
        tag = alliance ? getTag20() : getTag24();
        if(tag != null){
            setTargetDegrees(tag.ftcPose.bearing);
        } else{
            log.warn("no tag");
        }
    }

    private void tagOtosAutoAim(){
        tag = alliance ? getTag20() : getTag24();
        // Will use the otos if frame has already been used, or if no tag is found.
        if(tag == null){
            otosAutoAim();
        }else{
            setTargetDegrees(tag.ftcPose.bearing);
        }
    }

    private void otosRelocalizeAutoAim(){
        tag = alliance ? getTag20() : getTag24();
        // Will use the otos if frame has already been used, or if no tag is found.
        if(tag == null){
            otosAutoAim();
            return;
        // Will relocalize the otos if the tag is within a certain range
        }else if(Math.abs(tag.center.x - (cameraRes.getWidth() / 2.0)) <= tagTolerance){
            Pose pos = otosPoseFromTag(tag.robotPose);
            log.info("localize - " + pos);
            follower.setPose(pos);
            log.debug("otos pos" + follower.getPose());
        }
        setTargetDegrees(tag.ftcPose.bearing);
    }

    private void moveRtp(){
        turretMotor.setTargetPosition((int) targetPos);
        turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(0.5);
    }

    private void movePid(){
        telemetry.addData("Setting target", targetPos);
        if (Math.abs(turretMotor.getCurrentPosition() - targetPos) <= 2) {
            turretMotor.setPower(0);
        } else if (Math.abs(turretMotor.getCurrentPosition() - targetPos) <= 30) {
            turretMotor.setPower(turretMotor.getCurrentPosition() < (int) targetPos ? 0.05 : -0.05);
        } else {
            turretMotor.setPower(turretMotor.getCurrentPosition() < (int) targetPos ? 0.2 : -0.2);
        }
        telemetry.addData("Turret power", turretMotor.getPower());
    }

    private void initAprilTag() {

        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.

                .setLensIntrinsics(fx, fy, cx, cy)
                .setCameraPose(cameraPosition, new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 180, 0))
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();


        builder.setCamera(hardwareUtil.getWebcamName("turretCam"));

        // Enable the RC preview (LiveView). Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        builder.setCameraResolution(cameraRes);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);
    }

    private AprilTagDetection getTag(int id){
        detectionList = aprilTag.getDetections();
        if(detectionList.isEmpty()) return null;
        for(AprilTagDetection detection : detectionList){
            if(detection.id == id) return detection;
        }
        return null;
    }

    /**
     * Get tag 20 ONLY if it is found and the frame hasn't already been fetched, otherwise return null
     * @return Returns tag with ID 20 if found, and the frame hasn't been fetched already.
     */
    private AprilTagDetection getTag20(){
        AprilTagDetection tag20 = getTag(20);
        // If the tag is null, return null. Mainly to avoid null pointer exceptions later
        if(tag20 == null) return null;
        /*
        If the bearing of the current image is the same as from the last one, then ignore it as we've already used the frame

        We avoid using the same frame twice because the april tag processor runs slower than the main OpMode loop
        so the turret's position updating would be bottlenecked by the frame rate, forcing lower resolution/higher
        decimation to get decent frame times. This difference in updates means less accurate data.
        */
        if(tag20.ftcPose.bearing == previousBearing){
            previousBearing = tag20.ftcPose.bearing;
            return null;
        }

        previousBearing = tag20.ftcPose.bearing;
        return tag20;
    }

    /**
     * Get tag 24 ONLY if it is found and the frame hasn't already been fetched, otherwise return null
     * @return Returns tag with ID 24 if found, and the frame hasn't been fetched already.
     */
    private AprilTagDetection getTag24(){
        AprilTagDetection tag24 = getTag(24);
        // If the tag is null, return null. Mainly to avoid null pointer exceptions later
        if(tag24 == null) return null;
        telemetry.addData("Bearing", tag24.ftcPose.bearing);

        /*
        If the bearing of the current image is the same as from the last one, then ignore it as we've already used the frame

        We avoid using the same frame twice because the april tag processor runs slower than the main OpMode loop
        so the turret's position updating would be bottlenecked by the frame rate, forcing lower resolution/higher
        decimation to get decent frame times. This slower update rate means that the turret could be facing the
        wrong direction, due to the movement that happened between frames.
        */
        if(tag24.ftcPose.bearing == previousBearing){
            previousBearing = tag24.ftcPose.bearing;
            return null;
        }

        previousBearing = tag24.ftcPose.bearing;
        return tag24;
    }

    private Pose otosPoseFromTag(Pose3D tagPose){
        Pose pose = new Pose(tagPose.getPosition().x, tagPose.getPosition().y, tagPose.getOrientation().getYaw(AngleUnit.RADIANS), InvertedFTCCoordinates.INSTANCE);
        return poseFromFtcPose2d(PoseConverter.poseToPose2D(pose, PedroCoordinates.INSTANCE));
    }

    private Pose poseFromFtcPose2d(org.firstinspires.ftc.robotcore.external.navigation.Pose2D ftcPose2D){
        return new Pose(ftcPose2D.getX(DistanceUnit.INCH) + 76, -ftcPose2D.getY(DistanceUnit.INCH), addRadians(ftcPose2D.getHeading(AngleUnit.RADIANS), Math.PI));
    }

    private double addRadians(double radOne, double radTwo){
        if(radOne + radTwo <=  2 * Math.PI) return radOne + radTwo;
        return (radOne + radTwo) - 2 * Math.PI;
    }
}
