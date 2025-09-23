package org.firstinspires.ftc.teamcode.pedroPathing;

import android.util.Size;

import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS.Pose2D;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.pedropathing.localization.Localizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/**
 * This is the otosAprilTagLocalizer class. This class extends the Localizer superclass and is a
 * localizer that uses the SparkFun OTOS.
 *
 * @author Anyi Lin - 10158 Scott's Bots
 * @version 1.0, 7/20/2024
 */
public class otosAprilTagLocalizer implements Localizer {
    private Pose startPose;
    private final SparkFunOTOS otos;
    private AprilTagProcessor april;
    private VisionPortal portal;
    private SparkFunOTOS.Pose2D otosPose;
    private SparkFunOTOS.Pose2D otosVel;
    private SparkFunOTOS.Pose2D otosAcc;
    private double previousHeading;
    private double totalHeading;

    /**
     * This creates a new otosAprilTagLocalizer from a HardwareMap, with a starting Pose at (0,0)
     * facing 0 heading.
     *
     * @param map the HardwareMap
     */
    public otosAprilTagLocalizer(HardwareMap map, otosAprilTagConstants constants) {
        this(map, constants, new Pose());
    }

    /**
     * This creates a new otosAprilTagLocalizer from a HardwareMap and a Pose, with the Pose
     * specifying the starting pose of the localizer.
     *
     * @param map the HardwareMap
     * @param setStartPose the Pose to start from
     */

    public otosAprilTagLocalizer(HardwareMap map, otosAprilTagConstants constants, Pose setStartPose) {

        otos = map.get(SparkFunOTOS.class, constants.otosHardwareMapName);

        otos.setLinearUnit(constants.linearUnit);
        otos.setAngularUnit(constants.angleUnit);
        otos.setOffset(constants.offset);
        otos.setLinearScalar(constants.linearScalar);
        otos.setAngularScalar(constants.angularScalar);

        otos.calibrateImu();
        otos.resetTracking();

        setStartPose(setStartPose);

        otosPose = new SparkFunOTOS.Pose2D();
        otosVel = new SparkFunOTOS.Pose2D();
        otosAcc = new SparkFunOTOS.Pose2D();
        totalHeading = 0;
        previousHeading = startPose.getHeading();

        resetOTOS();

        initAprilTag(map);
    }

    private void initAprilTag(HardwareMap map) {

        // Create the AprilTag processor.
        april = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(true)
                .setDrawCubeProjection(true)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder()
                .setCameraResolution(new Size(1920, 1080));

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(map.get(WebcamName.class, "Webcam 1"));

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(april);

        // Build the Vision Portal, using the above settings.
        portal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()



    /**
     * This returns the current pose estimate.
     *
     * @return returns the current pose estimate as a Pose
     */
    @Override
    public Pose getPose() {
        Pose pose = new Pose(otosPose.x, otosPose.y, otosPose.h);

        Vector vec = pose.getAsVector();
        vec.rotateVector(startPose.getHeading());

        return startPose.plus(new Pose(vec.getXComponent(), vec.getYComponent(), pose.getHeading()));
    }

    /**
     * This returns the current velocity estimate.
     *
     * @return returns the current velocity estimate as a Pose
     */
    @Override
    public Pose getVelocity() {
        return new Pose(otosVel.x, otosVel.y, otosVel.h);
    }

    /**
     * This returns the current velocity estimate.
     *
     * @return returns the current velocity estimate as a Vector
     */
    @Override
    public Vector getVelocityVector() {
        return getVelocity().getAsVector();
    }

    /**
     * This sets the start pose. Changing the start pose should move the robot as if all its
     * previous movements were displacing it from its new start pose.
     *
     * @param setStart the new start pose
     */
    @Override
    public void setStartPose(Pose setStart) {
        startPose = setStart;
    }

    /**
     * This sets the current pose estimate. Changing this should just change the robot's current
     * pose estimate, not anything to do with the start pose.
     *
     * @param setPose the new current pose estimate
     */
    @Override
    public void setPose(Pose setPose) {
        resetOTOS();
        Pose setOTOSPose = setPose.minus(startPose);
        otos.setPosition(new SparkFunOTOS.Pose2D(setOTOSPose.getX(), setOTOSPose.getY(), setOTOSPose.getHeading()));
    }

    public void updatePoseAprilTag(Pose newPose){
        Pose2D blueGoal = new Pose2D(0, 0, Math.toRadians(0));
        Pose2D redGoal = new Pose2D(0, 0, Math.toRadians(0));

        List<AprilTagDetection> currentDetections = april.getDetections();
        // Remove all april tags that are not 20 (blue goal) or 24 (red goal)
        currentDetections.removeIf(detection -> detection.id != 20 && detection.id != 24);
        for(AprilTagDetection detection : currentDetections){
            Pose2D pos = blueGoal;
            if(detection.id == 24)
                pos = redGoal;

            Pose2D calculated = new Pose2D(
                    pos.x - detection.ftcPose.x,
                    pos.y - detection.ftcPose.y,
                    detection.ftcPose.yaw);
        }

        resetOTOS();
        otos.setPosition(new SparkFunOTOS.Pose2D(newPose.getX(), newPose.getY(), newPose.getHeading()));
    }

    /**
     * This updates the total heading of the robot. The OTOS handles all other updates itself.
     */
    @Override
    public void update() {
        otos.getPosVelAcc(otosPose,otosVel,otosAcc);
        totalHeading += MathFunctions.getSmallestAngleDifference(otosPose.h, previousHeading);
        previousHeading = otosPose.h;
    }

    /**
     * This resets the OTOS.
     */
    public void resetOTOS() {
        otos.resetTracking();
    }

    /**
     * This returns how far the robot has turned in radians, in a number not clamped between 0 and
     * 2 * pi radians. This is used for some tuning things and nothing actually within the following.
     *
     * @return returns how far the robot has turned in total, in radians.
     */
    public double getTotalHeading() {
        return totalHeading;
    }

    /**
     * This returns the multiplier applied to forward movement measurement to convert from OTOS
     * ticks to inches. For the OTOS, this value is the same as the lateral multiplier.
     * This is found empirically through a tuner.
     *
     * @return returns the forward ticks to inches multiplier
     */
    public double getForwardMultiplier() {
        return otos.getLinearScalar();
    }

    /**
     * This returns the multiplier applied to lateral/strafe movement measurement to convert from
     * OTOS ticks to inches. For the OTOS, this value is the same as the forward multiplier.
     * This is found empirically through a tuner.
     *
     * @return returns the lateral/strafe ticks to inches multiplier
     */
    public double getLateralMultiplier() {
        return otos.getLinearScalar();
    }

    /**
     * This returns the multiplier applied to turning movement measurement to convert from OTOS ticks
     * to radians. This is found empirically through a tuner.
     *
     * @return returns the turning ticks to radians multiplier
     */
    public double getTurningMultiplier() {
        return otos.getAngularScalar();
    }

    /**
     * This does nothing since this localizer does not use the IMU.
     */
    public void resetIMU() {
    }

    @Override
    public double getIMUHeading() {
        return Double.NaN;
    }

    /**
     * This returns whether if any component of robot's position is NaN.
     *
     * @return returns whether the robot's position is NaN
     */
    public boolean isNAN() {
        return Double.isNaN(getPose().getX()) || Double.isNaN(getPose().getY()) || Double.isNaN(getPose().getHeading());
    }
}