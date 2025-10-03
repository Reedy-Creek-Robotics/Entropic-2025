package org.firstinspires.ftc.teamcode.components;

import android.annotation.SuppressLint;
import android.util.Size;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

@SuppressLint("DefaultLocale")
public class AprilTag extends BaseComponent {

    AprilTagProcessor aprilProcessor;
    VisionPortal portal;
    List<Integer> goals = Arrays.asList(20, 24);
    List<Integer> homeGoal = Collections.singletonList(20);

    public AprilTagDetection nullDetection = new AprilTagDetection(
            -1,
            -1,
            -1,
            null,
            null,
            null,
            null,
            null,
            null,
            -1
    );

    Position cameraPosition = new Position(
            DistanceUnit.INCH,
            -0.25,
            5.25,
            8.1875,
            0
    );

    YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(
            AngleUnit.DEGREES,
            0,
            -90,
            0,
            0
    );

    public AprilTag(RobotContext context){
        super(context);

        if(context.getAlliance() == 1) {homeGoal = Collections.singletonList(24);}

        aprilProcessor = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                .setDrawAxes(true)
                .setDrawTagOutline(true)
                .setDrawTagID(true)
                .setLensIntrinsics(595.21, 595.21, 984.515, 599.035)
                .setCameraPose(cameraPosition, cameraOrientation)
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                .build();

        aprilProcessor.setDecimation(3);

        // Build the Vision Portal, using the above settings.
        portal = new VisionPortal.Builder()
                .setCameraResolution(new Size(1920, 1200))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam"))
                .setAutoStopLiveView(true)
                .addProcessor(aprilProcessor)
                .build();
    }

    /**
     * Set the decimation of the AprilTagProcessor
     * @param decimation Decimation level to set
     */
    public void setDecimation(int decimation){
        aprilProcessor.setDecimation(decimation);
    }

    /**
     *
     * @param id Tag ID to get pose from
     * @return Pose3D of robot pose, null if the tag is not found.
     */
    public Pose3D getRobotPose(int id){
        return getDetection(id).robotPose;
    }

    /**
     * Get a filtered list of April Tag Detections from a provided list of detections
     *
     * @param detections List of April Tag Detections. Can be obtained from processor.getDetections();
     * @param ids List of ids to filter to, any ids not in this list will be filtered. Ex: Arrays.asList(int, int, int)
     * @return Returns a filtered list of detections
     */
    private List<AprilTagDetection> getFilteredDetections(List<AprilTagDetection> detections, List<Integer> ids){
        detections.removeIf(detection -> !ids.contains(detection.id));
        return detections;
    }

    /**
     * Get a filtered list of April Tag Detections from a list of fresh detections from processor.getDetections();
     * Will only contain id's 20 and 24, if either are found
     * @return Returns a filtered list of detections
     */
    private List<AprilTagDetection> getFilteredDetections(){
        return getFilteredDetections(aprilProcessor.getDetections(), goals);
    }

    private List<AprilTagDetection> getFilteredDetections(List<Integer> ids){
        return getFilteredDetections(aprilProcessor.getDetections(), ids);
    }

    /**
     * Get a specific detection by id from a list of detections
     * @param detections List of April Tag Detections. Can be obtained from processor.getDetections();
     * @param id The id to search for
     * @return Returns the AprilTagDetection with the id provided if found, otherwise return null.
     */
    public AprilTagDetection getDetection(List<AprilTagDetection> detections, int id){
        for (AprilTagDetection detection : detections){
            if(detection.id == id) {
                return detection;
            }
        }
        return nullDetection;
    }

    /**
     * Get a specific detection by id from a fresh list of detections
     * @param id The id to search for
     * @return Returns the AprilTagDetection with the id provided if found, otherwise return null.
     */
    public AprilTagDetection getDetection(int id){
        return getDetection(aprilProcessor.getDetections(), id);
    }

    @Override
    public void update() {
        List<AprilTagDetection> currentDetections = getFilteredDetections(goals);

        if (currentDetections.isEmpty()) {
            return;
        }
        for (AprilTagDetection detection : currentDetections) {
            telemetry.addLine(String.format("%d: XY %6.1f %6.1f", detection.id, detection.ftcPose.x, detection.ftcPose.y));
            telemetry.addLine(String.format("%d: Yaw %6.1f", detection.id, detection.ftcPose.yaw));
            telemetry.addLine(String.format("%d: RB %6.1f %6.1f", detection.id, detection.ftcPose.range, detection.ftcPose.bearing));
        }
    }

    private Pose toPose(SparkFunOTOS.Pose2D pose2D){
        return new Pose(pose2D.x, pose2D.y, pose2D.h);
    }

    private SparkFunOTOS.Pose2D toPose2D(Pose pose){
        return new SparkFunOTOS.Pose2D(pose.getX(), pose.getY(), pose.getHeading());
    }
}