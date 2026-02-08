package org.firstinspires.ftc.teamcode.components;

import android.util.Size;

import com.google.blocks.ftcrobotcontroller.util.CurrentGame;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.geometry.CoordinateSystem;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.custom.ImageRegion;
import org.firstinspires.ftc.teamcode.custom.PredominantColorProcessor;
import org.firstinspires.ftc.teamcode.util.HardwareUtil;
import org.firstinspires.ftc.teamcode.util.LogCatUtil;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;


public class Lighthouse extends BaseComponent {
    LogCatUtil log;
    HardwareUtil hardwareUtil;
    VisionPortal portal;
    AprilTagProcessor tagReader;
    List<AprilTagDetection> detectionList;

    ElapsedTime relocalizeTimer;

    private static Pose redTagPose = new Pose(125.92371707,128.62272876);
    private static Pose blueTagPose = new Pose(14.63878293, 128.62272876);
    private static double maxRelocalizeDistance = 80;

    boolean enableLighthouse = true;

    static double fx = 1035.48, fy = 1035.48, cx = 933.549, cy = 564.851;


    Robot robot;

    int liveViewContainerId;

    public Lighthouse(RobotContext context, Robot robot, int liveViewContainerId) {
        super(context);
        log = new LogCatUtil("Lighthouse");
        hardwareUtil = new HardwareUtil(log, hardwareMap);
        this.robot = robot;
        this.liveViewContainerId = liveViewContainerId;
    }

    @Override
    public void init(){
        tagReader = new AprilTagProcessor.Builder()
                .setLensIntrinsics(fx, fy, cx, cy)
                /*
                 * Camera axes:
                 * Origin location: Center of the lens
                 * Axes orientation: +x right, +y down, +z forward (from camera's perspective)
                 *
                 * Robot axes (this is typical, but you can define this however you want):
                 * Origin location: Center of the robot at field height
                 * Axes orientation: +x right, +y forward, +z upward
                 *
                 * Position:
                 * If all values are zero (no translation), that implies the camera is at the center of the
                 * robot. Suppose your camera is positioned 5 inches to the left, 7 inches forward, and 12
                 * inches above the ground - you would need to set the position to (-5, 7, 12).
                 *
                 * Orientation:
                 * If all values are zero (no rotation), that implies the camera is pointing straight up. In
                 * most cases, you'll need to set the pitch to -90 degrees (rotation about the x-axis), meaning
                 * the camera is horizontal. Use a yaw of 0 if the camera is pointing forwards, +90 degrees if
                 * it's pointing straight left, -90 degrees for straight right, etc. You can also set the roll
                 * to +/-90 degrees if it's vertical, or 180 degrees if it's upside-down.
                 */
                .setCameraPose(new Position(DistanceUnit.INCH, -2.5, 6.5, 11.75, 0), new YawPitchRollAngles(AngleUnit.DEGREES, 0, -90, 0, 0))
                .build();
        tagReader.setDecimation(0);

        portal = new VisionPortal.Builder()
                .addProcessor(tagReader)
                .setCameraResolution(new Size(1920, 1080))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setCamera(hardwareUtil.getWebcamName("Lighthouse"))
                .setLiveViewContainerId(liveViewContainerId)
                .build();

        relocalizeTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    }

    @Override
    public void update(){
        if(portal.getCameraState() != VisionPortal.CameraState.STREAMING) return;
        detectionList = tagReader.getDetections();
        if(detectionList.isEmpty()) {return;}
        for(AprilTagDetection detection : detectionList){
            if(
                    ((
                            detection.id == 24 //&& //tag id is 24
//                            robot.getPose().distanceFrom(redTagPose) < maxRelocalizeDistance //distance to tag is not over max
                    )
                    ||//or
                    (
                            detection.id == 20 //&& //tag id is 20
//                            robot.getPose().distanceFrom(blueTagPose) < maxRelocalizeDistance //distance to tag is not over max
                    ))
                    &&//and
                            robot.getVelocity() < 3 //velocity is less than 3in/sec
                    &&//and
                            robot.getAngVelocity() < 0.2 //angular velocity is less than 0.2rad/sec
                    &&//and
                            relocalizeTimer.time() > 1000 //it's been at least 5 seconds since last relocalizing
            ) {
                relocalizeTimer.reset();
                Pose newPose = pedroPoseFromFtcPose3d(detection.robotPose);
                log.debug("Pedro Pose: " + newPose);
                if(enableLighthouse) robot.setPose(newPose);
                telemetry.addLine("Re localizing to " + newPose);
            }
        }
    }

    private Pose pedroPoseFromFtcPose3d(Pose3D robotPose) {
        log.debug("April Tag Pose3D: " + robotPose.toString());
        Pose ftcPose = poseFromPose3d(robotPose);
        double convertedX;
        double convertedY;
//        double convertedH;

        convertedX = ftcPose.getY() + 72;
        convertedY = -ftcPose.getX() + 72;
//        convertedH = ftcPose.getHeading();

        return new Pose(convertedX, convertedY, robot.getPose().getHeading());
    }

    @Override
    public void addTelemetry() {
        telemetry.addData("Velocity Magnitude", robot.getVelocity());
        telemetry.addData("Lighthouse Enable", enableLighthouse);
    }

    //    @Override
//    public void addTelemetry(){
//
//    }

    private Pose poseFromPose3d(Pose3D pose3D){
        Pose newPose = new Pose(
                pose3D.getPosition().toUnit(DistanceUnit.INCH).x,
                pose3D.getPosition().toUnit(DistanceUnit.INCH).y,
                pose3D.getOrientation().getYaw(AngleUnit.RADIANS),
                InvertedFTCCoordinates.INSTANCE
        );
        return newPose;
    }

    public void setEnableLighthouse(boolean enableLighthouse) {
        this.enableLighthouse = enableLighthouse;
    }
    public boolean getEnableLighthouse(){
        return enableLighthouse;
    }
}
