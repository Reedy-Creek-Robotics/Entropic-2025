package org.firstinspires.ftc.teamcode.components;

import android.util.Size;

import com.google.blocks.ftcrobotcontroller.util.CurrentGame;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.geometry.CoordinateSystem;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.Servo;

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

    private static Pose redTagPose = new Pose(125.92371707,128.62272876);
    private static Pose blueTagPose = new Pose(14.63878293, 128.62272876);
    private static double maxRelocalizeDistance = 80;

    boolean enableLighthouse = true;

    Robot robot;

    public Lighthouse(RobotContext context, Robot robot) {
        super(context);
        log = new LogCatUtil("Lighthouse");
        hardwareUtil = new HardwareUtil(log, hardwareMap);
        this.robot = robot;
    }

    @Override
    public void init(){
        tagReader = new AprilTagProcessor.Builder()
                //.setLensIntrinsics(fx, fy, cx, cy);
                .setCameraPose(new Position(DistanceUnit.INCH, 5.25, 6.25, 7.5, 0), new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 270, 0))
                .build();
        tagReader.setDecimation(0);

        portal = new VisionPortal.Builder()
                .addProcessor(tagReader)
                .setCameraResolution(new Size(1920, 1080))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setCamera(hardwareUtil.getWebcamName("Lighthouse"))
//                .setLiveViewContainerId(1)
                .build();
    }

    @Override
    public void update(){
        if(portal.getCameraState() != VisionPortal.CameraState.STREAMING) return;
        detectionList = tagReader.getDetections();
        if(detectionList.isEmpty()) {return;}
        for(AprilTagDetection detection : detectionList){
            if(
                    ((
                            detection.id == 24 && //tag id is 24
                            robot.getPose().distanceFrom(redTagPose) < maxRelocalizeDistance //distance to tag is not over max
                    )
                    ||//or
                    (
                            detection.id == 20 && //tag id is 20
                            robot.getPose().distanceFrom(blueTagPose) < maxRelocalizeDistance //distance to tag is not over max
                    ))
                    &&//and
                            robot.getVelocity() < 3 //velocity is less than 3in/sec
                    &&//and
                            robot.getAngVelocity() < 0.2 //angular velocity is less than 0.2rad/sec
                    &&//and
                            enableLighthouse //is true
            ) {
                Pose newPose = PedroCoordinates.INSTANCE.convertToPedro(poseFromPose3d(detection.robotPose));
                log.debug("Pedro Pose: " + newPose);
                robot.setPose(newPose);
                telemetry.addLine("Re localizing to " + newPose);
            }
        }
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
        log.debug("April Tag Pose3D: " + pose3D.toString());
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
