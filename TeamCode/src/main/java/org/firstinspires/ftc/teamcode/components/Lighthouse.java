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

        tagReader = new AprilTagProcessor.Builder()
                //.setLensIntrinsics(fx, fy, cx, cy);
                .setCameraPose(new Position(DistanceUnit.INCH, 5.25, 6.25, 7.5, 0), new YawPitchRollAngles(AngleUnit.DEGREES, 0, 0, 90, 0))
                .build();
        tagReader.setDecimation(0);

        portal = new VisionPortal.Builder()
                .addProcessor(tagReader)
                .setCameraResolution(new Size(1920, 1080))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setCamera(hardwareUtil.getWebcamName("Lighthouse"))
                .build();
    }

    @Override
    public void init(){

    }

    @Override
    public void update(){
        detectionList = tagReader.getDetections();
        if(detectionList.isEmpty()) {return;}
        for(AprilTagDetection detection : detectionList){
            if(
                    ((
                            detection.id == 24 &&
                            robot.getPose().distanceFrom(redTagPose) > maxRelocalizeDistance
                    ) ||
                    (
                            detection.id == 20 &&
                            robot.getPose().distanceFrom(blueTagPose) > maxRelocalizeDistance
                    ))
                            && enableLighthouse
            ) {
                Pose newPose = poseFromPose3d(detection.robotPose).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
                robot.setPose(newPose);
                telemetry.addLine("Re localizing to " + newPose);
            }
        }
    }

//    @Override
//    public void addTelemetry(){
//
//    }

    private Pose poseFromPose3d(Pose3D pose3D){
        return new Pose(
                pose3D.getPosition().toUnit(DistanceUnit.INCH).x,
                pose3D.getPosition().toUnit(DistanceUnit.INCH).y,
                pose3D.getOrientation().getYaw(AngleUnit.RADIANS),
                InvertedFTCCoordinates.INSTANCE
        );
    }

    public void setEnableLighthouse(boolean enableLighthouse) {
        this.enableLighthouse = enableLighthouse;
    }
    public boolean getEnableLighthouse(){
        return enableLighthouse;
    }
}
