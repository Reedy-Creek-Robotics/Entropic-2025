package org.firstinspires.ftc.teamcode;

import android.annotation.SuppressLint;
import android.util.Size;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS.Pose2D;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp
public class AprilTagTest extends LinearOpMode {

    /*Pose2D offset = new Pose2D(
            0.875,
            0.125,
            0
    );*/

    Position offset = new Position(
            DistanceUnit.INCH,
            .875,
            .125,
            0,
            0
    );

    Pose2D blueGoal = new SparkFunOTOS.Pose2D(128.623, 14.639, Math.toRadians(-54));
    Pose2D redGoal = new SparkFunOTOS.Pose2D(128.623, 125.924, Math.toRadians(54));
    /*
    From top down view, assuming front of robot is at top of image:
    x offset is the vertical distance between the camera and fixed point
    y offset is the lateral distance between the camera and fixed point
    heading is not used

    INCHES
     */

    SparkFunOTOS otos;
    AprilTagProcessor aprilBlue;
    AprilTagProcessor aprilRed;
    VisionPortal portal;

    @Override
    public void runOpMode() throws InterruptedException {
        otos = hardwareMap.get(SparkFunOTOS.class, "otos");
        initAprilTag();

        waitForStart();

        while(opModeIsActive()){
            updatePoseAprilTag();
        }
    }

    @SuppressLint("DefaultLocale")
    public void updatePoseAprilTag(){
        Pose2D calculated = new SparkFunOTOS.Pose2D();

        List<AprilTagDetection> currentDetections = aprilBlue.getDetections();
        // Remove all aprilBlue tags that are not 20 (blue goal) or 24 (red goal)
        currentDetections.removeIf(detection -> detection.id != 20 && detection.id != 24);
        if(currentDetections.isEmpty()) {return;}
        for(AprilTagDetection detection : currentDetections){

            boolean blueTag = true;

            telemetry.addLine(String.format("%d: XYH %6.1f %6.1f %6.1f", detection.id, detection.ftcPose.y, detection.ftcPose.x, detection.ftcPose.yaw));
            telemetry.addLine(String.format("adj %d: XYH %6.1f %6.1f %6.1f", detection.id, detection.ftcPose.y, detection.ftcPose.x, detection.ftcPose.yaw));



            Pose2D pos = new Pose2D();
            blueTag = detection.id == 20;

            ElapsedTime timer = new ElapsedTime();
            portal.setProcessorEnabled(blueTag ? aprilBlue : aprilRed, false);
            pos = blueTag ? blueGoal : redGoal;
            telemetry.addData("Time ms", timer.milliseconds());

            if(calculated.equals(new Pose2D())) {
                calculated = new SparkFunOTOS.Pose2D(
                        pos.y - detection.ftcPose.y,
                        pos.x - detection.ftcPose.x,
                        pos.h - detection.ftcPose.yaw);
            }else{
                calculated = toPose2D(
                        toPose(calculated).plus(new Pose(
                                pos.y - detection.ftcPose.y,
                                pos.x - detection.ftcPose.x,
                                pos.h - detection.ftcPose.yaw
                        )).div(2)
                );
            }

            portal.setProcessorEnabled(blueTag ? aprilBlue : aprilRed, false);
        }

        calculated = new Pose2D(calculated.x, calculated.y, calculated.h);

        //resetOTOS();
        //otos.setPosition(calculated);

        telemetry.addLine(String.format("Calc: XYH %6.1f %6.1f %6.1f", calculated.x, calculated.y, calculated.h));
        telemetry.update();
    }

    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilBlue = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                .setDrawAxes(true)
                //.setDrawCubeProjection(true)
                .setDrawTagOutline(true)
                .setDrawTagID(false)
                .setCameraPose(offset, new YawPitchRollAngles(AngleUnit.DEGREES,54,0,0,0))
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCurrentGameTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                //ToDo Fix this
                .setSuppressCalibrationWarnings(true)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        aprilRed = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                .setDrawAxes(true)
                .setDrawTagOutline(true)
                .setLensIntrinsics(595.21, 595.21, 984.515, 599.035)
                .setDrawTagID(false)
                .setCameraPose(offset, new YawPitchRollAngles(AngleUnit.DEGREES,-54,0,0,0))

                //ToDo Fix this
                .setSuppressCalibrationWarnings(true)

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

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder()
                .setCameraResolution(new Size(1920, 1080))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG);

        // Set the camera (webcam vs. built-in RC phone camera).
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam"));

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
        builder.addProcessor(aprilBlue);
        builder.addProcessor(aprilRed);

        // Build the Vision Portal, using the above settings.
        portal = builder.build();

        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }   // end method initAprilTag()

    /**
     * This resets the OTOS.
     */
    public void resetOTOS() {
        otos.resetTracking();
    }

    private Pose toPose(Pose2D pose2D){
        return new Pose(pose2D.x, pose2D.y, pose2D.h);
    }

    private Pose2D toPose2D(Pose pose){
        return new Pose2D(pose.getX(), pose.getY(), pose.getHeading());
    }
}
