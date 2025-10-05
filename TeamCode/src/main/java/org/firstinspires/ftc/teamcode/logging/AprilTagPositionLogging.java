package org.firstinspires.ftc.teamcode.logging;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.log.DatalogAprilTag;
import java.text.SimpleDateFormat;

@TeleOp(name = "Logging: AprilTag Position", group = "Logging")
public class AprilTagPositionLogging extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    DatalogAprilTag log;

    /***

     */
    @Override
    public void runOpMode() throws InterruptedException {
        initAprilTag();

        // init the file logging
        // add a timestamp on end of filename so each run of op mode gives
        // you a unique file
        String timeStamp = new SimpleDateFormat("yyyyMMdd-HHmmss").format(new java.util.Date());
        log = new DatalogAprilTag("AprilTagTester_" + timeStamp);

        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch START to start OpMode");
        telemetry.addData("Press left-right to select tile corner X position (in ft)","");
        telemetry.addData("Press up-down to select tile corner Y position (in ft)","");
        telemetry.update();
        waitForStart();

        double selectX = 0;
        double selectY = 0;
        boolean loggingEnabled = false;
        while(opModeIsActive()) {
            // controls
            if( gamepad1.dpadDownWasPressed() ) {
                selectY -= 2;
            }

            if( gamepad1.dpadUpWasPressed() ) {
                selectY += 2;
            }

            // start the test
            if( gamepad1.dpadLeftWasPressed() ) {
                selectX -= 2;
            }

            // stop the test
            if( gamepad1.dpadRightWasPressed() ) {
                selectX += 2;
            }

            if( gamepad1.circleWasPressed()) {
                loggingEnabled = !loggingEnabled;
            }

            List<AprilTagDetection> currentDetections = aprilTag.getDetections();
            telemetry.addData("# AprilTags Detected", currentDetections.size());

            for (AprilTagDetection detection : currentDetections) {

                if(detection == null || detection.robotPose == null ||
                        detection.ftcPose == null ||
                        detection.robotPose.getOrientation() == null ||
                        detection.robotPose.getPosition() == null) {
                    continue;
                }


                double aprilTagID = detection.id;
                double poseX = detection.robotPose.getPosition().x;
                double poseY = detection.robotPose.getPosition().y;
                double poseZ = detection.robotPose.getPosition().z;
                double posePitch = detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES);
                double poseRoll = detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES);
                double poseYaw = detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);;
                double rawX = detection.ftcPose.x;
                double rawY = detection.ftcPose.y;
                double rawZ = detection.ftcPose.z;
                double rawPitch = detection.ftcPose.pitch;
                double rawRoll = detection.ftcPose.roll;
                double rawYaw = detection.ftcPose.yaw;
                double poseBearing = detection.ftcPose.bearing;
                //double poseElevation = detection.ftcPose.elevation;

                if (detection.metadata != null) {
                    // log - only when logging is enabled
                    if( loggingEnabled ) {
                        log.selectX.set(selectX);
                        log.selectY.set(selectY);
                        log.ID.set(aprilTagID);
                        log.poseX.set(poseX);
                        log.poseY.set(poseY);
                        log.poseBearing.set(poseBearing);
                        log.poseYaw.set(poseY);
                        log.rawX.set(rawX);
                        log.rawY.set(rawY);
                        log.rawZ.set(rawZ);
                        log.rawPitch.set(rawPitch);
                        log.rawRoll.set(rawRoll);
                        log.rawYaw.set(rawYaw);
                        log.writeLine();
                        telemetry.addData("Logging: ","AprilTagTester_" + timeStamp);
                    }

                    telemetry.addData("  Sel X: ",selectX);
                    telemetry.addData("  Sel Y: ",selectY);

                    telemetry.addData("     ID: ",aprilTagID);
                    telemetry.addData("      X: ",poseX);
                    telemetry.addData("      Y: ",poseY);
                    //telemetry.addData("bearing: ",poseBearing);
                    telemetry.addData("    yaw: ",poseYaw);
                } else {
                    telemetry.addData("Unknown ID: ", aprilTagID);
                    //telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
                }
            }


            // telemetry
            telemetry.update();
        }

        // Clean shutdown
    }

    private void initAprilTag() {

        // Create the AprilTag processor.
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
                //.setLensIntrinsics(1040.34, 1040.34, 929.692, 558.489)
                .setCameraPose(cameraPosition, cameraOrientation)
                .setLensIntrinsics(593.186, 593.186, 985.272, 539.561)
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
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(1920, 1080));


        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);

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

    }   // end method initAprilTag()





}