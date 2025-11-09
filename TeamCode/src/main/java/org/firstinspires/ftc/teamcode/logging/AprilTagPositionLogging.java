package org.firstinspires.ftc.teamcode.logging;

// test comment for commit

import android.util.Size;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.game.Controller;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

import org.firstinspires.ftc.teamcode.log.DatalogAprilTag;

import java.text.SimpleDateFormat;

import static org.firstinspires.ftc.teamcode.game.Controller.Button.*;

@TeleOp(name = "Logging: AprilTag Position", group = "Logging")
public class AprilTagPositionLogging extends LinearOpMode {
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private DatalogAprilTag log;

    private Controller controller;

    /**
     * The position of the current measurement in the sequence.
     */
    private int position = 0;

    private static class MeasurementPosition {
        public double x;
        public double y;
        public String angle;
        public boolean wasMeasured;

        public MeasurementPosition(double x, double y, String angle) {
            this.x = x;
            this.y = y;
            this.angle = angle;
            this.wasMeasured = false;
        }
    }

    private static class Measurement {
        public double aprilTagID;
        public double poseX;
        public double poseY;
        public double poseZ;
        public double posePitch;
        public double poseRoll;
        public double poseYaw;
        public double rawX;
        public double rawY;
        public double rawZ;
        public double rawPitch;
        public double rawRoll;
        public double rawYaw;
        public double poseBearing;

        public Measurement(AprilTagDetection detection) {
            aprilTagID = detection.id;
            poseX = detection.robotPose.getPosition().x;
            poseY = detection.robotPose.getPosition().y;
            poseZ = detection.robotPose.getPosition().z;
            posePitch = detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES);
            poseRoll = detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES);
            poseYaw = detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);
            rawX = detection.ftcPose.x;
            rawY = detection.ftcPose.y;
            rawZ = detection.ftcPose.z;
            rawPitch = detection.ftcPose.pitch;
            rawRoll = detection.ftcPose.roll;
            rawYaw = detection.ftcPose.yaw;
            poseBearing = detection.ftcPose.bearing;
        }

        public void log(MeasurementPosition position, DatalogAprilTag log, double decimationValue) {
            log.selectX.set(position.x);
            log.selectY.set(position.y);
            log.selectAngle.set(position.angle);
            log.ID.set(aprilTagID);
            log.poseX.set(poseX);
            log.poseY.set(poseY);
            log.poseBearing.set(poseBearing);
            log.poseYaw.set(poseYaw);
            log.rawX.set(rawX);
            log.rawY.set(rawY);
            log.rawZ.set(rawZ);
            log.rawPitch.set(rawPitch);
            log.rawRoll.set(rawRoll);
            log.rawYaw.set(rawYaw);
            log.decimation.set(decimationValue);
            log.writeLine();
        }
    }

    /**
     * Generate the list of measurements that are to be taken.
     */
    private static List<MeasurementPosition> generateMeasurementPositions() {
        // Loop over the field, snaking back and forth, left to right, top to bottom
        List<MeasurementPosition> positions = new ArrayList<>();
        for (int index = 0; index <= 24; index++) {
            int row = index / 5;
            int y = 2 * (4 - row - 2);
            int x;
            if (row % 2 == 0) {
                x = 2 * ((index % 5) - 2);
            } else {
                x = 2 * (4 - (index % 5) - 2);
            }

            positions.add(new MeasurementPosition(x, y, "z"));
            positions.add(new MeasurementPosition(x, y, "r"));
            positions.add(new MeasurementPosition(x, y, "b"));
        }
        return positions;
    }
    /***

     */

    private int findPosition(List<MeasurementPosition> positions, double x, double y, String heading) {
        // returns index of a specific position within the list because the snaking motion is confusing
        for (int index = 0; index <= positions.size(); index++) {
            MeasurementPosition point = positions.get(index);
            if (point.x == x && point.y == y && point.angle.equals(heading)) {
                return index;
            }
        }
        return -1;
    }

    private void displayField(MeasurementPosition location) {
        /*
        prints a 5 * 5 grid displaying the current measurement position with in the field
        Sample Output:
        * * * * *
        * * * * *
        * R * * *
        * * * * *
        * * * * *
         */
        for (int X = -4; X <= 4; X += 2) {
            StringBuilder line = new StringBuilder();
            for (int Y = -4; Y <= 4; Y += 2) {
                if (location.x == X && location.y == Y) {
                    line.append(" ").append(location.angle).append(" ");
                } else {
                    line.append(" * ");
                }
            }
            telemetry.addData(line.toString(),"");
        }
    }

    private int adjustPosition(Controller controller, List<MeasurementPosition> positions, int positionIndex) {
        MeasurementPosition currentPosition = positions.get(positionIndex);

        // use trigger buttons to cycle between positions
        if (controller.isPressed(LEFT_BUMPER)) {
            return Math.max(0, positionIndex - 1);
        } else if (controller.isPressed(RIGHT_BUMPER)) {
            return Math.min(positions.size() - 1, positionIndex + 1);
        }

        // use dpad to move around field
        if (controller.isPressed(DPAD_UP)) {
            // move tile up if possible
            return findPosition(positions,
                    currentPosition.x,
                    Math.min(4, currentPosition.y + 2),
                    currentPosition.angle);
        } else if (controller.isPressed(DPAD_DOWN)) {
            // move tile down if possible
            return findPosition(positions,
                    currentPosition.x,
                    Math.max(-4, currentPosition.y - 2),
                    currentPosition.angle);
        } else if (controller.isPressed(DPAD_LEFT)) {
            // move tile left if possible
            return findPosition(positions,
                    Math.max(-4, currentPosition.x - 2),
                    currentPosition.y,
                    currentPosition.angle);
        } else if (controller.isPressed(DPAD_RIGHT)) {
            // move tile right if possible
            return findPosition(positions,
                    Math.min(4, currentPosition.x + 2),
                    currentPosition.y,
                    currentPosition.angle);
        }

        return positionIndex; // return original position if no buttons are pressed
    }

    @Override
    public void runOpMode() throws InterruptedException {
        float decimationValue = 1;
        initAprilTag(decimationValue);

        this.controller = new Controller(gamepad1);

        // init the file logging
        // add a timestamp on end of filename so each run of op mode gives
        // you a unique file
        String timeStamp = new SimpleDateFormat("yyyyMMdd-HHmmss").format(new java.util.Date());
        log = new DatalogAprilTag("AprilTagTester_" + timeStamp);

        telemetry.addData("This OpMode will guide you through AprilTag error measurement", "");
        telemetry.addData("You will be asked to move the camera to different positions on the field, and different angles", "");
        telemetry.addData("Press the back buttons to move between positions", "");
        telemetry.addData("Alternatively, use the DPAD to manually move through field", "");
        telemetry.addData("Press X to take a measurement", "");
        telemetry.update();

        waitForStart();

        int positionIndex = 0;
        List<MeasurementPosition> positions = generateMeasurementPositions();

        int measurementsToLogFromEachPosition = 10;
        int measurementsRemaining = 0;

        while (opModeIsActive()) {
            // Show on telemetry the current target position
            MeasurementPosition currentPosition = positions.get(positionIndex);

            // update current position based on user input
            positionIndex = adjustPosition(this.controller, positions, positionIndex);

            // Show the target position on telemetry
            telemetry.addLine().addData("Position",  "%d of %d", positionIndex + 1, positions.size());
            telemetry.addData("  Sel X: ", currentPosition.x);
            telemetry.addData("  Sel Y: ", currentPosition.y);
            telemetry.addLine().addData("Sel ang: ", "%s", currentPosition.angle);
            telemetry.addData("Measurement captured from this position", currentPosition.wasMeasured);
            displayField(positions.get(positionIndex));

            // When the user presses X, enable logging for the next N measurements
            if (controller.isPressed(CROSS) && measurementsRemaining == 0) {
                measurementsRemaining = measurementsToLogFromEachPosition;
            }

            // Detect AprilTags
            List<AprilTagDetection> detections = aprilTag.getDetections();
            telemetry.addData("# AprilTags Detected", detections.size());

            boolean measurmentCaptured = false; // stores if any april tags were successfully captured this run through
            for (AprilTagDetection detection : detections) {
                // Make sure it's a valid detection, sometimes these come back as null intermittently
                if (!isValidDetection(detection)) {
                    continue;
                }

                if (detection.metadata == null) {
                    telemetry.addData("Unknown ID: ", detection.id);
                    continue;
                }

                if (measurementsRemaining > 0) {
                    measurmentCaptured = true;

                    // If we are logging measurements, write this one to the log
                    Measurement measurement = new Measurement(detection);
                    measurement.log(currentPosition, log, decimationValue);
                    telemetry.addData("Logging Measurements", measurementsRemaining);
                }

                // display april tag to telemetry
                telemetry.addData("ID", detection.id);
                telemetry.addData("Pose X (in)", "%.2f", detection.robotPose.getPosition().x);
                telemetry.addData("Pose Y (in)", "%.2f", detection.robotPose.getPosition().y);
                telemetry.addData("Yaw (deg)", "%.2f", detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES));
            }
            if (measurmentCaptured) { // if any april tags were measured, decrement the counter
                measurementsRemaining--;

                // If we have finished logging measurements for this position, advance to the next position
                if (measurementsRemaining == 0) {
                    positions.get(positionIndex).wasMeasured = true;
                    positionIndex = Math.min(positions.size() - 1, positionIndex + 1);
                }
            }


            // telemetry
            telemetry.update();
        }

        // Clean shutdown
    }

    private static boolean isValidDetection(AprilTagDetection detection) {
        return detection != null &&
                detection.robotPose != null &&
                detection.ftcPose != null &&
                detection.robotPose.getOrientation() != null &&
                detection.robotPose.getPosition() != null;
    }

    private void initAprilTag(float decimationValue) {

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
        aprilTag.setDecimation(decimationValue);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(1920, 1200));


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
