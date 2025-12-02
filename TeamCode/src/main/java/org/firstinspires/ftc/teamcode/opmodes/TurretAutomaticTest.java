package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.game.Controller;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous
public class TurretAutomaticTest extends OpMode {

    static double ticksPerRev = 145.1;
    static int baseMotorSpeed = 1150;
    static double drivePulleyTeeth = 24;
    static double turretPulleyTeeth = 134;

    static double gearRatio = turretPulleyTeeth / drivePulleyTeeth;

    double effectiveTicksPerRev = 145.1 * gearRatio;


    DcMotorEx turret;
    Controller controller;

    boolean move = false;

    // CAMERA

    int decimation = 1;

    static double fx = 595.21, fy = 595.21, cx = 984.515, cy = 599.035;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    List<AprilTagDetection> detectionList;
    AprilTagDetection tag;

    double bearing;
    double centerX;

    boolean useCenterX;

    @Override
    public void init() {
        initAprilTag();
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        controller = new Controller(gamepad1);
    }

    @Override
    public void loop() {

        if(controller.isPressed(Controller.Button.LEFT_STICK_BUTTON)){
            move = !move;
            turret.setPower(0);
        }

        if(controller.isPressed(Controller.Button.RIGHT_STICK_BUTTON)){
            useCenterX = !useCenterX;
        }

        if(controller.isPressed(Controller.Button.DPAD_UP)){
            decimation++;
            aprilTag.setDecimation(decimation);
        }

        if(controller.isPressed(Controller.Button.DPAD_DOWN)){
            decimation--;
            aprilTag.setDecimation(decimation);
        }

        telemetry.addData("fps", visionPortal.getFps());
        telemetry.addData("decimation", decimation);
        telemetry.addData("useCenterX", useCenterX);
        telemetry.addData("move", move);

        tag = getTag24();
        if(tag != null) {
            if(useCenterX){
                bearing = tag.ftcPose.bearing;
                centerX = tag.center.x;
                telemetry.addData("id", tag.id);
                telemetry.addData("Bearing", bearing);
                telemetry.addData("centerX", centerX);

                if (centerX < 520 && centerX > 504) {
                    telemetry.addLine("GOOD");
                    telemetry.addData("power", 0);
                    if(move) turret.setPower(0);
                } else if (centerX < 624 && centerX > 400) {
                    telemetry.addLine("CLOSE");
                    telemetry.addData("power", centerX > 512 ? 0.1 : -0.1);
                    if(move) turret.setPower(centerX > 512 ? 0.1 : -0.1);
                } else {
                    telemetry.addLine("BAD");
                    telemetry.addData("power", centerX > 512 ? 0.5 : -0.5);
                    if(move) turret.setPower(centerX > 512 ? 0.5 : -0.5);
                }
            }else {
                bearing = tag.ftcPose.bearing;
                centerX = tag.center.x;
                telemetry.addData("id", tag.id);
                telemetry.addData("Bearing", bearing);
                telemetry.addData("centerX", centerX);

                if (Math.abs(bearing) <= 5) {
                    telemetry.addLine("GOOD");
                    telemetry.addData("power", 0);
                    if (move) turret.setPower(0);
                } else if (Math.abs(bearing) <= 15) {
                    telemetry.addLine("CLOSE");
                    telemetry.addData("power", bearing < 0 ? 0.2 : -0.2);
                    if (move) turret.setPower(bearing < 0 ? 0.2 : -0.2);
                } else {
                    telemetry.addLine("BAD");
                    telemetry.addData("power", bearing < 0 ? 0.5 : -0.5);
                    if (move) turret.setPower(bearing < 0 ? 0.5 : -0.5);
                }
            }
        }else{
            telemetry.addLine("Tag 24 Not Found");
            if (move) turret.setPower(bearing < 0 ? 0.3 : -0.3);
        }
        telemetry.update();
    }

    private AprilTagDetection getTag24(){
        detectionList = aprilTag.getDetections();
        if(detectionList.isEmpty()) return null;
        for(AprilTagDetection detection : detectionList){
            if(detection.id == 24) return detection;
        }
        return null;
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

                //.setLensIntrinsics(fx, fy, cx, cy)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(1);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
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
