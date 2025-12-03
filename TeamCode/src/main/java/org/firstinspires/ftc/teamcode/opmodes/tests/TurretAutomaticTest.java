package org.firstinspires.ftc.teamcode.opmodes.tests;

import android.util.Size;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.game.Controller;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous
@Configurable
public class TurretAutomaticTest extends OpMode {

    static double ticksPerRev = 145.1;
    static double ticksPerDeg = ticksPerRev / 360;
    static int baseMotorSpeed = 1150;
    static double drivePulleyTeeth = 24;
    static double turretPulleyTeeth = 134;

    static double gearRatio = turretPulleyTeeth / drivePulleyTeeth;

    static double effectiveTicksPerRev = 145.1 * gearRatio;
    static double effectiveTicksPerDeg = effectiveTicksPerRev / 360;

    private VoltageSensor batteryVoltageSensor;

    DcMotorEx turret;
    Controller controller;

    IMU imu;

    boolean move = false;

    int pos = 0;

    // CAMERA

    int decimation = 1;

    static double fx = 595.21, fy = 595.21, cx = 984.515, cy = 599.035;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    List<AprilTagDetection> detectionList;
    AprilTagDetection tag;

    double bearing;
    double centerX;
    double headingImu;

    boolean useCenterX;
    boolean runToPos;

    @Override
    public void init() {
        initAprilTag();
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        //turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setTargetPosition(turret.getCurrentPosition());
        turret.setTargetPositionTolerance(10);
        //PIDFCoefficients runToPosCoefs = turret.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        //setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, new PIDFCoefficients(runToPosCoefs.p*1.3, 0, 0, 0));
        controller = new Controller(gamepad1);
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters params = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.LEFT
        ));
    }

    @Override
    public void loop() {

        if (controller.isPressed(Controller.Button.OPTIONS)) {
            imu.resetYaw();
        }

        if (controller.isPressed(Controller.Button.SHARE)){
            DcMotorEx.RunMode mode = turret.getMode();
            turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turret.setMode(mode);
        }

        if(controller.isPressed(Controller.Button.LEFT_STICK_BUTTON)){
            move = !move;
            turret.setPower(0);
        }

        if(controller.isPressed(Controller.Button.RIGHT_STICK_BUTTON)){
            useCenterX = !useCenterX;
        }

        if(controller.isPressed(Controller.Button.PS)){
            runToPos = !runToPos;
            turret.setMode(runToPos ? DcMotor.RunMode.RUN_TO_POSITION : DcMotor.RunMode.RUN_USING_ENCODER);
            if(runToPos) {
                turret.setPower(1);
                turret.setTargetPosition(turret.getCurrentPosition());
            }
        }

        if(controller.isPressed(Controller.Button.DPAD_UP)){
            decimation++;
            aprilTag.setDecimation(decimation);
        }

        if(controller.isPressed(Controller.Button.DPAD_DOWN)){
            decimation--;
            aprilTag.setDecimation(decimation);
        }

        headingImu = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        telemetry.addData("heading imu", headingImu);
        telemetry.addData("pos", turret.getCurrentPosition());
        telemetry.addData("fps", visionPortal.getFps());
        telemetry.addData("decimation", decimation);
        telemetry.addData("useCenterX", useCenterX);
        telemetry.addData("move", move);
        telemetry.addData("run to pos", runToPos);

        tag = getTag24();
        if(tag != null) {

            bearing = tag.ftcPose.bearing;
            centerX = tag.center.x;
            telemetry.addData("id", tag.id);
            telemetry.addData("bearing", bearing);
            telemetry.addData("center x", centerX);

            if(runToPos){
                telemetry.addLine("target tag");
                turret.setTargetPosition(turret.getCurrentPosition() + (int) (-bearing * effectiveTicksPerDeg));
            }else {
                if (useCenterX) {
                    if (centerX < 520 && centerX > 504) {
                        telemetry.addLine("good");
                        telemetry.addData("power", 0);
                        if (move) turret.setPower(0);
                    } else if (centerX < 624 && centerX > 400) {
                        telemetry.addLine("close");
                        telemetry.addData("power", centerX > 512 ? 0.1 : -0.1);
                        if (move) turret.setPower(centerX > 512 ? 0.1 : -0.1);
                    } else {
                        telemetry.addLine("bad");
                        telemetry.addData("power", centerX > 512 ? 0.5 : -0.5);
                        if (move) turret.setPower(centerX > 512 ? 0.5 : -0.5);
                    }
                } else {
                    if (Math.abs(bearing) <= 5) {
                        telemetry.addLine("good");
                        telemetry.addData("power", 0);
                        if (move) turret.setPower(0);
                    } else if (Math.abs(bearing) <= 15) {
                        telemetry.addLine("close");
                        telemetry.addData("power", bearing < 0 ? 0.2 : -0.2);
                        if (move) turret.setPower(bearing < 0 ? 0.2 : -0.2);
                    } else {
                        telemetry.addLine("bad");
                        telemetry.addData("power", bearing < 0 ? 0.5 : -0.5);
                        if (move) turret.setPower(bearing < 0 ? 0.5 : -0.5);
                    }
                }
            }
        }else{
            telemetry.addLine("tag 24 not found");
            if (move&&!runToPos) turret.setPower(bearing < 0 ? 0.3 : -0.3);
            if (runToPos){
                telemetry.addLine("target imu");
                turret.setTargetPosition((int) (headingImu * effectiveTicksPerDeg));
            }
        }

        if(move&&runToPos){
            turret.setPower(turret.getTargetPosition() > turret.getCurrentPosition() ? 1 : -1);
        }

        if(!move) turret.setPower(0);

        telemetry.addData("target", turret.getTargetPosition());
        telemetry.update();
    }

    private AprilTagDetection getTag24(){
        return getTag(24);
    }

    private AprilTagDetection getTag(int id){
        detectionList = aprilTag.getDetections();
        if(detectionList.isEmpty()) return null;
        for(AprilTagDetection detection : detectionList){
            if(detection.id == id) return detection;
        }
        return null;
    }

    private AprilTagDetection getTag20(){
        return getTag(20);
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
        aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(1280, 960));

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
