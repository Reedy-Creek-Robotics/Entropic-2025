package org.firstinspires.ftc.teamcode.opmodes.tests;

import android.util.Size;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS.Pose2D;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.components.DriveTrain;
import org.firstinspires.ftc.teamcode.components.Robot;
import org.firstinspires.ftc.teamcode.game.Controller;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

//ToDo make camera tracking set otos pose on each new frame with a valid tag.

@TeleOp
@Configurable
public class TurretDriveTest extends OpMode {

    ElapsedTime timer;

    static double ticksPerRev = 145.1;
    static double ticksPerDeg = ticksPerRev / 360;
    static int baseMotorSpeed = 1150;
    static double drivePulleyTeeth = 24;
    static double turretPulleyTeeth = 134;

    static Pose2D redTag = new Pose2D(128, 128, 234);
    static Pose2D blueTag = new Pose2D(16, 128, 144);

    static double gearRatio = turretPulleyTeeth / drivePulleyTeeth;

    static double effectiveTicksPerRev = 145.1 * gearRatio;
    static double effectiveTicksPerDeg = effectiveTicksPerRev / 360;

    private VoltageSensor batteryVoltageSensor;

    /**
     * false - red (tag 24)<br>
     * true - blue (tag 20)
     */
    boolean alliance = true;

    DcMotorEx turret;
    Controller controller;

    PanelsTelemetry panelsTelem;
    TelemetryManager telem;


    Robot robot;
    DriveTrain driveTrain;

    SparkFunOTOS otos;
    Pose2D otosPos = new Pose2D();
    Pose2D otosVel = new Pose2D();
    Pose2D otosAcc = new Pose2D();

    static Pose2D otosOffset = new Pose2D(-2.75, 0, Math.toRadians(90));
    static Pose2D startPos = new Pose2D(72-otosOffset.x, 72, Math.toRadians(0));

    // CAMERA

    int decimation = 1;

    boolean freshFrame = true;

    static double fx = 595.21, fy = 595.21, cx = 984.515, cy = 599.035;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;

    List<AprilTagDetection> detectionList;
    AprilTagDetection tag;

    Pose2D tagPose = new Pose2D();

    double bearing;
    double previousBearing;

    @Override
    public void init() {
        initAprilTag();
        timer = new ElapsedTime();
        panelsTelem = PanelsTelemetry.INSTANCE;
        telem = panelsTelem.getTelemetry();
        otos = hardwareMap.get(SparkFunOTOS.class, "otos");
        otos.setOffset(otosOffset);
        otos.setPosition(startPos);
        otos.calibrateImu();
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setTargetPosition(turret.getCurrentPosition());
        turret.setTargetPositionTolerance(10);
        turret.setPositionPIDFCoefficients(4);
        controller = new Controller(gamepad1);

        driveTrain = new Robot(this).getDriveTrain();
    }

    @Override
    public void start() {
        super.start();

        turret.setPower(1);
        turret.setTargetPosition(0);
    }

    @Override
    public void loop() {
        driveTrain.drive(controller.analogValue(Controller.AnalogControl.LEFT_STICK_Y), controller.analogValue(Controller.AnalogControl.LEFT_STICK_X), controller.analogValue(Controller.AnalogControl.RIGHT_STICK_X), 1);

        otos.getPosVelAcc(otosPos, otosVel, otosAcc);

        if(controller.isPressed(Controller.Button.LEFT_STICK_BUTTON)){
            alliance = !alliance;
        }

        if(controller.isPressed(Controller.Button.RIGHT_STICK_BUTTON)){
            if(turret.isMotorEnabled()) {
                turret.setMotorDisable();
            } else {
                turret.setMotorEnable();
            }
        }

        if(controller.isPressed(Controller.Button.SHARE)){
            DcMotorEx.RunMode mode = turret.getMode();
            turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turret.setMode(mode);
        }

        if(controller.isPressed(Controller.Button.DPAD_UP)){
            decimation++;
            aprilTag.setDecimation(decimation);
        }

        if(controller.isPressed(Controller.Button.DPAD_DOWN)){
            decimation--;
            aprilTag.setDecimation(decimation);
        }

        telem.addData("heading imu", otosPos.h);
        telem.addData("turret ticks", turret.getCurrentPosition());
        telem.addData("fps", visionPortal.getFps());
        telem.addData("decimation", decimation);
        telem.addData("alliance", alliance ? "blue" : "red");

        tag = getTag(alliance ? 20 : 24);
        if(tag != null) {

            bearing = tag.ftcPose.bearing;
            telem.addData("id", tag.id);
            telem.addData("bearing", bearing);

            //telem.addLine("target tag");
            turret.setTargetPosition(turret.getCurrentPosition() + (int) (-bearing * effectiveTicksPerDeg));
            setTargetDegrees(getPositionDegrees() - bearing);

            //previousBearing = bearing;

            if(Math.abs(bearing) <= 5){
                tagPose = new Pose2D(tag.robotPose.getPosition().x + 72, tag.robotPose.getPosition().y + 72, tag.robotPose.getOrientation().getYaw());
                otos.setPosition(tagPose);
            }

        }else{
            telem.addLine("no tag found");
            //telem.addLine("target imu");
            if(tagPose != null) {
                setTargetDegrees(Math.toDegrees(Math.tanh((alliance ? blueTag.x : redTag.x - tagPose.x) / (alliance ? blueTag.y : redTag.y - tagPose.y))));
            }else{
                setTargetDegrees(Math.toDegrees(Math.tanh((alliance ? blueTag.x : redTag.x - otosPos.x) / (alliance ? blueTag.y : redTag.y - otosPos.y))));
            }
        }

        /*if(Math.abs(turret.getCurrentPosition() - turret.getTargetPosition()) <= 15){
            turret.setPower(turret.getTargetPosition() > turret.getCurrentPosition() ? 0.15 : -0.15);
        }else if(Math.abs(turret.getCurrentPosition() - turret.getTargetPosition()) <= 20) {
            turret.setPower(turret.getTargetPosition() > turret.getCurrentPosition() ? 0.3 : -0.3);
        }else if(Math.abs(turret.getCurrentPosition() - turret.getTargetPosition()) <= 50){
            turret.setPower(turret.getTargetPosition() > turret.getCurrentPosition() ? 0.7 : -0.7);
        }else{
            turret.setPower(turret.getTargetPosition() > turret.getCurrentPosition() ? 1 : -1);
        }*/

        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telem.addData("target", turret.getTargetPosition());
        telem.update();
        panelsTelem.getFtcTelemetry().update();
    }

    private void setTargetDegrees(double degrees){
        if(degrees < 0) degrees = 360+degrees;
        turret.setTargetPosition((int) (degrees * ticksPerDeg));
        telem.addData("set target", (int) (degrees * ticksPerDeg));
    }

    private double getPositionDegrees(){
        return turret.getCurrentPosition() / ticksPerDeg;
    }

    private AprilTagDetection getTag(int id){
        detectionList = aprilTag.getDetections();
        if(detectionList.isEmpty()) return null;
        for(AprilTagDetection detection : detectionList){
            if(detection.id == id) return detection;
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
