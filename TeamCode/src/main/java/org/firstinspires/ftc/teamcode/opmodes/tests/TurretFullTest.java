package org.firstinspires.ftc.teamcode.opmodes.tests;

import android.util.Log;
import android.util.Size;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.components.DriveTrain;
import org.firstinspires.ftc.teamcode.components.Robot;
import org.firstinspires.ftc.teamcode.game.Controller;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

/*
    All of this code assumes toward the goal wall is positive y, and toward the red goal is positive x.
    Origin is where the robot starts, currently 3 tiles down from the red goal, and 2 tiles over, at the intersection.
    Other objects, like the goals, are measured from this position
    Forward facing is directly toward the goal wall.
 */

@TeleOp
@Disabled
@Configurable
public class TurretFullTest extends OpMode {

    static double ticksPerRev = 145.1;
    static double ticksPerDeg = ticksPerRev / 360;
    static int baseMotorSpeed = 1150;
    static double drivePulleyTeeth = 24;
    static double turretPulleyTeeth = 134;

    static double gearRatio = turretPulleyTeeth / drivePulleyTeeth;

    static double effectiveTicksPerRev = 145.1 * gearRatio;
    static double effectiveTicksPerDeg = effectiveTicksPerRev / 360;

    // Must be at least 360 degrees
    static double maxHeading = 360 * 2;
    static double minHeading = -360 * 2;

    SparkFunOTOS.Pose2D otosPos = new SparkFunOTOS.Pose2D();
    SparkFunOTOS.Pose2D otosVel = new SparkFunOTOS.Pose2D();
    SparkFunOTOS.Pose2D otosAcc = new SparkFunOTOS.Pose2D();

    static SparkFunOTOS.Pose2D otosOffset = new SparkFunOTOS.Pose2D(-1, 0, -90);
    static SparkFunOTOS.Pose2D startPos = new SparkFunOTOS.Pose2D(0, 0, 0);

    static SparkFunOTOS.Pose2D redTag = new SparkFunOTOS.Pose2D(31.875, 58.5, 234);
    static SparkFunOTOS.Pose2D blueTag = new SparkFunOTOS.Pose2D(0, 0, 144); //ToDo update to match coordinate system

    /**
     * false - red (tag 24)<br>
     * true - blue (tag 20)
     */
    boolean alliance = false;

    SparkFunOTOS otos;
    DcMotorEx turret;
    AprilTagProcessor aprilTag;
    VisionPortal visionPortal;
    Controller controller;
    PanelsTelemetry panelsTelemetry;
    TelemetryManager telem;
    Robot robot;
    DriveTrain driveTrain;

    /**
     * 0 - otos only<br>
     * 1 - tag only - stop if no tag<br>
     * 2 - tag when found - otos when no tag<br>
     * 3 - otos with periodic relocalization from tag
     */
    int autoAimMethod = 0;
    
    static String[] aimMethods = {"otos only", "tag only", "tag + otos", "relocalize"};

    /**
     * 0 - run to position<br>
     * 1 - rough 'pid' like system
     */
    int moveMethod = 1;
    
    static String[] moveMethods = {"run to position", "rough 'pid'"};

    double targetPos = 0;

    @Override
    public void init() {
        //initAprilTag();

        otos = hardwareMap.get(SparkFunOTOS.class, "otos");
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        controller = new Controller(gamepad1);
        panelsTelemetry = PanelsTelemetry.INSTANCE;
        telem = panelsTelemetry.getTelemetry();
        driveTrain = new Robot(this).getDriveTrain();
        driveTrain.init();

        otos.resetTracking();
        otos.setLinearUnit(DistanceUnit.INCH);
        otos.setAngularUnit(AngleUnit.DEGREES);
        otos.setOffset(new SparkFunOTOS.Pose2D(-3, -2.75, 90));
        otos.setPosition(startPos);
        otos.setLinearScalar(72/71.3);
        otos.setAngularScalar(1800/1781.5);
        otos.calibrateImu();

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    @Override
    public void start() {
        super.start();
    }

    @Override
    public void loop() {
        otos.getPosVelAcc(otosPos, otosVel, otosAcc);
        //clockwise is positive rot. otosPos.h represents this from 0 - 360 degrees.
        //otosPos.h = otosPos.h + 180;

        if(controller.isPressed(Controller.Button.RIGHT_STICK_BUTTON)){
            if(turret.isMotorEnabled()) {
                turret.setMotorDisable();
            } else {
                turret.setMotorEnable();
            }
        }

        driveTrain.drive(controller.analogValue(Controller.AnalogControl.LEFT_STICK_Y), controller.analogValue(Controller.AnalogControl.LEFT_STICK_X), controller.analogValue(Controller.AnalogControl.RIGHT_STICK_X), 1);

        if(controller.isPressed(Controller.Button.DPAD_UP)){
            autoAimMethod++;
        }else if(controller.isPressed(Controller.Button.DPAD_DOWN)){
            autoAimMethod--;
        }
        
        if(autoAimMethod > 3) autoAimMethod = 3;
        if(autoAimMethod < 0) autoAimMethod = 0;

        if(controller.isPressed(Controller.Button.DPAD_RIGHT)){
            moveMethod++;
        }else if(controller.isPressed(Controller.Button.DPAD_LEFT)){
            moveMethod--;
        }

        if(moveMethod > 1) moveMethod = 1;
        if(moveMethod < 0) moveMethod = 0;

        switch(autoAimMethod){
            case 0:
                otosAutoAim();
            case 1:
                tagAutoAim();
            case 2:
                tagOtosAutoAim();
            case 3:
                otosRelocalizeAutoAim();
        }

        switch(moveMethod){
            case 0:
                moveRtp();
            case 1:
                movePid();
        }

        telem.addData("otos x", otosPos.x);
        telem.addData("otos y", otosPos.y);
        telem.addData("otos h", otosPos.h);
        telem.addData("pos", turret.getCurrentPosition());
        telem.addData("target", targetPos);
        telem.addData("auto aim method", aimMethods[autoAimMethod]);
        telem.addData("move method", moveMethods[moveMethod]);
        telem.update(telemetry);
    }

    private void otosAutoAim(){
        // Calculates the theta using tanh function
        double theta = Math.toDegrees(Math.tanh((alliance ? blueTag.x : redTag.x - otosPos.x) / (alliance ? blueTag.y : redTag.y - otosPos.y)));
        // We subtract the theta from the heading to account for robot rotation.
        setTargetDegrees(-otosPos.h - theta);
        telem.addData("theta", theta);
        telem.addData("delta deg", -otosPos.h - theta);
    }

    private void tagAutoAim(){

    }

    private void tagOtosAutoAim(){

    }

    private void otosRelocalizeAutoAim(){

    }

    private void moveRtp(){

    }

    private void movePid(){
        //turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        if (Math.abs(turret.getCurrentPosition() - targetPos) <= 3) {
            turret.setPower(0);
        } else if (Math.abs(turret.getCurrentPosition() - targetPos) <= 25) {
            turret.setPower(turret.getCurrentPosition() < (int) targetPos ? 0.1 : -0.1);
        } else {
            turret.setPower(turret.getCurrentPosition() < (int) targetPos ? 0.2 : -0.2);
        }

        telem.addData("power", turret.getPower());
    }

    private void setTargetDegrees(double degrees){
        //if(degrees < 0) degrees = 360 + degrees;
        degrees = degrees + getPositionDegrees();
        degrees = -degrees /*+ getPositionDegrees()*/;

        while(degrees > maxHeading){
            degrees = degrees - 360;
            Log.println(Log.WARN, "turret", "over max heading");
        }

        while(degrees < minHeading){
            degrees = degrees + 360;
            Log.println(Log.WARN, "turret", "under min heading");
        }

        targetPos = degrees * effectiveTicksPerDeg;
    }

    private double getPositionDegrees(){
        return turret.getCurrentPosition() / effectiveTicksPerDeg;
    }

    private SparkFunOTOS.Pose2D pose3dToPose2d(Pose3D pos){
        pos.getPosition().toUnit(DistanceUnit.INCH);
        double x = pos.getPosition().y+72;
        double y = pos.getPosition().x-72;
        double yaw = pos.getOrientation().getYaw(AngleUnit.DEGREES);

        return new SparkFunOTOS.Pose2D(x, y, yaw);
    }

    /*private void initAprilTag() {

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

    }   // end method initAprilTag()*/
}
