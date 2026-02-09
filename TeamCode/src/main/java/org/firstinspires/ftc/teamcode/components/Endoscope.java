package org.firstinspires.ftc.teamcode.components;

import android.util.Size;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcontroller.external.samples.ConceptLEDStick;
import org.firstinspires.ftc.teamcode.custom.ImageRegion;
import org.firstinspires.ftc.teamcode.custom.PredominantColorProcessor;
import org.firstinspires.ftc.teamcode.lib.*;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.teamcode.util.HardwareUtil;
import org.firstinspires.ftc.teamcode.util.LogCatUtil;
import org.firstinspires.ftc.vision.VisionPortal;
import org.openftc.easyopencv.OpenCvCameraFactory;

import java.util.Arrays;
import java.util.Vector;
import java.util.concurrent.TimeUnit;

public class Endoscope extends BaseComponent {
    LogCatUtil log;
    HardwareUtil hardwareUtil;
    VisionPortal portal;
    PredominantColorProcessor frontBallSensor;
    PredominantColorProcessor centerBallSensor;
    PredominantColorProcessor rearBallSensor;
    PredominantColorProcessor prelimFrontSensor;
    PredominantColorProcessor prelimRearSensor;

    PredominantColorProcessor.Result resultFront = new PredominantColorProcessor.Result(new int[3]);
    PredominantColorProcessor.Result resultCenter = new PredominantColorProcessor.Result(new int[3]);
    PredominantColorProcessor.Result resultRear = new PredominantColorProcessor.Result(new int[3]);
    PredominantColorProcessor.Result resultPrelimFront = new PredominantColorProcessor.Result(new int[3]);
    PredominantColorProcessor.Result resultPrelimRear = new PredominantColorProcessor.Result(new int[3]);

    PredominantColorProcessor.Result previousResultFront = new PredominantColorProcessor.Result(new int[3]);
    PredominantColorProcessor.Result previousResultCenter = new PredominantColorProcessor.Result(new int[3]);
    PredominantColorProcessor.Result previousResultRear = new PredominantColorProcessor.Result(new int[3]);
    PredominantColorProcessor.Result previousResultPrelimFront = new PredominantColorProcessor.Result(new int[3]);
    PredominantColorProcessor.Result previousResultPrelimRear = new PredominantColorProcessor.Result(new int[3]);

    PrismAnimations.Solid solid = new PrismAnimations.Solid(Color.WHITE);

    public int prelimDetectValue = 70;

    boolean enableArtifactManagement = true;
    boolean cameraSettingsSet = false;

    Robot robot;
    UpgradedTransfer transfer;
    Servo internalLight;
    GoBildaPrismDriver lights;

    static int exposureMs = 32;
    static int gain = 10;

    int liveViewContainerId;

    PredominantColorProcessor blobMaker(double left, double top, double right, double bottom, String name){
        return new PredominantColorProcessor.Builder()
                .setRoi(ImageRegion.asUnityCenterCoordinates(left, top, right, bottom))
                .setSwatches(PredominantColorProcessor.Swatch.BLACK)
                .setName(name)
                .build();
    }

    public Endoscope(RobotContext context, Robot robot, int liveViewContainerId) {
        super(context);
        log = new LogCatUtil("Endoscope");
        hardwareUtil = new HardwareUtil(log, hardwareMap);
        this.robot = robot;
        this.liveViewContainerId = liveViewContainerId;
    }

    @Override
    public void init(){
        frontBallSensor = blobMaker(-0.45, -0.25, -0.317684, -0.35, "frontBallSensor");
        centerBallSensor = blobMaker(0, 0.15, 0.1, -.05, "centerBallSensor");
        rearBallSensor = blobMaker(0.411581, -0.25, 0.55, -0.35, "rearBallSensor");
        prelimFrontSensor = blobMaker(-0.94, 0.043841, -0.8, -0.077244, "prelimFrontSensor");
        prelimRearSensor = blobMaker(0.809077, 0.018789, 0.968701, -0.089770, "prelimRearSensor");

        portal = new VisionPortal.Builder()
                .addProcessor(frontBallSensor)
                .addProcessor(centerBallSensor)
                .addProcessor(rearBallSensor)
                .addProcessor(prelimFrontSensor)
                .addProcessor(prelimRearSensor)
                .setCameraResolution(new Size(320, 240))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setShowStatsOverlay(true)
                .setCamera(hardwareUtil.getWebcamName("Endoscope"))
//                .enableLiveView(false)
                .setLiveViewContainerId(liveViewContainerId)
                .build();

        internalLight = hardwareUtil.getServo("internalLight");
        internalLight.setPosition(1);

        lights = hardwareUtil.getGoBildaPrismDriver("lightStrip");

        lights.loadAnimationsFromArtboard(GoBildaPrismDriver.Artboard.ARTBOARD_0);
        lights.updateAllAnimations();

        transfer = robot.getTransfer();
    }

    @Override
    public void update(){
        resultFront = averageResult(frontBallSensor.getAnalysis(), previousResultFront);
        resultCenter = averageResult(centerBallSensor.getAnalysis(), previousResultCenter);
        resultRear = averageResult(rearBallSensor.getAnalysis(), previousResultRear);
        resultPrelimFront = averageResult(prelimFrontSensor.getAnalysis(), previousResultPrelimFront);
        resultPrelimRear = averageResult(prelimRearSensor.getAnalysis(), previousResultPrelimRear);

        previousResultFront = frontBallSensor.getAnalysis();
        previousResultCenter = centerBallSensor.getAnalysis();
        previousResultRear = rearBallSensor.getAnalysis();
        previousResultPrelimFront = prelimFrontSensor.getAnalysis();
        previousResultPrelimRear = prelimRearSensor.getAnalysis();

        if(!cameraSettingsSet && portal.getCameraState() == VisionPortal.CameraState.STREAMING){
            log.debug("Camera set. Expos: " + exposureMs + " Gain: " + gain);
            portal.getCameraControl(ExposureControl.class).setMode(ExposureControl.Mode.Manual);
            portal.getCameraControl(ExposureControl.class).setExposure(exposureMs, TimeUnit.MILLISECONDS);
            portal.getCameraControl(GainControl.class).setGain(gain);
            cameraSettingsSet = true;
        }

        internalLight.setPosition(1);

        if((enableArtifactManagement)  && (resultPrelimFront.HSV[2] > prelimDetectValue) && (getPresence(resultFront.HSV) == 0)){
            transfer.incomingFront();
            telemetry.addLine("incoming Front!");
//            log.debug("incoming Front! HSV: " + Arrays.toString(resultPrelimFront.HSV));
        }

        if((enableArtifactManagement) && (resultPrelimRear.HSV[2] > prelimDetectValue) && (getPresence(resultRear.HSV) == 0)){
            transfer.incomingRear();
            telemetry.addLine("incoming Rear!");
//            log.debug("incoming Rear! HSV: " + Arrays.toString(resultPrelimRear.HSV));
        }
    }

    @Override
    public void addTelemetry(){
        telemetry.addData("Front HSV", Arrays.toString(resultFront.HSV));
        telemetry.addData("Center HSV", Arrays.toString(resultCenter.HSV));
        telemetry.addData("Rear HSV", Arrays.toString(resultRear.HSV));
        telemetry.addData("PrelimFront HSV", Arrays.toString(resultPrelimFront.HSV));
        telemetry.addData("PrelimRear HSV", Arrays.toString(resultPrelimRear.HSV));
        telemetry.addLine("----------------");
        telemetry.addData("Front Detect", getPresence(resultFront.HSV));
        telemetry.addData("Center Detect", getPresence(resultCenter.HSV));
        telemetry.addData("Rear Detect", getPresence(resultRear.HSV));
        telemetry.addLine("------ Management: " + enableArtifactManagement + " ------");
        telemetry.addData("PrelimFront Detect",  resultPrelimFront.HSV[2] > prelimDetectValue);
        telemetry.addData("PrelimRear Detect",  resultPrelimRear.HSV[2] > prelimDetectValue);
    }

    /**
    Colors:
     0 = no ball
     1 = purple ball
     2 = green ball
     3 = unknown
     **/
    public int getPresence(int[] HSV){
        if (HSV[0] < 50 || HSV[0] > 160){
            return 0;
        // plus or minus 20 from 130
        } else if (Math.abs(HSV[0] - 130) < 20) {
            return 1;
        } else if (Math.abs(HSV[0] - 80) < 20){
            return 2;
        } else {
            return 3;
        }
    }

    /*
        100 black
        130 P
        80 G
     */

    public PredominantColorProcessor getFrontBallSensor() {
        return frontBallSensor;
    }

    public PredominantColorProcessor getCenterBallSensor() {
        return centerBallSensor;
    }

    public PredominantColorProcessor getRearBallSensor() {
        return rearBallSensor;
    }

    public PredominantColorProcessor getPrelimFrontSensor() {
        return prelimFrontSensor;
    }

    public PredominantColorProcessor getPrelimRearSensor() {
        return prelimRearSensor;
    }

    public void setEnableArtifactManagement(boolean enableArtifactManagement) {
        this.enableArtifactManagement = enableArtifactManagement;
    }
    public boolean getEnableArtifactManagement(){
        return enableArtifactManagement;
    }

    private int[] averageHSV(Vector<int[]> HSVs){
        int[] result = new int[3];

        for(int[] list : HSVs){
            for(int i = 0; i < 3; i++) {
                result[i] += list[i];
            }
        }

        for(int i = 0; i <= 2; i++) {
            result[i] /= HSVs.size();
        }

        return(result);
    }

    private PredominantColorProcessor.Result averageResult(PredominantColorProcessor.Result... results){
//        int[][] hsvs = new int[3][results.length];

        Vector<int[]> hsvs = new Vector<>();

        for(PredominantColorProcessor.Result result : results){
            hsvs.add(result.HSV);
        }

        return new PredominantColorProcessor.Result(averageHSV(hsvs));
    }
}
