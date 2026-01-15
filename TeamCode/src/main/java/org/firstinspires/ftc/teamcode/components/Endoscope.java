package org.firstinspires.ftc.teamcode.components;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.HardwareUtil;
import org.firstinspires.ftc.teamcode.util.LogCatUtil;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

import java.util.Arrays;

public class Endoscope extends BaseComponent {
    LogCatUtil log;
    HardwareUtil hardwareUtil;
    VisionPortal portal;
    PredominantColorProcessor frontBallSensor;
    PredominantColorProcessor centerBallSensor;
    PredominantColorProcessor rearBallSensor;
    PredominantColorProcessor prelimFrontSensor;
    PredominantColorProcessor prelimRearSensor;

    int prelimDetectSaturation = 200;
    int prelimDetectValue = 100;


    boolean enableArtifactManagement = true;

    Robot robot;
    Transfer transfer;

    PredominantColorProcessor blobMaker(double left, double top, double right, double bottom){
        return new PredominantColorProcessor.Builder()
                .setRoi(ImageRegion.asUnityCenterCoordinates(left, top, right, bottom))
                .setSwatches(PredominantColorProcessor.Swatch.BLACK)
                .build();
    }

    public Endoscope(RobotContext context, Robot robot) {
        super(context);
        log = new LogCatUtil("Endoscope");
        hardwareUtil = new HardwareUtil(log, hardwareMap);
        this.robot = robot;

        frontBallSensor = blobMaker(-0.514867, -0.382046, -0.317684, -0.878914);
        centerBallSensor = blobMaker(-0.114241, 0.955, 0.170579, 0.7);
        rearBallSensor = blobMaker(0.411581, -0.348643, 0.605634, -0.845511);
        prelimFrontSensor = blobMaker(-0.984351, 0.043841, -0.837246, -0.077244);
        prelimRearSensor = blobMaker(0.809077, 0.018789, 0.968701, -0.089770);


        portal = new VisionPortal.Builder()
                .addProcessor(frontBallSensor)
                .addProcessor(centerBallSensor)
                .addProcessor(rearBallSensor)
                .addProcessor(prelimFrontSensor)
                .addProcessor(prelimRearSensor)
                .setCameraResolution(new Size(320, 240))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .setShowStatsOverlay(true)
                .setCamera(hardwareMap.get(WebcamName.class, "Endoscope"))
                .build();
    }

    @Override
    public void init(){
        transfer = robot.getTransfer();
    }

    @Override
    public void update(){
        PredominantColorProcessor.Result resultFront = frontBallSensor.getAnalysis();
        PredominantColorProcessor.Result resultCenter = centerBallSensor.getAnalysis();
        PredominantColorProcessor.Result resultRear = rearBallSensor.getAnalysis();
        PredominantColorProcessor.Result resultPrelimFront = prelimFrontSensor.getAnalysis();
        PredominantColorProcessor.Result resultPrelimRear = prelimRearSensor.getAnalysis();

        telemetry.addData("Front HSV", Arrays.toString(resultFront.HSV));
        telemetry.addData("Center HSV", Arrays.toString(resultCenter.HSV));
        telemetry.addData("Rear HSV", Arrays.toString(resultRear.HSV));
        telemetry.addData("PrelimFront HSV", Arrays.toString(resultPrelimFront.HSV));
        telemetry.addData("PrelimRear HSV", Arrays.toString(resultPrelimRear.HSV));
        telemetry.addLine("----------------");
        telemetry.addData("Front Detect", getPresence(resultFront.HSV));
        telemetry.addData("Center Detect", getPresence(resultCenter.HSV));
        telemetry.addData("Rear Detect", getPresence(resultRear.HSV));
        telemetry.addData("PrelimFront Detect", resultPrelimFront.HSV[1] > prelimDetectSaturation && resultPrelimFront.HSV[2] > prelimDetectValue);
        telemetry.addData("PrelimRear Detect", resultPrelimRear.HSV[1] > prelimDetectSaturation && resultPrelimRear.HSV[2] > prelimDetectValue);

        if((enableArtifactManagement) && (resultPrelimFront.HSV[1] > prelimDetectSaturation) && (resultPrelimFront.HSV[2] > prelimDetectValue) && (getPresence(resultFront.HSV) == 0)){
            transfer.incomingFront();
            telemetry.addLine("incoming Front!");
        }
        if((enableArtifactManagement) && (resultPrelimRear.HSV[1] > prelimDetectSaturation) && (resultPrelimRear.HSV[2] > prelimDetectValue) && (getPresence(resultRear.HSV) == 0)){
            transfer.incomingRear();
            telemetry.addLine("incoming Rear!");
        }
    }

    /**
    Colors:
     0 = no ball
     1 = purple ball
     2 = green ball
     3 = unknown
     **/
    public int getPresence(int[] HSV){
        if (HSV[1] < 160){
            return 0;
        } else if (HSV[0] > 120) {
            return 1;
        } else if (HSV[0] <= 120){
            return 2;
        } else {
            return 3;
        }
    }

    public PredominantColorProcessor getFrontBallSensor() {
        return frontBallSensor;
    }

    public PredominantColorProcessor getCenterBallSensor() {
        return centerBallSensor;
    }

    public PredominantColorProcessor getRearBallSensor() {
        return rearBallSensor;
    }

    public void setEnableArtifactManagement(boolean enableArtifactManagement) {
        this.enableArtifactManagement = enableArtifactManagement;
    }
}
