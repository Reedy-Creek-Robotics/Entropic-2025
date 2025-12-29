package org.firstinspires.ftc.teamcode.components;

import android.util.Size;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.util.HardwareUtil;
import org.firstinspires.ftc.teamcode.util.LogCatUtil;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.opencv.ColorRange;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

public class Endoscope extends BaseComponent {
    LogCatUtil log;
    HardwareUtil hardwareUtil;
    VisionPortal portal;
    PredominantColorProcessor frontBallSensor;
    PredominantColorProcessor centerBallSensor;
    PredominantColorProcessor rearBallSensor;


    PredominantColorProcessor blobMaker(double left, double top, double right, double bottom){
        return new PredominantColorProcessor.Builder()
                .setRoi(ImageRegion.asUnityCenterCoordinates(left, top, right, bottom))
                .setSwatches(
                        PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
                        PredominantColorProcessor.Swatch.ARTIFACT_PURPLE
                )
                .build();
    }


    public Endoscope(RobotContext context) {
        super(context);
        log = new LogCatUtil("Endoscope");
        hardwareUtil = new HardwareUtil(log, hardwareMap);

        frontBallSensor = blobMaker(.3, 1, 1, -1);
        centerBallSensor = blobMaker(.3, 1, 1, -1);
        rearBallSensor = blobMaker(.3, 1, 1, -1);


        portal = new VisionPortal.Builder()
                .addProcessor(frontBallSensor)
                .addProcessor(centerBallSensor)
                .addProcessor(rearBallSensor)
                .setCameraResolution(new Size(640, 480))
                .setCamera(hardwareMap.get(WebcamName.class, "Endoscope"))
                .build();
    }

    @Override
    public void init(){

    }

    @Override
    public void update(){
        PredominantColorProcessor.Result resultFront = frontBallSensor.getAnalysis();
        PredominantColorProcessor.Result resultCenter = centerBallSensor.getAnalysis();
        PredominantColorProcessor.Result resultRear = rearBallSensor.getAnalysis();

        log.info("Front Closest Swatch: " + resultFront.closestSwatch.toString());
        log.info("Center Closest Swatch: " + resultCenter.closestSwatch.toString());
        log.info("Rear Closest Swatch: " + resultRear.closestSwatch.toString());
    }
}
