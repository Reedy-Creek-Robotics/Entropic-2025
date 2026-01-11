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


    PredominantColorProcessor blobMaker(double left, double top, double right, double bottom){
        return new PredominantColorProcessor.Builder()
                .setRoi(ImageRegion.asUnityCenterCoordinates(left, top, right, bottom))
                .setSwatches(
                        PredominantColorProcessor.Swatch.ARTIFACT_GREEN,
                        PredominantColorProcessor.Swatch.ARTIFACT_PURPLE,
                        PredominantColorProcessor.Swatch.RED,
                        PredominantColorProcessor.Swatch.BLUE,
                        PredominantColorProcessor.Swatch.YELLOW,
                        PredominantColorProcessor.Swatch.WHITE,
                        PredominantColorProcessor.Swatch.ORANGE,
                        PredominantColorProcessor.Swatch.BLACK
                )
                .build();
    }


    public Endoscope(RobotContext context, Robot robot) {
        super(context);
        log = new LogCatUtil("Endoscope");
        hardwareUtil = new HardwareUtil(log, hardwareMap);

        frontBallSensor = blobMaker(-0.787167, 0.340292, -0.176839, -0.164927);
        centerBallSensor = blobMaker(-0.133020, 0.770355, 0.276995, -0.411273);
        rearBallSensor = blobMaker(0.364632, 0.361169, 0.974961, -0.820459);


        portal = new VisionPortal.Builder()
                .addProcessor(frontBallSensor)
                .addProcessor(centerBallSensor)
                .addProcessor(rearBallSensor)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
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

        telemetry.addData("Front Closest Swatch", resultFront.closestSwatch);
        telemetry.addData("Center Closest Swatch", resultCenter.closestSwatch);
        telemetry.addData("Rear Closest Swatch", resultRear.closestSwatch);

        telemetry.addData("Front HSV", Arrays.toString(resultFront.HSV));
        telemetry.addData("Center HSV", Arrays.toString(resultCenter.HSV));
        telemetry.addData("Rear HSV", Arrays.toString(resultRear.HSV));

        telemetry.addData("Front Detect", getPresence(resultFront.HSV));
        telemetry.addData("Center Detect", getPresence(resultCenter.HSV));
        telemetry.addData("Rear Detect", getPresence(resultRear.HSV));
    }

    // 0 = no ball
    // 1 = purple ball
    // 2 = green ball
    // 3 = unknown
    public int getPresence(int[] HSV){
        if (HSV[1] < 120){
            return 0;
        } else if (HSV[0] > 120) {
            return 1;
        } else if (HSV[0] <= 120){
            return 2;
        } else {
            return 3;
        }
    }
}
