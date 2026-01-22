package org.firstinspires.ftc.teamcode.opmodes.componentTests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.Endoscope;
import org.firstinspires.ftc.teamcode.components.Robot;
import org.firstinspires.ftc.teamcode.custom.PredominantColorProcessor;

import java.util.Arrays;

@TeleOp
public class EndoscopeTest extends OpMode {
    Robot robot;
    Endoscope endoscope;

    PredominantColorProcessor.Result resultFront;
    PredominantColorProcessor.Result resultCenter;
    PredominantColorProcessor.Result resultRear;
    PredominantColorProcessor.Result resultPrelimFront;
    PredominantColorProcessor.Result resultPrelimRear;

    @Override
    public void init() {
        robot = new Robot(this);
        robot.init();
        endoscope = robot.getEndoscope();

        endoscope.setEnableArtifactManagement(false);
        robot.getTurret().setAutoAim(false);
        robot.getShooter().setAutoSpeed(false);
        robot.getShooter().setVelocity(0);
    }

    @Override
    public void loop() {
        resultFront = endoscope.getFrontBallSensor().getAnalysis();
        resultCenter = endoscope.getCenterBallSensor().getAnalysis();
        resultRear = endoscope.getRearBallSensor().getAnalysis();
        resultPrelimFront = endoscope.getPrelimFrontSensor().getAnalysis();
        resultPrelimRear = endoscope.getPrelimRearSensor().getAnalysis();

        telemetry.addData("Front HSV", Arrays.toString(resultFront.HSV));
        telemetry.addData("Center HSV", Arrays.toString(resultCenter.HSV));
        telemetry.addData("Rear HSV", Arrays.toString(resultRear.HSV));
        telemetry.addData("PrelimFront HSV", Arrays.toString(resultPrelimFront.HSV));
        telemetry.addData("PrelimRear HSV", Arrays.toString(resultPrelimRear.HSV));
        telemetry.addLine("----------------");
        telemetry.addData("Front Detect", endoscope.getPresence(resultFront.HSV));
        telemetry.addData("Center Detect", endoscope.getPresence(resultCenter.HSV));
        telemetry.addData("Rear Detect", endoscope.getPresence(resultRear.HSV));
        telemetry.addLine("----------------");
        telemetry.addData("PrelimFront Detect",  resultPrelimFront.HSV[2] > endoscope.prelimDetectValue);
        telemetry.addData("PrelimRear Detect",  resultPrelimRear.HSV[2] > endoscope.prelimDetectValue);
    }
}
