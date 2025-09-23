package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class PedroTest extends LinearOpMode {

    static Follower follower;

    public static PathBuilder builder;

    @Override
    public void runOpMode() throws InterruptedException {
        follower = Constants.createFollower(hardwareMap);
        builder = follower.pathBuilder();

        follower.setStartingPose(new Pose(84, 12, Math.toRadians(90)));

        PathChain line1 = builder
                .addPath(
                        new BezierCurve(
                                new Pose(84.000, 12.000),
                                new Pose(90.000, 60.000),
                                new Pose(72.000, 60.000),
                                new Pose(36.000, 60.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                .build();

        PathChain line2 = builder
                .addPath(new BezierLine(new Pose(36.000, 60.000), new Pose(84.000, 60.000)))
                .setHeadingInterpolation(HeadingInterpolator.facingPoint(new Pose(84, 90, Math.toRadians(90))))
                .build();

        PathChain line3 = builder
                .addPath(
                        new BezierCurve(
                                new Pose(84.000, 60.000),
                                new Pose(84.000, 90.000),
                                new Pose(40.000, 80.000),
                                new Pose(60.000, 30.000),
                                new Pose(60.000, 24.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        int pathNum = 0;

        waitForStart();

        while(opModeIsActive()) {
            follower.update();
            //Tuning.drawCurrentAndHistory();

            if (!follower.isBusy()) {
                pathNum++;
                switch(pathNum){
                    case 0:
                        follower.followPath(line1);
                        break;
                    case 1:
                        follower.followPath(line2);
                        break;
                    case 2:
                        follower.followPath(line3);
                        break;
                }
            }

        }

    }
}