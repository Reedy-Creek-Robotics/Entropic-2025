package org.firstinspires.ftc.teamcode.opmodes.auto.Far;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous
public class RedFarSingle extends FarSingle {

    @Override
    public void buildPaths() {
        paths = new Paths(follower);
    }

    @Override
    public void initRobot(){
        alliance = false;
        super.initRobot();
//        robot.getTurret().setTurretExtraMove(-2);
    }

    public static class Paths extends FarSingle.Paths {

        public Paths(Follower follower) {
            super(follower);
        }

        @Override
        public void createPaths(Follower follower) {
            alignWithBallSet3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(87.500, 8.500),

                                    new Pose(96.000, 36.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))

                    .build();

            pickupBallSet3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(96.000, 36.000),

                                    new Pose(134.000, 36.000)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            shootBallSet3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(134.000, 36.000),

                                    new Pose(88.000, 16.000)
                            )
                    ).setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            parking = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(88.000, 16.000),

                                    new Pose(118.000, 16.000)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();
        }
    }
}
