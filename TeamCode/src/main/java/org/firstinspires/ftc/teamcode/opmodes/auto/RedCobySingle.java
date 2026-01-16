package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

public class RedCobySingle extends CobySingle {

    @Override
    public void buildPaths() {
        paths = new Paths(follower);
    }

    public static class Paths extends CobySingle.Paths {

        public Paths(Follower follower) {
            super(follower);
        }

        @Override
        public void createPaths(Follower follower) {
            shootPreload = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(90.000, 6.000),
                                    new Pose(84.000, 12.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(270))
                    .build();

            alignWithBallSet3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(84.000, 12.000),
                                    new Pose(102.000, 36.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(0))
                    .build();

            pickupBallSet3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(102.000, 36.000),
                                    new Pose(122.000, 36.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            shootBallSet3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(122.000, 36.000),
                                    new Pose(84.000, 12.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            parking = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(84.000, 12.000),
                                    new Pose(108.000, 12.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(90))
                    .build();
        }
    }
}
