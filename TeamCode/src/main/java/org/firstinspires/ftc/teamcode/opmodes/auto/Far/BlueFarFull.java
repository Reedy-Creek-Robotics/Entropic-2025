package org.firstinspires.ftc.teamcode.opmodes.auto.Far;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class BlueFarFull extends FarFull{

    @Override
    public void buildPaths() {
        paths = new Paths(follower);
    }

    @Override
    public void initRobot(){
        alliance = true;
        super.initRobot();
    }

    public static class Paths extends FarFull.Paths{

        public Paths(Follower follower) {
            super(follower);
        }

        @Override
        public void createPaths(Follower follower) {
            shootPreload = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(54.000, 6.000),

                                    new Pose(54.000, 12.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            alignWithBallSet3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(54.000, 12.000),

                                    new Pose(42.000, 36.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            pickupBallSet3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(42.000, 36.000),

                                    new Pose(14.000, 36.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            shootBallSet3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(14.000, 36.000),

                                    new Pose(54.000, 12.000)
                            )
                    ).setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            alignWithBallSet2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(54.000, 12.000),

                                    new Pose(42.000, 60.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            pickupBallSet2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(42.000, 60.000),

                                    new Pose(15.000, 60.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            shootBallSet2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(15.000, 60.000),

                                    new Pose(54.000, 12.000)
                            )
                    ).setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            alignWithBallSet1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(54.000, 12.000),

                                    new Pose(42.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            pickupBallSet1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(42.000, 84.000),

                                    new Pose(15.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            shootBallSet1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(15.000, 84.000),

                                    new Pose(54.000, 12.000)
                            )
                    ).setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            parking = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(54.000, 12.000),

                                    new Pose(36.000, 12.000)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();
        }
    }
}
