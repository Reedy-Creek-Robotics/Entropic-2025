package org.firstinspires.ftc.teamcode.opmodes.auto.Goal;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class RedGoalFull extends GoalFull{

    @Override
    public void buildPaths(){
        paths = new Paths(follower);
    }

    @Override
    public void initRobot() {
        alliance = false;
        super.initRobot();
    }

    public static class Paths extends GoalFull.Paths{
        public Paths(Follower follower) {
            super(follower);
        }

        @Override
        public void createPaths(Follower follower) {
            shootPreload = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(114.000, 137.000),
                                    new Pose(96.000, 96.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(10))
                    .build();

            alignWithBallSet1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(96.000, 96.000),
                                    new Pose(102.000, 88.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(10), Math.toRadians(10))
                    .setBrakingStart(3)
                    .setBrakingStrength(2)
                    .build();

            pickupBallSet1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(102.000, 88.000),
                                    new Pose(127.000, 88.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(10), Math.toRadians(10))
                    .build();

            shootBallSet1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(130.000, 88.000),
                                    new Pose(96.000, 96.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(10), Math.toRadians(0))
                    .build();

            alignWithBallSet2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(96.000, 96.000),
                                    new Pose(102.000, 63.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(10))
                    .setBrakingStart(3)
                    .setBrakingStrength(2)
                    .build();

            pickupBallSet2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(102.000, 63.000),
                                    new Pose(130.000, 63.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(10), Math.toRadians(10))
                    .build();

            shootBallSet2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(130.000, 63.000),
                                    new Pose(96.000, 96.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(10), Math.toRadians(0))
                    .build();

            alignWithBallSet3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(96.000, 96.000),
                                    new Pose(102.000, 44.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(10))
                    .setBrakingStart(3)
                    .setBrakingStrength(2)
                    .build();

            pickupBallSet3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(102.000, 44.00),
                                    new Pose(130.000, 44.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(10), Math.toRadians(10))
                    .build();

            shootBallSet3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(130.000, 44.000),
                                    new Pose(96.000, 96.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(10), Math.toRadians(0))
                    .build();

            parking = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(96.000, 96.000),
                                    new Pose(120, 96)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

        }
    }

}



