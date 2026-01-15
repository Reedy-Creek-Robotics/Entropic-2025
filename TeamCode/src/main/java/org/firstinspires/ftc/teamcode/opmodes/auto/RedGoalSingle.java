package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class RedGoalSingle extends GoalSingle{

    @Override
    public void buildPaths(){
        paths = new Paths(follower);
    }

    public static class Paths extends GoalSingle.Paths{
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
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(270))

                    .build();

            alignWithBallSet1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(96.000, 96.000),

                                    new Pose(102.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(270), Math.toRadians(0))

                    .build();

            pickupBallSet1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(102.000, 84.000),

                                    new Pose(122.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            shootBallSet1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(122.000, 84.000),

                                    new Pose(96.000, 96.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();

            parking = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(96.000, 96.000),

                                    new Pose(96.000, 60.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))

                    .build();
        }
    }

}



