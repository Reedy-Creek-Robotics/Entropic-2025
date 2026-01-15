package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class BlueGoalSingle extends GoalSingle{

    @Override
    public void buildPaths(){
        paths = new Paths(follower);
    }

    public static class Paths extends GoalSingle.Paths{

        public Paths(Follower follower){
            super(follower);
        }

        @Override
        public void createPaths(Follower follower){
            shootPreload = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(33.000, 138.000),

                                    new Pose(48.000, 96.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(90))

                    .build();

            alignWithBallSet1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(48.000, 96.000),

                                    new Pose(42.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))

                    .build();

            pickupBallSet1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(42.000, 84.000),

                                    new Pose(22.000, 84.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            shootBallSet1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(22.000, 84.000),

                                    new Pose(48.000, 96.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();

            parking = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(48.000, 96.000),

                                    new Pose(48.000, 60.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                    .build();
        }
    }
}



