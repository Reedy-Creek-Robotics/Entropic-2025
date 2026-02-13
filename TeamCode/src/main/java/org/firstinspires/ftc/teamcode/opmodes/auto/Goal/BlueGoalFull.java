package org.firstinspires.ftc.teamcode.opmodes.auto.Goal;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class BlueGoalFull extends GoalFull{

    @Override
    public void buildPaths(){
        paths = new Paths(follower);
    }

    @Override
    public void initRobot(){
        alliance = true;
        super.initRobot();
        robot.getTurret().setTurretExtraMove(5);
    }

    public static class Paths extends GoalFull.Paths {

        @Override
        public void createPaths(Follower follower) {

            shootPreload = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(33.000, 138.000),

                                    new Pose(48.000, 96.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .setBrakingStrength(3)
                    .build();

            alignWithBallSet1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(48.000, 96.000),

                                    new Pose(48.000, 92.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            pickupBallSet1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(48.000, 92.000),

                                    new Pose(22.000, 92.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            shootBallSet1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(22.000, 92.000),

                                    new Pose(48.000, 96.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
//                    .setReversed()
//                    .setTangentHeadingInterpolation()
                    .setBrakingStart(3)
                    .setBrakingStrength(2)
                    .build();

            alignWithBallSet2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(48.000, 96.000),

                                    new Pose(48.000, 67.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            pickupBallSet2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(48.000, 67.000),

                                    new Pose(13.000, 67.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            shootBallSet2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(13.000, 67.000),
                                    new Pose(60.000, 67.000),
                                    new Pose(60.000, 108.000)
                            )
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .setBrakingStart(3)
                    .setBrakingStrength(2)
                    .build();

            alignWithBallSet3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(60.000, 108.000),

                                    new Pose(48.000, 42.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            pickupBallSet3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(48.000, 42.000),
                                    new Pose(15.000, 42.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();

            shootBallSet3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(15.000, 42.000),
                                    new Pose(48.000, 96.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .setBrakingStart(3)
                    .setBrakingStrength(2)
                    .build();

            parking = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(48.000, 96.000),
                                    new Pose(24.000, 96.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                    .build();
        }

        public Paths(Follower follower) {
            super(follower);
        }
    }

}



