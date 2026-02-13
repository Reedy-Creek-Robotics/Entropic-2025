package org.firstinspires.ftc.teamcode.opmodes.auto.Far;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous
public class BlueFarSingle extends FarSingle {
    @Override
    public void buildPaths() {
        paths = new Paths(follower);
    }

    @Override
    public void initRobot() {
        alliance = true;
        super.initRobot();
//        robot.getTurret().setTurretExtraMove(2);
    }

    public static class Paths extends FarSingle.Paths{

        public Paths(Follower follower) {
            super(follower);
        }

        @Override
        public void createPaths(Follower follower) {
            alignWithBallSet3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(56.000, 8.000),

                                    new Pose(48.000, 36.000)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))

                    .build();

            pickupBallSet3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(48.000, 36.000),

                                    new Pose(10.000, 36.000)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();

            shootBallSet3 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(10.000, 36.000),

                                    new Pose(56.000, 16.000)
                            )
                    ).setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            parking = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(56.000, 16.000),

                                    new Pose(26.000, 16.000)
                            )
                    ).setTangentHeadingInterpolation()

                    .build();
        }
    }
}
