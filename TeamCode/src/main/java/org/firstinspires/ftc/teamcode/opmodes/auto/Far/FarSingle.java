package org.firstinspires.ftc.teamcode.opmodes.auto.Far;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.opmodes.auto.AutoMain;

public abstract class FarSingle extends AutoMain {
    Paths paths;

    @Override
    public void initAuto() {
        robot.getEndoscope().setEnableArtifactManagement(false);
        robot.getTurret().setAutoAim(false);
        robot.getRobotContext().setAlliance(alliance);
        robot.getShooter().setAutoSpeed(false);
        robot.setUseTelemetry(true);
        follower.setStartingPose(paths.alignWithBallSet3.getPose(new PathChain.PathT(0, 0)));
    }

    @Override
    public void runPath() {

        if(opmodeTimer.getElapsedTime() > 29500 && pathState < 15){
            pathState = 6;
        }

        switch(pathState){
            case 0:
                robot.getTurret().setTargetFromPose(paths.shootBallSet3.endPose());
                robot.getShooter().setVelocity(robot.getShooter().velocityFromDistance(paths.shootBallSet3.endPose().distanceFrom(robot.getTurret().getTargetGoal())));
                pathState++;
                break;
            case 1:
                if(!robot.getShooter().isBusy()){
//                    robot.getDriveTrain().nothingForTime(1000);
                    robot.getTransfer().rollersForTime(1, 1750);
                    robot.getIntake().runIntakeCommand(1);
                    robot.getTransfer().rollersForTime(1, 1000);
                    pathState++;
                }
                break;
            case 2:
                if(!robot.isBusy()){
                    robot.getIntake().setIntakePower(0);
                    robot.getTurret().setTargetFromPose(paths.shootBallSet3.endPose());
                    follower.followPath(paths.alignWithBallSet3, true);
                    pathState++;
                }
                break;
            case 3:
                if(!follower.isBusy()){
                    robot.getEndoscope().setEnableArtifactManagement(true);
                    robot.getIntake().setIntakePower(1);
                    follower.followPath(paths.pickupBallSet3, 0.5, true);
                    pathState++;
                }
                break;
            case 4:
                if(!follower.isBusy()){
                    robot.getIntake().setIntakePower(0);
                    robot.getEndoscope().setEnableArtifactManagement(false);
                    robot.getTransfer().stopAllTransferCommands();
                    follower.followPath(paths.shootBallSet3);
                    pathState++;
                }
                break;
            case 5:
                if(!follower.isBusy()){
                    robot.getEndoscope().setEnableArtifactManagement(false);
                    robot.getDriveTrain().nothingForTime(1000);
                    robot.getTransfer().rollersForTime(1, 2000);
                    robot.getIntake().runIntakeCommand(1);
                    robot.getTransfer().rollersForTime(1, 2000);
                    pathState++;
                }
                break;
            case 6:
                robot.getIntake().setIntakePower(0);
                robot.stopAllCommands();
                robot.getTransfer().stopAllTransferCommands();
                follower.followPath(
                        follower.pathBuilder().addPath(
                                new BezierLine(
                                        follower.getPose(),
                                        paths.parking.endPose()
                                )
                        ).build()
                );
                pathState++;
            case 7:
                if(!follower.isBusy()){
                    robot.stopAllCommands();
                    running = false;
                }
        }
    }

    public abstract static class Paths{
        public PathChain alignWithBallSet3;
        public PathChain pickupBallSet3;
        public PathChain shootBallSet3;
        public PathChain parking;

        public Paths(Follower follower){
            createPaths(follower);
        }

        public abstract void createPaths(Follower follower);
    }
}