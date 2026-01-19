package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;

public abstract class FarSingle extends AutoMain {
    Paths paths;

    @Override
    public void initAuto() {
        robot.getEndoscope().setEnableArtifactManagement(false);
        robot.getTurret().setAutoAim(false);
        robot.getShooter().setAutoSpeed(false);
        follower.setStartingPose(paths.shootPreload.getPose(new PathChain.PathT(0, 0)));
    }

    @Override
    public void runPath() {
        switch(pathState){
            case 0:
                follower.followPath(paths.shootPreload, true);
                robot.getTurret().setTargetFromPose(paths.shootPreload.endPose());
                robot.getShooter().setVelocity(1386);
                pathState++;
                break;
            case 1:
            case 5:
                if(!follower.isBusy() && robot.isBusy()){
                    robot.getEndoscope().setEnableArtifactManagement(false);
                    robot.stopAllCommands();
                }
                if(!follower.isBusy() && !robot.isBusy()){
                    robot.getTransfer().rollersForTime(0.75, 1500);
                    robot.getIntake().runIntakeCommand(0.5);
                    robot.getTransfer().rollersForTime(1, 1000);
                    pathState++;
                }
                break;
            case 2:
                if(!robot.isBusy()){
                    robot.getIntake().setIntakePower(0);
                    robot.getTurret().setTargetFromPose(paths.shootBallSet3.endPose());
                    robot.getShooter().setVelocity(1386);
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
                    follower.followPath(paths.shootBallSet3);
                    pathState++;
                }
                break;
            case 6:
                if(!robot.isBusy()){
                    robot.getIntake().setIntakePower(0);
                    follower.followPath(paths.parking);
                    pathState++;
                }
                break;
            case 7:
                if(!follower.isBusy()){
                    robot.stopAllCommands();
                    running = false;
                }
        }
    }

    public abstract static class Paths{
        public PathChain shootPreload;
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