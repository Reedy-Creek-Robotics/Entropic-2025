package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;

public abstract class GoalSingle extends AutoMain {
    Paths paths;

    @Override
    public void initAuto(){
        robot.getEndoscope().setEnableArtifactManagement(false);
        follower.setStartingPose(paths.shootPreload.getPose(new PathChain.PathT(0, 0)));
    }

    @Override
    public void runPath() {
        switch(pathState){
            case 0:
                follower.followPath(paths.shootPreload, true);
                pathState = 1;
                break;
            case 1:
                if(!follower.isBusy()){
                    robot.getIntake().setIntakePower(0.25);
                    robot.getTransfer().rollersForTime(1, 2500);
                    pathState = 2;
                }
                break;
            case 2:
                if(!robot.isBusy()){
                    robot.getIntake().setIntakePower(0);
                    follower.followPath(paths.alignWithBallSet1, true);
                    pathState = 3;
                }
                break;
            case 3:
                if(!follower.isBusy()){
                    robot.getEndoscope().setEnableArtifactManagement(true);
                    robot.getIntake().setIntakePower(1);
                    follower.followPath(paths.pickupBallSet1, 0.5, true);
                    pathState = 4;
                }
                break;
            case 4:
                if(!follower.isBusy()){
                    robot.getIntake().setIntakePower(0);
                    follower.followPath(paths.shootBallSet1);
                    pathState = 5;
                }
                break;
            case 5:
                if(!follower.isBusy()){
                    robot.getIntake().setIntakePower(0.25);
                    robot.getTransfer().rollersForTime(1, 2500);
                    pathState = 6;
                }
                break;
            case 6:
                if(!robot.isBusy()){
                    robot.getIntake().setIntakePower(0);
                    follower.followPath(paths.parking);
                    pathState = 7;
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
        public PathChain alignWithBallSet1;
        public PathChain pickupBallSet1;
        public PathChain shootBallSet1;
        public PathChain parking;

        public Paths(Follower follower){
            createPaths(follower);
        }

        public abstract void createPaths(Follower follower);
    }
}
