package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

public abstract class GoalFull extends AutoMain {
    Paths paths;

    @Override
    public void initAuto(){
        robot.getEndoscope().setEnableArtifactManagement(false);
        robot.getTurret().setAutoAim(false);
        robot.getRobotContext().setAlliance(alliance);
        robot.getShooter().setAutoSpeed(false);
        follower.setStartingPose(paths.shootPreload.getPose(new PathChain.PathT(0, 0)));
    }

    @Override
    public void runPath() {

        if(opmodeTimer.getElapsedTime() > 28500){
            robot.stopAllCommands();
            follower.followPath(follower.pathBuilder().addPath(new BezierLine(
                    follower.getPose(),
                    paths.parking.endPose()
            )).build());
            pathState = 15;
        }

        switch(pathState){
            case 0:
                follower.followPath(paths.shootPreload, true);
                robot.getTurret().setTargetFromPose(paths.shootPreload.endPose());
                robot.getShooter().setVelocity(1260);
                pathState++;
                break;
            case 1:
            case 5:
            case 9:
            case 13:
                if(!follower.isBusy() && robot.isBusy()){
                    robot.getEndoscope().setEnableArtifactManagement(false);
                    robot.stopAllCommands();
                }
                if(!follower.isBusy() && !robot.isBusy()){
                    robot.getTransfer().rollersForTime(1, 1500);
                    robot.getIntake().runIntakeCommand(1);
                    robot.getTransfer().rollersForTime(1, 1000);
                    pathState++;
                }
                break;
            case 2:
                if(!robot.isBusy()){
                    robot.getIntake().setIntakePower(0);
                    robot.getTurret().setTargetFromPose(paths.shootBallSet1.endPose());
                    follower.followPath(paths.alignWithBallSet1, true);
                    pathState++;
                }
                break;
            case 3:
                if(!follower.isBusy()){
                    robot.getEndoscope().setEnableArtifactManagement(true);
                    robot.getIntake().setIntakePower(1);
                    follower.followPath(paths.pickupBallSet1, 0.75, true);
                    pathState++;
                }
                break;
            case 4:
                if(!follower.isBusy()){
                    robot.getIntake().setIntakePower(0);
//                    robot.getTransfer().rollersForTime(-1, 500);
                    follower.followPath(paths.shootBallSet1);
                    pathState++;
                }
                break;
            case 6:
                if(!robot.isBusy()){
                    robot.getIntake().setIntakePower(0);
                    robot.getTurret().setTargetFromPose(paths.shootBallSet2.endPose());
                    follower.followPath(paths.alignWithBallSet2, true);
                    pathState++;
                }
                break;
            case 7:
                if(!follower.isBusy()){
                    robot.getEndoscope().setEnableArtifactManagement(true);
                    robot.getIntake().setIntakePower(1);
                    follower.followPath(paths.pickupBallSet2, 0.75, true);
                    pathState++;
                }
                break;
            case 8:
                if(!follower.isBusy()){
                    robot.getIntake().setIntakePower(0);
//                    robot.getTransfer().rollersForTime(-1, 500);
                    follower.followPath(paths.shootBallSet2);
                    pathState++;
                }
                break;
            case 10:
                if(!robot.isBusy()){
                    robot.getIntake().setIntakePower(0);
                    robot.getTurret().setTargetFromPose(paths.shootBallSet3.endPose());
                    follower.followPath(paths.alignWithBallSet3, true);
                    pathState++;
                }
                break;
            case 11:
                if(!follower.isBusy()){
                    robot.getEndoscope().setEnableArtifactManagement(true);
                    robot.getIntake().setIntakePower(1);
                    follower.followPath(paths.pickupBallSet3, 0.75, true);
                    pathState++;
                }
                break;
            case 12:
                if(!follower.isBusy()){
                    robot.getIntake().setIntakePower(0);
//                    robot.getTransfer().rollersForTime(-1, 500);
                    follower.followPath(paths.shootBallSet3);
                    pathState++;
                }
                break;
            case 14:
                if(!robot.isBusy()){
                    robot.getIntake().setIntakePower(0);
                    follower.followPath(paths.parking);
                    pathState = 999;
                }
                break;
            case 15:
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
        public PathChain alignWithBallSet2;
        public PathChain pickupBallSet2;
        public PathChain shootBallSet2;
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
