package org.firstinspires.ftc.teamcode.opmodes.auto.Goal;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.opmodes.auto.AutoMain;

public abstract class GoalFull extends AutoMain {
    Paths paths;

    @Override
    public void initAuto(){
        robot.getEndoscope().setEnableArtifactManagement(false);
        robot.getTurret().setAutoAim(false);
        robot.getRobotContext().setAlliance(alliance);
        robot.getShooter().setAutoSpeed(false);
        robot.setUseTelemetry(true);
        follower.setStartingPose(paths.shootPreload.getPose(new PathChain.PathT(0, 0)));
    }

    @Override
    public void runPath() {

//        if(opmodeTimer.getElapsedTime() > 29500 && pathState < 15){
//            pathState = 15;
//        }

        switch(pathState){
            case 0:
                follower.followPath(paths.shootPreload, true);
                robot.getTurret().setTargetFromPose(paths.shootPreload.endPose());
                robot.getShooter().setVelocity(robot.getShooter().velocityFromDistance(paths.shootPreload.endPose().distanceFrom(robot.getTurret().getTargetGoal())));
                pathState++;
                break;
            case 1:
                if(!follower.isBusy()){
                    robot.getDriveTrain().nothingForTime(1000);
                    robot.getTransfer().rollersForTime(1, 1750);
                    robot.getIntake().runIntakeCommand(1);
                    robot.getTransfer().rollersForTime(1, 1000);
                    pathState++;
                }
                break;
            case 2:
                if(robot.getShooter().getShootCount() >= 3){
                    robot.getShooter().resetShootCount();
                    robot.stopAllCommands();
                    robot.getDriveTrain().nothingForTime(500);
                }
                if(!robot.isBusy()){
                    robot.getShooter().resetShootCount();
                    robot.getTurret().setAutoAim(true);
                    robot.getIntake().setIntakePower(0);
                    robot.getTurret().setTargetFromPose(paths.shootBallSet1.endPose());
                    robot.getShooter().setVelocity(robot.getShooter().velocityFromDistance(paths.shootBallSet1.endPose().distanceFrom(robot.getTurret().getTargetGoal())));
                    follower.followPath(paths.alignWithBallSet1, true);
                    pathState++;
                }
                break;
            case 3:
                if(!follower.isBusy()){
                    robot.getTurret().setAutoAim(true);
                    robot.getEndoscope().setEnableArtifactManagement(true);
                    robot.getIntake().setIntakePower(1);
                    follower.followPath(paths.pickupBallSet1, 0.5, true);
                    pathState++;
                }
                break;
            case 4:
                if(!follower.isBusy()){
                    robot.getEndoscope().setEnableArtifactManagement(false);
                    robot.getTransfer().stopAllTransferCommands();
                    follower.followPath(paths.shootBallSet1);
                    pathState++;
                }
                break;
            case 5:
            case 9:
            case 13:
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
                if(robot.getShooter().getShootCount() >= 3){
                    robot.getShooter().resetShootCount();
                    robot.stopAllCommands();
                    robot.getDriveTrain().nothingForTime(500);
                }
                if(!robot.isBusy()){
                    robot.getShooter().setVelocity(robot.getShooter().velocityFromDistance(paths.shootBallSet2.endPose().distanceFrom(robot.getTurret().getTargetGoal())));
                    robot.getEndoscope().setEnableArtifactManagement(true);
                    robot.getTransfer().stopAllTransferCommands();
                    robot.getTransfer().runFrontRoller(0);
                    robot.getTransfer().runRearRoller(0);
                    robot.getTransfer().setBallState(0);
                    robot.getTransfer().setWaitForStateChange(false);


                    robot.getEndoscope().setEnableArtifactManagement(false);
                    robot.getTransfer().stopAllTransferCommands();
                    robot.getTurret().setTargetFromPose(paths.shootBallSet2.endPose());
                    follower.followPath(paths.alignWithBallSet2, true);
                    pathState++;
                }
                break;
            case 7:
                if(!follower.isBusy()){
                    robot.getEndoscope().setEnableArtifactManagement(true);
                    robot.getIntake().setIntakePower(1);
                    follower.followPath(paths.pickupBallSet2, 0.5, true);
                    pathState++;
                }
                break;
            case 8:
                if(!follower.isBusy()){

                    robot.getEndoscope().setEnableArtifactManagement(false);
                    robot.getTransfer().stopAllTransferCommands();
                    follower.followPath(paths.shootBallSet2);
                    pathState++;
                }
                break;
            case 10:
                if(robot.getShooter().getShootCount() >= 3){
                    robot.getShooter().resetShootCount();
                    robot.stopAllCommands();
                    robot.getDriveTrain().nothingForTime(500);
                }
                if(!robot.isBusy()) {
                    pathState = 16;
                }
                break;
//            case 10:
//               if(robot.getShooter().getShootCount() >= 3){
//                    robot.getShooter().resetShootCount();
//                    robot.stopAllCommands();
//                    robot.getDriveTrain().nothingForTime(500);
//                }
//                if(!robot.isBusy()){
//                    robot.getEndoscope().setEnableArtifactManagement(true);
//                    robot.getTransfer().runFrontRoller(0);
//                    robot.getTransfer().runRearRoller(0);
//                    robot.getTransfer().setBallState(0);
//                    robot.getTransfer().setWaitForStateChange(false);
//
//                    robot.getIntake().setIntakePower(0);
//                    robot.getTurret().setTargetFromPose(paths.shootBallSet3.endPose());
//                    follower.followPath(paths.alignWithBallSet3, true);
//                    pathState++;
//                }
//                break;
            case 11:
                if(!follower.isBusy()){
                    robot.getEndoscope().setEnableArtifactManagement(true);
                    robot.getIntake().setIntakePower(1);
                    follower.followPath(paths.pickupBallSet3, 0.5, true);
                    pathState++;
                }
                break;
            case 12:
                if(!follower.isBusy()){
                    robot.getEndoscope().setEnableArtifactManagement(false);
                    robot.getTransfer().stopAllTransferCommands();
                    follower.followPath(paths.shootBallSet3);
                    pathState++;
                }
                break;
            case 14:
                if(robot.getShooter().getShootCount() >= 3){
                    robot.stopAllCommands();
                }
                if(!robot.isBusy()){
                    robot.getEndoscope().setEnableArtifactManagement(true);
                    robot.getTransfer().runFrontRoller(0);
                    robot.getTransfer().runRearRoller(0);
                    robot.getTransfer().setBallState(0);
                    robot.getTransfer().setWaitForStateChange(false);
                    
                    pathState++;
                }
                break;
            case 15:
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
                pathState = 16;
            case 16:
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
