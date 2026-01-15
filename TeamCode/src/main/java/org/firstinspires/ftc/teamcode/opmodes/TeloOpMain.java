package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.game.Controller.AnalogControl.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.components.Robot;

import org.firstinspires.ftc.teamcode.components.BaseComponent;
import org.firstinspires.ftc.teamcode.components.RobotContext;
import org.firstinspires.ftc.teamcode.components.Transfer;
import org.firstinspires.ftc.teamcode.components.Turret;
import org.firstinspires.ftc.teamcode.game.Controller;

@TeleOp(name = "Tele Op", group = "!!Main")
public class TeloOpMain extends OpMode {

    RobotContext robotContext;
    Robot robot;

    protected Controller driver;

    double drive, strafe, turn;

    Follower follower;
    
    Transfer transfer;
    Boolean serving = false;

    //Pose startPose = new Pose(112, 134.5, Math.toRadians(0)); // Start Pose of our robot.
    Pose largeZoneReset = new Pose(96, 96, Math.toRadians(90)); // share button
    Pose smallZoneReset = new Pose(24, 72, Math.toRadians(90)); // options button
    Pose startPose = largeZoneReset;

    @Override
    public void init() {
        robotContext = BaseComponent.createRobotContext(this);

        robot = new Robot(this, false);
        driver = new Controller(gamepad1);

        transfer = robot.getTransfer();
        
        robot.init();

        follower = robot.getDriveTrain().getFollower();

        follower.setStartingPose(startPose);
    }

    @Override
    public void loop() {
        drive = driver.analogValue(LEFT_STICK_Y);
        strafe = driver.analogValue(LEFT_STICK_X);
        turn = driver.analogValue(RIGHT_STICK_X);

        robot.getDriveTrain().drive(drive, strafe, turn);

        robot.getIntake().setIntakePower(driver.analogValue(RIGHT_TRIGGER) - driver.analogValue(LEFT_TRIGGER) + (driver.isButtonDown(Controller.Button.SOUTH) ? .2:0));

        /*if(driver.isButtonDown(Controller.Button.NORTH) || driver.isButtonDown(Controller.Button.RIGHT_BUMPER)){
            transfer.runFrontRoller(1);
        }
        else if(driver.isButtonDown(Controller.Button.SOUTH)){
            transfer.runFrontRoller(-1);
        }else{
            transfer.runFrontRoller(0);
        }

        if(driver.isButtonDown(Controller.Button.DPAD_UP) || driver.isButtonDown(Controller.Button.LEFT_BUMPER)){
            transfer.runRearRoller(1);
        }else if(driver.isButtonDown(Controller.Button.DPAD_DOWN)){
            transfer.runRearRoller(-1);
        }else{
            transfer.runRearRoller(0);
        }*/

        if(driver.isPressed(Controller.Button.SHARE)){
            robot.getDriveTrain().getFollower().setPose(largeZoneReset);

        }else if(driver.isPressed(Controller.Button.OPTIONS)){
            robot.getDriveTrain().getFollower().setPose(smallZoneReset);
        }

        //serve balls
        if(driver.isButtonDown(Controller.Button.SOUTH)){
            serving = true;
            transfer.runFrontRoller(1);
            transfer.runRearRoller(1);
        }
        //purge
        else if(driver.isButtonDown(Controller.Button.NORTH)){
            serving = true;
            transfer.runFrontRoller(-1);
            transfer.runRearRoller(-1);
            robot.stopAllCommands();
        }
        else{
            if(serving){
                transfer.runFrontRoller(0);
                transfer.runRearRoller(0);
                serving = false;
                transfer.setBallState(0);
            }
        }
        if(driver.isPressed(Controller.Button.EAST)){
            transfer.serveUntilShot(0);
        }



        //if(driver.isPressed(Controller.Button.TOUCH_PAD)){
        //    robot.getTurret().resetEncoder();
        //}

        telemetry.addData("alliance", robotContext.getAlliance());
        robot.update();
    }
}
