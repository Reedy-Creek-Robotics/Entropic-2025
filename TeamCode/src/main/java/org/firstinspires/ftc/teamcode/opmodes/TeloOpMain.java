package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.game.Controller.AnalogControl.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.components.Endoscope;
import org.firstinspires.ftc.teamcode.components.Robot;

import org.firstinspires.ftc.teamcode.components.BaseComponent;
import org.firstinspires.ftc.teamcode.components.RobotContext;
import org.firstinspires.ftc.teamcode.components.Transfer;
import org.firstinspires.ftc.teamcode.components.Turret;
import org.firstinspires.ftc.teamcode.components.UpgradedTransfer;
import org.firstinspires.ftc.teamcode.game.ColorValue;
import org.firstinspires.ftc.teamcode.game.Controller;

@TeleOp(name = "Tele Op", group = "!!Main")
public class TeloOpMain extends OpMode {

    RobotContext robotContext;
    Robot robot;

    protected Controller driver;
    protected Controller meta;
    boolean manualMode;
    ColorValue manualEnabledColor = new ColorValue(255, 0 ,0);
    ColorValue manualDisabledColor = new ColorValue(0, 140 ,70);

    double drive, strafe, turn;

    Follower follower;
    
    UpgradedTransfer transfer;
    Endoscope endoscope;
    Turret turret;
    Boolean serving = false;

    //Pose startPose = new Pose(112, 134.5, Math.toRadians(0)); // Start Pose of our robot.
    Pose largeZoneReset = new Pose(96, 96, Math.toRadians(90)); // share button
    Pose smallZoneReset = new Pose(72, 24, Math.toRadians(90)); // options button
    Pose startPose = largeZoneReset;

    @Override
    public void init() {
        robotContext = BaseComponent.createRobotContext(this);

        robot = new Robot(this, false);
        driver = new Controller(gamepad1);
        meta = new Controller(gamepad2);

        transfer = robot.getTransfer();
        endoscope = robot.getEndoscope();
        turret = robot.getTurret();
        
        robot.init();

        follower = robot.getDriveTrain().getFollower();

        follower.setPose(startPose);

//        robot.loadStateFromDisk();

    }

    @Override
    public void loop() {
        /* DRIVER CONTROLS */
        drive = driver.analogValue(LEFT_STICK_Y);
        strafe = driver.analogValue(LEFT_STICK_X);
        turn = driver.analogValue(RIGHT_STICK_X);
        //drive, strafe, and turning
        robot.getDriveTrain().drive(drive, strafe, turn);

        //intake, outake, purge -.5, and shoot .5
        robot.getIntake().setIntakePower(driver.analogValue(RIGHT_TRIGGER) - driver.analogValue(LEFT_TRIGGER) + (driver.isButtonDown(Controller.Button.SOUTH) ? 0.5:0) + (driver.isButtonDown(Controller.Button.NORTH) ? -0.5:0));


        //shoot run rollers up
        if(driver.isButtonDown(Controller.Button.SOUTH)){
            serving = true;
            robot.stopAllCommands();
            transfer.runFrontRoller(1);
            transfer.runRearRoller(1);
        }
        //purge run rollers out
        else if(driver.isButtonDown(Controller.Button.NORTH)){
            serving = true;
            transfer.runFrontRoller(-1);
            transfer.runRearRoller(-1);
            robot.stopAllCommands();
        }
        else if(driver.isButtonDown(Controller.Button.EAST)){
            serving = true;
            transfer.runRearRoller(1);
        }
        else if(driver.isButtonDown(Controller.Button.WEST)){
            serving = true;
            transfer.runFrontRoller(1);
        }
        else{
            if(serving){
                serving = false;
                transfer.runFrontRoller(0);
                transfer.runRearRoller(0);
                transfer.setBallState(0);
            }
        }

        if(driver.isPressed(Controller.Button.PS)){
            robot.getShooter().setAutoSpeed(!robot.getShooter().getAutoSpeed());
        }


        /* META CONTROLS */
        //toggle artifact management
        if(meta.isPressed(Controller.Button.WEST)){
            endoscope.setEnableArtifactManagement(!endoscope.getEnableArtifactManagement());
        }
        // set alliances
        if(meta.isPressed(Controller.Button.SHARE)){
            robotContext.alliance = true;
            robot.getTurret().setAlliance(true);
        }
        if(meta.isPressed(Controller.Button.OPTIONS)){
            robotContext.alliance = false;
            robot.getTurret().setAlliance(false);
        }
        //re-localize @ goal zone
        if(meta.isPressed(Controller.Button.NORTH)){
            robot.getDriveTrain().getFollower().setPose(largeZoneReset);
        //re-localize @ far zone
        }if(meta.isPressed(Controller.Button.SOUTH)){
            robot.getDriveTrain().getFollower().setPose(smallZoneReset);
        }
        //manual mode toggle
        if(meta.isPressed(Controller.Button.PS)){
            if (manualMode){
                turret.setAutoAim(false);
                turret.setAutoMove(false);
                turret.getTurretMotor().setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                manualMode = false;
            }
            else {
                turret.getTurretMotor().setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                turret.setAutoAim(true);
                turret.setAutoMove(true);

                manualMode = true;
            }
        }

        //manual control turret
        if(manualMode){
            turret.getTurretMotor().setPower(Math.pow(-meta.leftStickX(), 3)/2);
        }


        telemetry.addData("alliance", robotContext.getAlliance() ? "Blue" : "Red");
        telemetry.addData("MANUAL MODE", manualMode);
        meta.setLED(manualMode ? manualEnabledColor : manualDisabledColor, 500);
        robot.update();
    }
}
