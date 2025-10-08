package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.Robot;
import org.firstinspires.ftc.teamcode.components.Shooter;
import org.firstinspires.ftc.teamcode.game.Controller;
import static org.firstinspires.ftc.teamcode.game.Controller.Button.*;

@TeleOp
public class TeleOpMain extends OpMode {

    Controller.Button TRANSFER_SHOOT = NORTH;
    Controller.Button INTAKE = SOUTH;
    Controller.Button REVERSE_INTAKE = WEST;

    Robot robot;
    
    double shooterCurrent;
    
    Shooter shooter;

    ElapsedTime shootTimer;

    protected Controller driver;
    protected Controller meta;

    int distance = 72;
    static int velocityTolerance = 50;

    private double drive = 0, turn = 0, strafe = 0, speedFactor = 1;

    /**
     * Position the bot is shooting from, used to find proper shooter velocity.
     * 0 - Far launch zone, at the intersection of the 2 lines, "tip of the triangle"
     * 1 - Near launch zone, at the intersection of the 2 lines, "tip of the triangle"
     * 2 - Near launch zone, closer to goal. 2 diagonal tile lengths away
     */
    int position = 0;

    static String[] positions = {"Far Zone - Intersection", "Near Zone - Intersection", "Near Zone - Close"};

    boolean intaking;
    boolean reverse;
    boolean shooting;

    @Override
    public void init() {
        robot   = new Robot(this, 1); //Red Alliance
        driver  = new Controller(gamepad1);
        meta    = new Controller(gamepad2);
        robot.getDriveTrain().calibrateOtos();
        robot.init();
        shooter = robot.getShooter1();
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        shootTimer = new ElapsedTime();
    }

    @Override
    public void start() {
        shooter.holdVelocity();
    }

    @Override
    public void loop() {
        driveBot();
        telemetry();
        robot.update();

        switch(position){
            case 0:
                distance = 0;
            case 1:
                distance = 1;
            case 2:
                distance = 2;
        }

        if(!shooting) {
            if (meta.isPressed(DPAD_UP)) position++;
            if (meta.isPressed(DPAD_DOWN)) position--;
            if (position > 2) position = 2;
            if (position < 0) position = 0;
        }

        shooterCurrent = shooter.getShooterCurrent();

        if(driver.isPressed(TRANSFER_SHOOT)){
//            shooter.shootAtDistance(robot.getAprilTag().getFtcPose(24).range);
//            shooter.holdVelocity();
//            robot.getTransfer1().transferUntilShot();
            if(!shooting) {
                shooter.setVelocity(shooter.velocityFromDistance(distance));
                shooting = true;
            }else{
                shooting = false;
            }
        }

        if(driver.isPressed(INTAKE)){
            reverse = false;
            intaking = !intaking;
            robot.getIntake().driveIntake(intaking ? 1 : 0);
        }

        if(driver.isPressed(REVERSE_INTAKE)){
            intaking = false;
            reverse = !reverse;
            robot.getIntake().driveIntake(intaking ? -0.5 : 0);
        }

        if(driver.isPressed(DPAD_UP)){
            speedFactor += 0.1;
        } else if (driver.isPressed(DPAD_DOWN)) {
            speedFactor -= 0.1;
        }
        
        if(shooting){
            if(shooter.getVelocity() < shooter.velocityFromDistance(distance) - velocityTolerance || shooter.getVelocity() > shooter.velocityFromDistance(distance) + velocityTolerance) {
                shootTimer.reset();
            }

            if(shootTimer.milliseconds() > 1000) {
                robot.getTransfer1().timedTransfer(1500);
                shooting = false;
            }
        }else{
            shooter.holdVelocity();
        }

    }

    @Override
    public void stop() {
        shooter.setZeroPowerBehavior(BRAKE);
        robot.stop();
        super.stop();
    }

    /**
     * Robot centric drive method
     */
    private void driveBot(){
        drive = driver.leftStickY();
        strafe = driver.leftStickX();
        turn = driver.rightStickX();

        robot.getDriveTrain().drive(drive, strafe, turn, speedFactor);
    }

    /**
     * Driver centric drive method
     */
    private void driveDriver(){
        drive = driver.leftStickY();
        strafe = driver.leftStickX();
        turn = driver.rightStickX();

        robot.getDriveTrain().driverRelative(drive,strafe,turn,1);
    }

    private void telemetry(){
        telemetry.addData("Position", positions[position]);
        telemetry.addData("Intaking", intaking);
        telemetry.addData("Shooting", shooting);
    }
}
