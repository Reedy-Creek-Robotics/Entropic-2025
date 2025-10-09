package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.Intake;
import org.firstinspires.ftc.teamcode.components.Robot;
import org.firstinspires.ftc.teamcode.components.Shooter;
import org.firstinspires.ftc.teamcode.components.Transfer;
import org.firstinspires.ftc.teamcode.game.ColorValue;
import org.firstinspires.ftc.teamcode.game.Controller;
import static org.firstinspires.ftc.teamcode.game.Controller.Button.*;

@TeleOp
public class TeleOpMain extends OpMode {

    Controller.Button TRANSFER_SHOOT = NORTH;
    Controller.Button INTAKE = SOUTH;
    Controller.Button REVERSE_INTAKE = WEST;
    Controller.Button REVERSE_ROLLERS = EAST;

    Robot robot;
    
    double shooterCurrent;
    
    Shooter shooter;
    Transfer transfer;
    Intake intake;

    ElapsedTime shootTimer;

    protected Controller driver;
    protected Controller meta;

    int velocity;
    int distance = 72;
    static int velocityTolerance = 50;

    private double drive = 0, turn = 0, strafe = 0, speedFactor = 1;

    /**
     * Position the bot is shooting from, used to find proper shooter velocity.
     * 0 - Far launch zone, at the intersection of the 2 lines, "tip of the triangle"
     * 1 - Near launch zone, at the intersection of the 2 lines, "tip of the triangle"
     * 2 - Near launch zone, closer to goal. ~1.5 diagonal tile lengths away
     */
    int position = 0;

    static String[] positions = {"Far Zone - Intersection", "Near Zone - Intersection", "Near Zone - Close"};

    boolean intaking;
    boolean reverseIntake;
//    boolean reverseRollers;
//    boolean shooting;
//    boolean rollerOverride = true;

    @Override
    public void init() {
        robot   = new Robot(this, 1); //Red Alliance
        driver  = new Controller(gamepad1);
        meta    = new Controller(gamepad2);
        robot.getDriveTrain().calibrateOtos();
        robot.init();
        shooter = robot.getShooter1();
        transfer = robot.getTransfer1();
        intake = robot.getIntake();
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);
        shootTimer = new ElapsedTime();
    }

    @Override
    public void start() {
        shooter.holdVelocity();
    }

    @Override
    public void loop() {
        velocity = shooter.velocityFromDistance(distance);
        driveBot();
        telemetry();
        robot.update();

        if(position == 0){
            distance = 118;
        }else if(position == 1){
            distance = 80;
        }else if(position == 2){
            distance = 46;
        }

        shooterCurrent = shooter.getShooterCurrent();

//        if(driver.isPressed(TRANSFER_SHOOT)){
//            shooter.shootAtDistance(robot.getAprilTag().getFtcPose(24).range);
//            shooter.holdVelocity();
//            transfer.transferUntilShot();
//            if(!shooting) {
//                shooter.setVelocity(velocity);
//                shooting = true;
//            }else{
//                shooting = false;
//            }
//        }

//        if(driver.isPressed(LEFT_STICK_BUTTON)){
//            rollerOverride = !rollerOverride;
//        }

        if(driver.isPressed(INTAKE)){
            reverseIntake = false;
            intaking = !intaking;
            intake.driveIntake(intaking ? 1 : 0);
            transfer.runRoller1(1);
        }

        if(driver.isPressed(REVERSE_INTAKE)){
            intaking = false;
            reverseIntake = !reverseIntake;
            intake.driveIntake(reverseIntake ? -0.5 : 0);
        }

//        if(driver.isPressed(REVERSE_ROLLERS)){
//            reverseRollers = !reverseRollers;
//        }

        if(driver.isPressed(DPAD_UP)){
            speedFactor += 0.1;
        } else if (driver.isPressed(DPAD_DOWN)) {
            speedFactor -= 0.1;
        }

        /*
        if(shooting){
            if(rollerOverride) {
                transfer.runRollers(driver.leftTrigger());
            }else{

                if(shooter.getVelocity() < shooter.velocityFromDistance(distance) - velocityTolerance || shooter.getVelocity() > shooter.velocityFromDistance(distance) + velocityTolerance) {
                    shootTimer.reset();
                    telemetry.addLine("Shooter out of range");
                }

                if(shootTimer.milliseconds() > 1000) {
                    telemetry.setAutoClear(false);
                    telemetry.addLine("Transferring...");
                    telemetry.update();
                    telemetry.setAutoClear(true);
                    transfer.timedTransfer(1500);
                    shooting = false;
                }
            }

            if(shootTimer.milliseconds() > 6000){
                shooting = false;
            }
        }else{
            shooter.holdVelocity();
            transfer.runRollers(reverseRollers ? -0.5 : 0);
            if (meta.isPressed(DPAD_UP)) position++;
            if (meta.isPressed(DPAD_DOWN)) position--;
            if (position > 2) position = 2;
            if (position < 0) position = 0;
        }
        */

        shooter.setVelocity(velocity);
        if(driver.isButtonDown(LEFT_BUMPER) || driver.isButtonDown(RIGHT_BUMPER)) {
            transfer.runRoller1(driver.isButtonDown(LEFT_BUMPER) ? -0.5 : 0);
            transfer.runRoller2(driver.isButtonDown(RIGHT_BUMPER) ? -0.5 : 0);
        }else if(intaking){
            transfer.runRoller1(1);
            transfer.runRoller2(driver.leftTrigger() > 0.1 ? driver.leftTrigger() : -1);
        }
        else{
            transfer.runRollers(driver.leftTrigger() - driver.rightTrigger());
        }
        if (meta.isPressed(DPAD_UP)) position++;
        if (meta.isPressed(DPAD_DOWN)) position--;
        if (position > 2) position = 2;
        if (position < 0) position = 0;
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
        telemetry.addData("Reverse Intake", reverseIntake);
        if(shooter.getVelocity() < velocity - velocityTolerance || shooter.getVelocity() > velocity + velocityTolerance) {
            shootTimer.reset();
        }

        // If the velocity is within the tolerance for 750ms, run shooting code.
        if(shootTimer.milliseconds() > 750) {
            telemetry.addLine("STEADY VELOCITY");
            setLedAndRumble(true);
        }else{
            setLedAndRumble(false);
        }

//        telemetry.addData("Reverse Rollers", reverseRollers);
//        telemetry.addData("Shooting", shooting);
    }

    /**
     * Sets the driver and meta controllers LED and rumble to indicate the shooter is ready
     * @param activate Whether to make controllers indicate ready to shoot
     */
    private void setLedAndRumble(boolean activate){
        driver.setLED(new ColorValue(activate ? 0 : 255, activate ? 255 : 0, 0), 99999);
        meta.setLED(new ColorValue(activate ? 0 : 255, activate ? 255 : 0, 0), 99999);
        driver.rumble(activate ? 0.2 : 0, activate ? 0.2 : 0, 99999);
    }
}