package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.Robot;
import org.firstinspires.ftc.teamcode.game.Controller;

@TeleOp
public class TeleOpMain extends OpMode {

    Controller.Button TRANSFER_SHOOT = Controller.Button.Y;
    Controller.Button INTAKE = Controller.Button.A;

    Robot robot;

    protected Controller driver;

    private double drive = 0, turn = 0, strafe = 0, speedFactor = 1;

    boolean intaking;

    @Override
    public void init() {
        robot = new Robot(this, 1); //Red Alliance
        driver = new Controller(gamepad1);
        robot.getDriveTrain().calibrateOtos();
        robot.init();
    }

    @Override
    public void start() {
        robot.getShooter1().holdVelocity();
    }

    @Override
    public void loop() {
        driveDriver();

        if(driver.isPressed(TRANSFER_SHOOT)){
            robot.getShooter1().shootAtDistance(robot.getAprilTag().getFtcPose(24).range);
            robot.getTransfer1().transferUntilShot();
            robot.getShooter1().holdVelocity();
        }

        if(driver.isPressed(INTAKE)){
            intaking = !intaking;
            robot.getIntake().driveIntake(intaking ? 1 : 0);
        }

        if(driver.isPressed(Controller.Button.DPAD_UP)){
            speedFactor += 0.1;
        } else if (driver.isPressed(Controller.Button.DPAD_DOWN)) {
            speedFactor -= 0.1;
        }

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
}
