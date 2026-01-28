package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.MotorControlAlgorithm;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.components.DualShooter;
import org.firstinspires.ftc.teamcode.components.Robot;
import org.firstinspires.ftc.teamcode.components.Shooter;
import org.firstinspires.ftc.teamcode.game.Controller;

//@Disabled
@Configurable
@TeleOp
public class ShooterTester extends OpMode {

    Controller controller;

    DualShooter shooter;

    Robot robot;

    VoltageSensor voltage;

    TelemetryManager panelsTelem;

    double power = 1500;
    double avgInactiveCurrent = 1;

    boolean reversed = true;
    boolean active;

    int velocityTolerance = 50;

    ElapsedTime shootTimer;

    static PIDFCoefficients pidf = new PIDFCoefficients(600, 3, 0, 0, MotorControlAlgorithm.PIDF);

//    private static double SHOOTER_CURRENT_THRESHOLD = 0.5;

    @Override
    public void init() {
        robot = new Robot(this, false);
        robot.init();

        shooter = robot.getShooter();

        controller = new Controller(gamepad1);
        voltage = hardwareMap.voltageSensor.iterator().next();

        shootTimer = new ElapsedTime();
        panelsTelem = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    @Override
    public void loop() {
        telemetry.addLine("Controls:");
        telemetry.addLine("NORTH to start");
        telemetry.addLine("SOUTH to run transfer up");
        telemetry.addLine("LEFT STICK to reverse");
        telemetry.addLine("DPAD UP/DOWN to change power x100");
        telemetry.addLine("DPAD LEFT/RIGHT to change power x20");
        telemetry.addLine("------------------------");

        shooter.setPIDFCoefficients(pidf);

        if(controller.isPressed(Controller.Button.DPAD_UP)){
            power += 100;
        }else if(controller.isPressed(Controller.Button.DPAD_DOWN)){
            power -= 100;
        }

        if(controller.isPressed(Controller.Button.DPAD_RIGHT)){
            power += 20;
        }else if(controller.isPressed(Controller.Button.DPAD_LEFT)){
            power -= 20;
        }

        if(controller.isButtonDown(Controller.Button.SOUTH)){
            robot.getTransfer().runFrontRoller(1);
            robot.getTransfer().runRearRoller(1);
        }else{
            robot.getTransfer().runFrontRoller(0);
            robot.getTransfer().runRearRoller(0);
        }

        if(power > 2240) power = 2240;
        if(power < 0) power = 0;

        if(controller.isPressed(Controller.Button.NORTH)) {
            active = !active;
        }

        if(controller.isPressed(Controller.Button.LEFT_STICK_BUTTON)){
            reversed = !reversed;
        }

        panelsTelem.addData("Active", active);
        telemetry.addData("Reversed", reversed);
        panelsTelem.addData("Set Power (tps)", power);

        if(active){
            shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            shooter.setDirection(reversed ? DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
            shooter.setVelocity((int) power);
        }else{
            shooter.setVelocity(0);
            shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        /*

         */

        panelsTelem.addData("Velocity", shooter.getVelocity());
        telemetry.addData("Current", shooter.getCombinedCurrent());
        telemetry.addData("Voltage", voltage.getVoltage());
        telemetry.addData("Avg Inactive", avgInactiveCurrent);
        telemetry.addData("Shoot Timer", shootTimer.milliseconds());

        // If the velocity is outside the tolerance, reset the timer
        if(shooter.getVelocity() < power - velocityTolerance || shooter.getVelocity() > power + velocityTolerance) {
            shootTimer.reset();
        }

        panelsTelem.update(telemetry);
    }

//    private boolean isShot(){
//        return shooter.getCurrent(CurrentUnit.AMPS) > baseline + SHOOTER_CURRENT_THRESHOLD || timeoutTimer.milliseconds() > TIMEOUT_THRESHOLD;
//    }
}