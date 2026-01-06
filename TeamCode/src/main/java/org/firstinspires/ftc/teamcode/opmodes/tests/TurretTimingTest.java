package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.game.Controller;
import org.firstinspires.ftc.teamcode.util.ArrayUtil;

import java.util.Arrays;
import java.util.Collections;
import java.util.List;

@Autonomous
@Disabled
public class TurretTimingTest extends LinearOpMode {

    ElapsedTime actionTimer;

    /**
     * In rpm
     */
    int baseMotorSpeed = 1150;
    double drivePulleyTeeth = 24;
    double turretPulleyTeeth = 134;

    double gearRatio = turretPulleyTeeth / drivePulleyTeeth;

    /**
     * In rpm
     */
    static int[] speeds = {30, 43, 60, 84, 117, 223, 312, 435, 1150, 1620, 6000};
    /**
     * In kg.cm
     */
    static double[] torques = {250.0, 185.0, 133.2, 93.6, 68.4, 38.0, 24.3, 18.7, 7.9, 5.4, 1.5};
    static double[] ticksPerRevs = {5281.1, 3895.9, 2786.2, 1993.6, 1425.1, 751.8, 537.7, 384.5, 145.1, 103.8, 28.0};


     /**
     * In kg.cm
     */
    double baseMotorTorque;
    double baseTicksPerRev;
    double baseTicksPerDeg;

    /**
     * Effective speed accounting for the gear ratio <br> Measured in RPM
     */
    double effectiveSpeed;
    /**
     * Effective torque accounting for the gear ratio <br> Measured in kg.cm
     */
    double effectiveTorque;
    double effectiveTicksPerRev;
    double effectiveTicksPerDeg;

    double targetPos = 0;
    int tolerance = 10; //ticks

    Controller controller;

    DcMotorEx turretMotor;

    double[] times = {};
    int[] positions;
    double[] powers = {1.0, 1.0, 1.0, 1.0, 1.0};

    @Override
    public void runOpMode() throws InterruptedException {

        actionTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.MONOSPACE);

        int selection = 0;
        int rpmIndex = 8;

        controller = new Controller(gamepad1);

        while(opModeInInit()){
            switch(selection) {
                case 0:
                    telemetry.addLine("Select Motor Rpm | Dpad Up/Down");
                    if(controller.isPressed(Controller.Button.DPAD_UP)){
                        rpmIndex++;
                    }

                    if(controller.isPressed(Controller.Button.DPAD_DOWN)){
                        rpmIndex--;
                    }
                    
                    if(rpmIndex > speeds.length -1) rpmIndex = speeds.length-1;
                    if(rpmIndex < 0) rpmIndex = 0;
                case 1:
                    telemetry.addLine("Select Drive Pulley Teeth Count | Dpad Up/Down");
                    if(controller.isPressed(Controller.Button.DPAD_UP)){
                        drivePulleyTeeth++;
                    }

                    if(controller.isPressed(Controller.Button.DPAD_DOWN)){
                        drivePulleyTeeth--;
                    }
                case 2:
                    telemetry.addLine("Select Turret Pulley Teeth Count | Dpad Up/Down");
                    if(controller.isPressed(Controller.Button.DPAD_UP)){
                        turretPulleyTeeth++;
                    }

                    if(controller.isPressed(Controller.Button.DPAD_DOWN)){
                        turretPulleyTeeth--;
                    }
                case 3:
                    telemetry.addLine("Select Movement Tolerance Ticks | Dpad Up/Down");
                    if(controller.isPressed(Controller.Button.DPAD_UP)){
                        tolerance++;
                    }

                    if(controller.isPressed(Controller.Button.DPAD_DOWN)){
                        tolerance--;
                    }
                default:
                    telemetry.addLine("Done Selecting - Start OpMode to test");
            }

            if(selection <= 3) telemetry.addLine("Press CROSS to advance.");
            telemetry.addLine();
            telemetry.addData("Motor Rpm", speeds[rpmIndex]);
            telemetry.addData("Drive Pulley Teeth", drivePulleyTeeth);
            telemetry.addData("Turret Pulley Teeth", turretPulleyTeeth);
            telemetry.addData("Gear Ratio", turretPulleyTeeth/drivePulleyTeeth);
            telemetry.addData("Tolerance", tolerance);
            telemetry.update();

            if(controller.isPressed(Controller.Button.CROSS)){
                selection++;
            }
        }

        waitForStart();

        updateInformation(rpmIndex);

        // GO TO 0
        commandPosition(0, 0.2);
        waitMotor(true);

        for(int i = 0; i < positions.length; i++){
            commandPosition(positions[i], powers[i]);
            waitMotor();
            commandZero();
            waitMotor(true);
        }

        for(int i = 0; i < positions.length; i++){
            commandPosition(positions[i] * -1, powers[i] * -1);
            waitMotor();
            commandZero();
            waitMotor(true);
        }

        telemetry.addLine("RESULTS: MILLISECONDS | TICKS | TICKS / MILLISECONDS");

        for(int i = 0; i < times.length; i++){
            telemetry.addLine(String.format("%d:  %4.4f  | % 4d  |  %4.4f", i+1, times[i], i < positions.length ? positions[i] : positions[i-positions.length], i < positions.length ? positions[i] : positions[i-positions.length] / times[i]));
        }

        telemetry.update();

        while(opModeIsActive());
    }
    
    private void updateInformation(int rpmIndex){
        baseMotorSpeed = speeds[rpmIndex];
        gearRatio = turretPulleyTeeth / drivePulleyTeeth;
        int motorType = ArrayUtil.findIndexOfItem(speeds, baseMotorSpeed);
        
        baseMotorTorque = torques[motorType];
        baseTicksPerRev = ticksPerRevs[motorType];
        baseTicksPerDeg =  baseTicksPerRev / 360;
        
        effectiveSpeed = baseMotorSpeed / gearRatio;
        effectiveTorque = baseMotorSpeed * gearRatio;
        effectiveTicksPerRev = baseTicksPerRev * gearRatio;
        effectiveTicksPerDeg = effectiveTicksPerRev / 360;

        positions = new int[]{(int) effectiveTicksPerRev, (int) baseTicksPerRev, (int) (effectiveTicksPerDeg * 45), (int) (effectiveTicksPerDeg * 90), (int) (effectiveTicksPerDeg * 180)};
    }

    private void waitMotor(boolean zero){
        actionTimer.reset();
        while(opModeIsActive() && turretMotor.isBusy()){
            telemetry.addData("Target", targetPos);
            telemetry.addData("Pos", turretMotor.getCurrentPosition());
            telemetry.addData("Power", turretMotor.getPower());
            telemetry.addData("Velocity", turretMotor.getVelocity());
            telemetry.addData("Timer", actionTimer.time());
            telemetry.update();
        }
        if(!zero) times = ArrayUtil.append(times, actionTimer.time());
    }

    private void waitMotor(){
        waitMotor(false);
    }

    private void commandPosition(int ticks, double power){
        turretMotor.setTargetPosition(ticks);
        turretMotor.setPower(power);
    }

    private void commandZero(){
        commandPosition(0, 0.2);
    }
}
