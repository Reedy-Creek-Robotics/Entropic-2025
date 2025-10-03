package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake extends BaseComponent{

    private DcMotorEx intakeMotor;

    public Intake(RobotContext context) {
        super(context);
    }

    @Override
    public void init() {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intakeMotor");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setMode(DcMotor.RunMode runMode) {
            intakeMotor.setMode(runMode);

    }

    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
            intakeMotor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );

        intakeMotor.setPIDFCoefficients(runMode, compensatedCoefficients);
    }

    public void setPIDFCoefficients(PIDFCoefficients coefficients) {
        setPIDFCoefficients(intakeMotor.getMode(), coefficients);
    }

    public void driveIntake(double power){
        intakeMotor.setPower(power);
    }

    public void driveIntake(int velocity){
        intakeMotor.setVelocity(velocity);
    }

    public void timedIntake(double power, double time){
        executeCommand(new TimedIntake(power, time));
    }

    public class TimedIntake implements Command{

        double power, time;
        ElapsedTime timer;

        public TimedIntake(double power, double time) {
            this.power = power;
            this.time = time;
        }

        @Override
        public void start() {
            driveIntake(power);
            timer = new ElapsedTime();
        }

        @Override
        public void stop() {
            driveIntake(0);
        }

        @Override
        public boolean update() {
            return timer.milliseconds() > time;
        }
    }
}
