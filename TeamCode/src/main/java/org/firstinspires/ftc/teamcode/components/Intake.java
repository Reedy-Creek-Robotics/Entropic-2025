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
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Sets the run mode of the intake motor
     * @param runMode Run mode to set the intake motor to
     */
    public void setMode(DcMotor.RunMode runMode) {
            intakeMotor.setMode(runMode);

    }

    /**
     * Sets the zero power behavior of the intake motor
     * @param zeroPowerBehavior Zero power behavior to set the intake motor to
     */
    public void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
            intakeMotor.setZeroPowerBehavior(zeroPowerBehavior);
    }

    /**
     * Sets PIDF Coefficients of the intake motor
     * <p>Feedforward value is compensated based on battery voltage</p>
     * @param runMode Run Mode to set the intake motor to
     * @param coefficients PIDF Coefficients to set on the intake motor
     */
    public void setPIDFCoefficients(DcMotor.RunMode runMode, PIDFCoefficients coefficients) {
        PIDFCoefficients compensatedCoefficients = new PIDFCoefficients(
                coefficients.p, coefficients.i, coefficients.d,
                coefficients.f * 12 / batteryVoltageSensor.getVoltage()
        );

        intakeMotor.setPIDFCoefficients(runMode, compensatedCoefficients);
    }

    /**
     * Sets PIDF Coefficients of the intake motor
     * <p>Feedforward value is compensated based on battery voltage</p>
     * @param coefficients PIDF Coefficients to set on the intake motor
     */
    public void setPIDFCoefficients(PIDFCoefficients coefficients) {
        setPIDFCoefficients(intakeMotor.getMode(), coefficients);
    }

    /**
     * Run intake motor at given power
     * @param power Power to run intake motor at
     */
    public void driveIntake(double power){
        intakeMotor.setPower(power);
    }

    /**
     * Run intake motor at given velocity
     * @param velocity Velocity to run intake motor at
     */
    public void driveIntake(int velocity){
        intakeMotor.setVelocity(velocity);
    }

    //ToDo Idea: Run intake until all Transfer color sensor sensors detect an artifact

    /**
     * Run intake for given time
     * @param power Power to run intake at
     * @param time Time to run intake for
     */
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
