package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.game.ColorValue;

public class Transfer extends BaseComponent {

    private CRServo roller;
    private ColorSensor color;
    private String servoHardwareName;
    private String colorHardwareName;

    // How many amps the current must increase from the baseline by for the shooter to be considered engaged with the artifact
    private final double SHOOTER_CURRENT_THRESHOLD = 0.5;
    private final double TIMEOUT_THRESHOLD = 5000; //ms

    private double shooterCurrent;
    private double transferPower = 1;
    private int transferTime = 2000; //ms

    /*
    This component will take care of all:
        Transfer servo actions (done)
        Color sensors (done)
        LED illumination indicating storage (pending LED/Storage information)
     */

    public Transfer(RobotContext context, String servoHardwareName, String colorHardwareName) {
        super(context);
        this.servoHardwareName = servoHardwareName;
        this.colorHardwareName = colorHardwareName;
    }

    @Override
    public void init() {
        roller = hardwareMap.get(CRServo.class, servoHardwareName);
        color = hardwareMap.get(ColorSensor.class, colorHardwareName);
    }

    /**
     * Set the current to be read as the shooter's current
     * @param current Current value
     */
    public void setShooterCurrent(double current) {
        shooterCurrent = current;
    }

    public double getShooterCurrent() {
        return shooterCurrent;
    }

    /**
     * Run the transfer roller at power
     * @param power Power to run transfer at
     */
    public void runRoller(double power) {
        roller.setPower(power);
    }

    /**
     * Set direction of the roller servo
     * @param direction Direction to set
     */
    public void setDirection(DcMotorSimple.Direction direction) {
        roller.setDirection(direction);
    }

    /**
     * Set the power to use when transferring. NOTE: THIS DOES NOT RUN THE TRANSFER!
     * @param power Power to set the roller to when transferring
     */
    public void setTransferPower(double power) {
        transferPower = power;
    }

    /**
     * Set the time to run the transfer for when transferring. NOTE: THIS DOES NOT RUN THE TRANSFER!
     * @param time Time to run the roller for in a timed transfer
     */
    public void setTransferTime(int time) {
        transferTime = time;
    }

    /**
     * Enable/Disable the color sensors LED
     * @param ledStatus  Enable LED
     */
    public void setLED(boolean ledStatus) {
        color.enableLed(ledStatus);
    }

    /**
     * Get the color values from the color sensor
     * @return ColorValue object with red, green, blue, and alpha channels stored.
     */
    public ColorValue getColor() {
        return new ColorValue(
                color.red(),
                color.green(),
                color.blue(),
                color.alpha()
        );
    }

    /**
     * Run the transfer at a set power for a set time to transfer to shooter.
     * <p>Power can be set with setTransferPower</p>
     * <p>Time can be set with setTransferTime</p>
     */
    public void transferToShooter() {
        executeCommand(new TimedTransfer(transferPower, transferTime));
    }

    /**
     * Run the transfer at a set power for a given time
     * <p>Power can be set with setTransferPower</p>
     * @param time Time in milliseconds to run transfer for
     */
    public void timedTransfer(double time) {
        executeCommand(new TimedTransfer(transferPower, time));
    }

    /**
     * Run the transfer until the shooter's current spikes above a threshold, indicating the shooter has made contact with the artifact.
     * <p>Will time out and stop after a timeout period</p>
     */
    public void transferUntilShot() {
        executeCommand(new TransferUntilShot());
    }

    public class TimedTransfer implements Command {

        double power, time;
        ElapsedTime timer;

        public TimedTransfer(double power, double time) {
            this.power = power;
            this.time = time;
        }

        @Override
        public void start() {
            runRoller(power);
            timer = new ElapsedTime();
        }

        @Override
        public void stop() {
            runRoller(0);
        }

        @Override
        public boolean update() {
            return timer.milliseconds() > time;
        }
    }

    public class TransferUntilShot implements Command {

        double baseline;
        ElapsedTime timeoutTimer;

        @Override
        public void start() {
            timeoutTimer = new ElapsedTime();
            baseline = getShooterCurrent();
            runRoller(transferPower);
        }

        @Override
        public void stop() {
            roller.setPower(0);
        }

        @Override
        public boolean update() {
            return getShooterCurrent() > baseline + SHOOTER_CURRENT_THRESHOLD || timeoutTimer.milliseconds() > TIMEOUT_THRESHOLD;
        }
    }
}
