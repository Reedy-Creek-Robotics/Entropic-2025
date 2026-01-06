package org.firstinspires.ftc.teamcode.components;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.util.LogCatUtil;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public abstract class BaseComponent implements Component {

    private OpMode opMode;

    protected RobotContext context;
    protected RobotDescriptor descriptor;

    protected HardwareMap hardwareMap;

    protected Telemetry ftcTelemetry;
    protected TelemetryManager telemetry;

    protected ElapsedTime commandTime;

    private Command currentCommand;
    private List<Command> nextCommands;

    private List<Component> subComponents = new ArrayList<>();

    public static String logPrefix = "Comp-";

    private LogCatUtil log;

    public BaseComponent(RobotContext context) {
        this.context = context;
        this.opMode = context.opMode;
        this.hardwareMap = opMode.hardwareMap;
        this.descriptor = context.descriptor;
        this.commandTime = new ElapsedTime();
        this.currentCommand = null;
        this.nextCommands = new ArrayList<>();
        this.ftcTelemetry = opMode.telemetry;
        this.telemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    /**
     * Executes the given command.  If there's another command in progress, this one will be added to the queue.
     */
    protected void executeCommand(Command command) {

        /*// Check if the command can be combined with the last command in the queue.
        Command lastCommandInQueue = getLastCommandInQueue();
        if (lastCommandInQueue instanceof CombinableCommand) {
            Command combined = ((CombinableCommand) lastCommandInQueue).combineWith(command);
            if (combined != null) {
                // The command was successfully combined with the last one in the queue, so replace the last one in
                // the queue with this new combination.
                replaceLastCommandInQueue(combined);
                return;
            }
        }*/

        this.nextCommands.add(command);
    }

    /**
     * Stops the current command and removes any additional commands from the queue.
     */
    public void stopAllCommands() {
        if (currentCommand != null) {
            currentCommand.stop();
            currentCommand = null;
        }
        nextCommands.clear();
    }

    /**
     *
     * @param opMode opMode class. Usually accessed with "this"
     * @param alliance alliance as a boolean. <br>false - red (tag 24) <br>true - blue (tag 20)
     * @return new RobotContext with parameters
     */
    public static RobotContext createRobotContext(OpMode opMode, boolean alliance) {
        return new RobotContext(
                opMode,
                new RobotDescriptor(),
                alliance
        );
    }

    /**
     *
     * @param opMode opMode class. Usually accessed with "this"
     * @return new RobotContext with parameters and red alliance
     */
    public static RobotContext createRobotContext(OpMode opMode) {
        return createRobotContext(opMode, false);
    }

    protected Command getCurrentCommand() {
        return currentCommand;
    }

    protected List<Command> getNextCommands() {
        return nextCommands;
    }

    private Command getLastCommandInQueue() {
        return nextCommands.isEmpty() ?
                currentCommand :
                nextCommands.get(nextCommands.size() - 1);
    }

    private void replaceLastCommandInQueue(Command command) {
        if (nextCommands.isEmpty()) {
            // The command queue is empty so the current command is the one being replaced.  Discard the current
            // command and add the new one to the front of the queue.  The next time update is called, the new
            // command will be started.
            currentCommand = null;
            nextCommands.add(0, command);
        } else {
            // The command is in the queue and hasn't been started yet.  Just replace it with the other command.
            nextCommands.set(nextCommands.size() - 1, command);
        }
    }

    protected void addSubComponents(Component... subComponents) {
        this.subComponents.addAll(Arrays.asList(subComponents));
    }

    @Override
    public void init() {
        log = new LogCatUtil("Base");
        for (Component subComponent : subComponents) {
            telemetry.addData("Init SubComponent", subComponent);
            updateTelemetry();
            subComponent.init();
        }
        telemetry.addLine("SubComponents Inited");
        updateTelemetry();
    }

    @Override
    public void update() {

        //telemetry.addData("Current Command",currentCommand);

        // If there is not a current command, but there are commands in the queue, start the next one.
        if (currentCommand == null && !nextCommands.isEmpty()) {
            currentCommand = nextCommands.remove(0);
            currentCommand.start();
            commandTime.reset();
        }

        // If there is a current command we are trying to execute, delegate to it for update status
        if (currentCommand != null) {
            boolean finished = currentCommand.update();

            // If the command is finished, remove it
            if (finished) {
                currentCommand.stop();
                currentCommand = null;
            }
        }

        // Also update any sub-components
        for (Component subComponent : subComponents) {
            subComponent.update();
        }
    }

    @Override
    public boolean isBusy() {
        // We are busy if any child component is busy.
        for (Component subComponent : subComponents) {
            if (subComponent.isBusy()) {
                return true;
            }
        }

        // We are busy if we have a command we are trying to execute and that command is still busy.
        return currentCommand != null || !nextCommands.isEmpty();
    }

    protected boolean isStopRequested() {
        return opMode instanceof LinearOpMode && ((LinearOpMode) opMode).isStopRequested();
    }

    protected void sleep(long millis) {
        if (opMode instanceof LinearOpMode) {
            LinearOpMode opMode = (LinearOpMode) this.opMode;
            opMode.sleep(millis);
            opMode.idle();
        }
    }

    public String toString() {
        return getClass().getSimpleName();
    }

    protected void updateTelemetry(){
        telemetry.update(ftcTelemetry);
    }


}
