package org.firstinspires.ftc.teamcode.util.log;

import java.util.ArrayList;
import java.util.List;

public class MotorDataLog {
    // The underlying datalogger object - it cares only about an array of loggable fields
    private final DataLogger datalogger;

    // These are all of the fields that we want in the datalog.
    // Note that order here is NOT important. The order is important in the setFields() call below
    public DataLogger.GenericField totalMotorCurrent = new DataLogger.GenericField("Total Current");
    public DataLogger.GenericField shooterCurrent = new DataLogger.GenericField("Shooter Current");
    public DataLogger.GenericField turretCurrent = new DataLogger.GenericField("Turret Current");
    public DataLogger.GenericField intakeCurrent = new DataLogger.GenericField("Intake Current");
    public DataLogger.GenericField lfCurrent = new DataLogger.GenericField("LF Current");
    public DataLogger.GenericField lrCurrent = new DataLogger.GenericField("LR Current");
    public DataLogger.GenericField rfCurrent = new DataLogger.GenericField("RF Current");
    public DataLogger.GenericField rrCurrent = new DataLogger.GenericField("RR Current");
    public DataLogger.GenericField batteryVoltage = new DataLogger.GenericField("Battery Voltage");

    public List<DataLogger.GenericField> fields = new ArrayList<>();


    public MotorDataLog(String name)
    {
        // Build the underlying datalog object
        datalogger = new DataLogger.Builder()

                // Pass through the filename
                .setFilename(name)

                // Request an automatic timestamp field
                .setAutoTimestamp(DataLogger.AutoTimestamp.DECIMAL_SECONDS)

                // Tell it about the fields we care to log.
                // Note that order *IS* important here! The order in which we list
                // the fields is the order in which they will appear in the log.
                .setFields(batteryVoltage, totalMotorCurrent, shooterCurrent, turretCurrent, intakeCurrent, lfCurrent, lrCurrent, rfCurrent, rrCurrent).build();
        fields.add(batteryVoltage);
        fields.add(totalMotorCurrent);
        fields.add(shooterCurrent);
        fields.add(turretCurrent);
        fields.add(intakeCurrent);
        fields.add(lfCurrent);
        fields.add(lrCurrent);
        fields.add(rfCurrent);
        fields.add(rrCurrent);
    }

    // Tell the datalogger to gather the values of the fields
    // and write a new line in the log.
    public void writeLine()
    {
        datalogger.writeLine();
    }
}