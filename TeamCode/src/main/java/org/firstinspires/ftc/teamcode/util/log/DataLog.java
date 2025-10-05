package org.firstinspires.ftc.teamcode.util.log;

public class DataLog {
    // The underlying datalogger object - it cares only about an array of loggable fields
    private final DataLogger datalogger;

    // These are all of the fields that we want in the datalog.
    // Note that order here is NOT important. The order is important in the setFields() call below
    public DataLogger.GenericField identifier = new DataLogger.GenericField("Identifier");
    public DataLogger.GenericField current = new DataLogger.GenericField("Current");
    public DataLogger.GenericField ticks = new DataLogger.GenericField("Ticks");
    public DataLogger.GenericField setVelocity = new DataLogger.GenericField("Set Velocity");
    public DataLogger.GenericField velocity = new DataLogger.GenericField("Velocity");
    //public DataLogger.GenericField runTime = new DataLogger.GenericField("Run Time");
    public DataLogger.GenericField motorType = new DataLogger.GenericField("Motor Type");
    public DataLogger.GenericField batteryVoltage = new DataLogger.GenericField("Battery Voltage");

    public DataLog(String name)
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
                .setFields(identifier, setVelocity, velocity, current, batteryVoltage).build();
    }

    // Tell the datalogger to gather the values of the fields
    // and write a new line in the log.
    public void writeLine()
    {
        datalogger.writeLine();
    }
}