package org.firstinspires.ftc.teamcode.log;

public class DatalogAprilTag {
    // The underlying datalogger object - it cares only about an array of loggable fields
    private final Datalogger datalogger;

    // These are all of the fields that we want in the datalog.
    // Note that order here is NOT important. The order is important in the setFields() call below
    public Datalogger.GenericField selectX = new Datalogger.GenericField("X_Pos");
    public Datalogger.GenericField selectY = new Datalogger.GenericField("Y_Pos");
    public Datalogger.GenericField ID = new Datalogger.GenericField("ID");
    public Datalogger.GenericField poseX = new Datalogger.GenericField("pose_X");
    public Datalogger.GenericField poseY = new Datalogger.GenericField("pose_Y");
    public Datalogger.GenericField poseBearing = new Datalogger.GenericField("pose_bearing");
    public Datalogger.GenericField poseYaw = new Datalogger.GenericField("pose_yaw");
    public Datalogger.GenericField rawX = new Datalogger.GenericField("raw_x");
    public Datalogger.GenericField rawY = new Datalogger.GenericField("raw_y");
    public Datalogger.GenericField rawZ = new Datalogger.GenericField("raw_z");
    public Datalogger.GenericField rawPitch = new Datalogger.GenericField("raw_pitch");
    public Datalogger.GenericField rawRoll = new Datalogger.GenericField("raw_roll");
    public Datalogger.GenericField rawYaw = new Datalogger.GenericField("raw_yaw");



    public DatalogAprilTag(String name)
    {
        // Build the underlying datalog object
        datalogger = new Datalogger.Builder()

            // Pass through the filename
            .setFilename(name)

            // Request an automatic timestamp field
            .setAutoTimestamp(Datalogger.AutoTimestamp.DECIMAL_SECONDS)

            // Tell it about the fields we care to log.
            // Note that order *IS* important here! The order in which we list
            // the fields is the order in which they will appear in the log.
            .setFields(selectX,selectY,ID,poseX,poseY,poseYaw,poseBearing,rawX,rawY,rawZ,rawPitch,rawRoll,rawYaw).build();
    }

    // Tell the datalogger to gather the values of the fields
    // and write a new line in the log.
    public void writeLine()
    {
        datalogger.writeLine();
    }
}