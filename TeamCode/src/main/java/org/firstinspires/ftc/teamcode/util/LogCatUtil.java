package org.firstinspires.ftc.teamcode.util;

import android.util.Log;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class LogCatUtil {

    static String logPrefix = "Comp-";
    String logTag;

    /**
     * Logging util to make logging to LogCat simpler
     * @param logTag Tag to be appended to the universal prefix - should be component name if used in a component
     */
    public LogCatUtil(String logTag){
        this.logTag = logTag;
    }

    private void write(int priority, String message){
        Log.println(priority, logPrefix+logTag, message);
    }

    public void warn(String message){
        write(Log.WARN, message);
    }

    public void error(String message){
        write(Log.ERROR, message);
    }

    public void debug(String message){
        write(Log.DEBUG, message);
    }

    public void info(String message){
        write(Log.INFO, message);
    }

    public void assertLog(String message){
        write(Log.ASSERT, message);
    }

    public void verbose(String message){
        write(Log.VERBOSE, message);
    }
}
