package org.firstinspires.ftc.teamcode.util;

import android.util.Log;
import static org.firstinspires.ftc.teamcode.components.BaseComponent.logPrefix;

import com.bylazar.configurables.annotations.Configurable;

@Configurable
public class LogCatUtil {

    String logTag;

    /**
     * Logging util to make logging to LogCat simpler
     * @param logTag Tag to be used for identifying in log
     * @param usePrefix whether to prefix logTag with logPrefix in log
     */
    public LogCatUtil(String logTag, boolean usePrefix){
        this.logTag = usePrefix ? logPrefix + logTag : logTag;
    }

    /**
     * Logging util to make logging to LogCat simpler
     * @param logTag Tag to be appended to the universal prefix - should be component name if used in a component
     */
    public LogCatUtil(String logTag){
        this(logTag, true);
    }


    private void write(int priority, String message){
        Log.println(priority, logTag, message);
    }

    public void warn(String message){
        write(Log.WARN, message);
    }

    public void error(String message){
        write(Log.ERROR, message);
    }

    /**
     * Logs two subsequent errors. First says that deviceName was not found in hardware map, and defaulting to an empty object.
     * <br>The second message prints the exception message
     * @param deviceName hardware name of the device
     * @param e exception caught
     */
    public void hardwareCatch(String deviceName, Exception e){
        error("Device \"" + deviceName + "\" not found in hardware map. Defaulting to empty object.");
        error("Exception: " + e.getMessage());
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
