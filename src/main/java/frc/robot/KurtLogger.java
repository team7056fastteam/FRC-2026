package frc.robot;

import java.io.File;
import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.io.UnsupportedEncodingException;
import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.TimeZone;
import edu.wpi.first.wpilibj.Timer;

public class KurtLogger {
    private PrintWriter log;
    private String logName;
    private double intialTime;

    public enum logType{robotPose, event, number, chassisSpeed}

    public KurtLogger(String additonalName){
        logName = additonalName;

        try {
            File file = new File("/u/Logs");
			    if (!file.exists()) {
				    if (file.mkdir()) {
					    System.out.println("Log Directory is created!");
				    } else {
					    System.out.println("Failed to create Log directory!");
				    }
			    }

		    Date date = new Date() ;
		    SimpleDateFormat dateFormat = new SimpleDateFormat("yyyy-MM-dd-HH-mm-ss");
		    dateFormat.setTimeZone(TimeZone.getTimeZone("EST5EDT"));
            log = new PrintWriter(file.getPath() + "/" + dateFormat.format(date) + "-" + additonalName + "Log.kurtLog", "UTF-8");
            System.out.println("Created logger at " + additonalName);
        } catch (FileNotFoundException e) {
            //e.printStackTrace();
        } catch (UnsupportedEncodingException e) {
            //e.printStackTrace();
        }
    }

    public void initLog(){
        intialTime = Timer.getFPGATimestamp();
        logData(logType.event, "Init", logName);
    }

    public void logData(logType type, String data, String additionalData){
        try{
            // System.out.println("Time:" + (Timer.getFPGATimestamp()-intialTime) + "/" + type.toString() + "/" + additionalData + "/" + data);
            log.println("Time:" + (Timer.getFPGATimestamp()-intialTime) + "/" + type.toString() + "/" + additionalData + "/" + data);
            log.flush();
        }
        catch(Exception e){
            //e.printStackTrace();
        }
    }

    public void close(){
        log.close();
    }
}
