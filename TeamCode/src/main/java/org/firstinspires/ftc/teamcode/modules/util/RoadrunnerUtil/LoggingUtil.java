package org.firstinspires.ftc.teamcode.modules.util.RoadrunnerUtil;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * Utility functions for managing log files in the RoadRunner folder.
 */
public class LoggingUtil {
    // The directory where the log files are stored
    public static final File ROAD_RUNNER_FOLDER =
            new File(AppUtil.ROOT_FOLDER + "/RoadRunner/");

    // Maximum allowable total size for log files (25MB)
    private static final long LOG_QUOTA = 25 * 1024 * 1024; // 25MB

    /**
     * Recursively builds a list of log files from the specified directory.
     *
     * @param logFiles The list to store the found files.
     * @param dir The directory to search for log files.
     */
    private static void buildLogList(List<File> logFiles, File dir) {
        // Iterate over all files in the directory
        for (File file : dir.listFiles()) {
            // If it's a directory, recursively search within it
            if (file.isDirectory()) {
                buildLogList(logFiles, file);
            } else {
                // Otherwise, add the file to the log list
                logFiles.add(file);
            }
        }
    }

    /**
     * Prunes old log files if the total size exceeds the LOG_QUOTA.
     * This method deletes the oldest files until the total size is within the limit.
     */
    private static void pruneLogsIfNecessary() {
        List<File> logFiles = new ArrayList<>();

        // Build a list of all log files in the ROAD_RUNNER_FOLDER
        buildLogList(logFiles, ROAD_RUNNER_FOLDER);

        // Sort the log files by their last modified time (oldest first)
        Collections.sort(logFiles, (lhs, rhs) ->
                Long.compare(lhs.lastModified(), rhs.lastModified()));

        // Calculate the total size of the log directory
        long dirSize = 0;
        for (File file: logFiles) {
            dirSize += file.length();
        }

        // While the total size exceeds the quota, delete the oldest files
        while (dirSize > LOG_QUOTA) {
            if (logFiles.size() == 0) break; // Break if no files left to delete
            File fileToRemove = logFiles.remove(0); // Remove the oldest file
            dirSize -= fileToRemove.length(); // Reduce the directory size
            // Delete the file
            //noinspection ResultOfMethodCallIgnored
            fileToRemove.delete();
        }
    }

    /**
     * Retrieves a log file with the provided name.
     *
     * @param name The name of the log file to retrieve.
     * @return A File object representing the log file.
     */
    public static File getLogFile(String name) {
        // Ensure the RoadRunner folder exists (create if it doesn't)
        //noinspection ResultOfMethodCallIgnored
        ROAD_RUNNER_FOLDER.mkdirs();

        // Prune old logs if necessary to stay within the size limit
        pruneLogsIfNecessary();

        // Return the log file with the specified name
        return new File(ROAD_RUNNER_FOLDER, name);
    }
}
