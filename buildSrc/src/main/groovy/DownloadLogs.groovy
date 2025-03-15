import org.gradle.api.DefaultTask
import org.gradle.api.tasks.TaskAction
import java.io.File

public abstract class DownloadLogs extends DefaultTask {
  @TaskAction
  public void downloadLogs() {
    def logPath = "/home/lvuser/logs/"
    def robotHostName = "lvuser@roborio-${project.frc.getTeamNumber()}-frc.local"
    def sshOps = "-o StrictHostKeyChecking=no"

    String[] getOldLogsCmds

    def fileSource = "${robotHostName}:${logPath}"
    String osName = System.getProperty("os.name").toLowerCase()

    if ( osName.contains("win") ) {
        getOldLogsCmds = ["powershell.exe", "-Command", "ssh ${sshOps} ${robotHostName} ls -1t ${logPath}"]
    } else {
        //TODO: test for mac
        getOldLogsCmds = ["ssh", sshOps, robotHostName, "ls", "-1t", logPath]
    }

    ProcessBuilder getOldLogsPb = new ProcessBuilder(getOldLogsCmds)
    Process getOldLogsP = getOldLogsPb.start()

    String result = new String(getOldLogsP.getInputStream().readAllBytes())

    String[] oldLogs = result.split("\n")

    for (String log : oldLogs) {
      if (log.startsWith("FRC_") && !log.startsWith("FRC_TBD")) {
        System.out.println("Downloading " + log)

        String[] downloadLogCmds
        String[] deleteLogCmds

        if ( osName.contains("win") ) {
          downloadLogCmds = ["powershell.exe", "-Command", "scp ${sshOps} ${fileSource + log} ${log}"]
          deleteLogCmds = ["powershell.exe", "-Command", "ssh ${sshOps} ${robotHostName} rm ${logPath + log}"]
        } else {
          //TODO: test for mac
          downloadLogCmds = ["scp", sshOps, fileSource + log, log]
          deleteLog = ["ssh", sshOps, robotHostName, "rm", logPath + log]
        }

        ProcessBuilder downloadLog = new ProcessBuilder(downloadLogCmds)
        Process downloadLogP = downloadLog.start()

        downloadLogP.waitForOrKill(30 * 1000)

        File file = new File(log)

        if (file.exists()) {
          System.out.println("Deleting " + log)

          ProcessBuilder deleteLog = new ProcessBuilder(deleteLogCmds)
          Process deleteLogP = deleteLog.start()
        }
      }
    }
  }
}