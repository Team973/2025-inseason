import org.gradle.api.DefaultTask
import org.gradle.api.tasks.TaskAction

public abstract class DownloadUserLog extends DefaultTask {
  @TaskAction
  public void downloadUserLog() {
    def robotHostName = "lvuser@roborio-${project.frc.getTeamNumber()}-frc.local"
    def logPath = "/home/lvuser/FRC_UserProgram.log"

    def fileSource = "${robotHostName}:${logPath}"

    def sshOps = "-o StrictHostKeyChecking=no"

    String[] commandList
    String osName = System.getProperty("os.name").toLowerCase()

    if (osName.contains("win")) {
      commandList = ["powershell.exe", "-Command", "scp ${sshOps} ${fileSource} ."]
    } else {
      //TODO: test for mac
      commandList = ["scp", sshOps, robotHostName, logPath]
    }

    ProcessBuilder pb = new ProcessBuilder(commandList)
    Process process = pb.start()

    println "scp terminated with exit code ${process.waitFor()}"
    if (process.waitFor() != 0) {
      throw new Exception("Couldn't download file; it probably doesn't exist.")
    }
  }
}