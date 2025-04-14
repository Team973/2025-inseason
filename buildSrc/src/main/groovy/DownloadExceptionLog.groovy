import org.gradle.api.DefaultTask
import org.gradle.api.tasks.TaskAction

public abstract class DownloadExceptionLog extends DefaultTask {
	@TaskAction
	public void downloadExceptionLog() {
    def robotHostName = "lvuser@roborio-${project.frc.getTeamNumber()}-frc.local"
		def logPath = "/home/lvuser/exception_log.txt"

		def fileSource = "${robotHostName}:${logPath}"

    def sshOps = "-o StrictHostKeyChecking=no"

		String[] downloadCmdList
		String[] deleteCmdList
		String osName = System.getProperty("os.name").toLowerCase()

    if (osName.contains("win")) {
			downloadCmdList = ["powershell.exe", "-Command", "scp ${sshOps} ${fileSource} ."]
			deleteCmdList = ["powershell.exe", "-Command", "ssh ${sshOps} ${robotHostName} rm ${logPath}"]
    } else {
			//TODO: test for mac
			downloadCmdList = ["scp", sshOps, robotHostName, logPath]
			deleteCmdList = ["ssh", sshOps, robotHostName, "rm", logPath]
    }

		ProcessBuilder pb = new ProcessBuilder(downloadCmdList)
		Process process = pb.start()

		println "scp terminated with exit code ${process.waitFor()}"

		if (process.waitFor() == 0) {
			ProcessBuilder deletePb = new ProcessBuilder(deleteCmdList)
			Process deleteProcess = deletePb.start()

			println "ssh terminated with exit code ${deleteProcess.waitFor()}"

			if (deleteProcess.waitFor() != 0) {
				throw new Exception("Couldn't delete file.")
			}
		} else {
			throw new Exception("Couldn't download file; it probably doesn't exist.")
		}
	}
}