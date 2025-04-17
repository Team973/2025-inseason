import org.gradle.api.DefaultTask
import org.gradle.api.tasks.TaskAction
import org.gradle.api.tasks.options.Option
import UploadLimelightConfig

public abstract class UploadLimelightConfigAll extends DefaultTask {
	private String fileName = "config"

  @Option(option = "file-name", description = "The name for the config file.")
  void setFileName(String fileName) {
      this.fileName = fileName
  }

	@TaskAction
	void uploadLimelightConfigAll() {
		UploadLimelightConfig.doUploadLimelightConfig(fileName, "left")
		UploadLimelightConfig.doUploadLimelightConfig(fileName, "right")
	}
}