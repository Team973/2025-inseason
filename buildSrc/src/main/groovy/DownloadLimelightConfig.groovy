import org.gradle.api.DefaultTask
import org.gradle.api.tasks.TaskAction
import org.gradle.api.tasks.options.Option
import java.io.BufferedReader
import java.io.InputStream
import java.io.InputStreamReader
import java.io.FileWriter
import java.io.File
import java.io.IOException
import java.net.HttpURLConnection
import java.net.URL

public abstract class DownloadLimelightConfig extends DefaultTask {
  private String fileName = "config"
  private String limelight = "right"

  @Option(option = "file-name", description = "The name for the config file.")
  void setFileName(String fileName) {
      this.fileName = fileName
  }

  @Option(option = "limelight", description = "Which limelight to download the config from; either 'left' or 'right'.")
  void setLimelight(String limelight) {
    this.limelight = limelight
  }

  @TaskAction
  void downloadLimelightConfig() {
    try {
      URL url = new URL("http://limelight-${limelight}:5807/pipeline-atindex?index=0")
      HttpURLConnection con = (HttpURLConnection) url.openConnection()
      con.setRequestMethod("GET")
      int responseCode = con.getResponseCode()

      System.out.println("Response Code: " + responseCode)

      if (responseCode == 200) {
        BufferedReader reader = new BufferedReader(new InputStreamReader(con.getInputStream()))

        File configFile = new File("config/limelight-${limelight}/" + fileName + ".json")
        configFile.createNewFile()

        FileWriter writer = new FileWriter(configFile)

        String inputLine

        while ((inputLine = reader.readLine()) != null) {
          writer.write(inputLine)
        }

        reader.close()
        writer.close();
      }
    } catch (Exception e) {
      e.printStackTrace()
    }
  }
}