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

public abstract class DownloadLimelightLeftConfig extends DefaultTask {
	private String fileName = "config"

	@Option(option = "file-name", description = "The name for the config file.")
	void setFileName(String fileName) {
			this.fileName = fileName
	}

	@TaskAction
  void downloadLimelightLeftConfig() {
		try {
			URL url = new URL("http://10.9.73.13:5807/pipeline-atindex?index=0")
			HttpURLConnection con = (HttpURLConnection) url.openConnection()
			con.setRequestMethod("GET")
			int responseCode = con.getResponseCode()

			System.out.println("Response Code: " + responseCode)

			if (responseCode == 200) {
				BufferedReader reader = new BufferedReader(new InputStreamReader(con.getInputStream()))

				String inputLine
				StringBuffer response = new StringBuffer()

				while ((inputLine = reader.readLine()) != null) {
					response.append(inputLine)
				}

				reader.close()

				File configFile = new File("config/limelight-left/" + fileName + ".txt")
				configFile.createNewFile()

				FileWriter writer = new FileWriter(configFile)

      	writer.write(response.toString())
      	writer.close();
			}
		} catch (Exception e) {
			e.printStackTrace()
		}
  }
}