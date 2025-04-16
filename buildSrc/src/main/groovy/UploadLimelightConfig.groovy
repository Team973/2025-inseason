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

public abstract class UploadLimelightConfig extends DefaultTask {
	private String fileName = "config"
	private String limelight = "right"

	@Option(option = "file-name", description = "The name for the config file.")
	void setFileName(String fileName) {
			this.fileName = fileName
	}

	@Option(option = "limelight", description = "Which limelight to upload the config to; either 'left' or 'right'.")
	void setLimelight(String limelight) {
		this.limelight = limelight
	}

	@TaskAction
  void downloadLimelightConfig() {
		try {
			File configFile = new File("config/limelight-${limelight}/" + fileName + ".json")
			Scanner reader = new Scanner(configFile);
			String data = ""

			while (reader.hasNextLine()) {
				data += reader.nextLine()
			}

     	reader.close();

			URL url = new URL("http://limelight-${limelight}:5807/upload-pipeline")
			HttpURLConnection con = (HttpURLConnection) url.openConnection()
			con.setRequestMethod("POST")

			con.setRequestProperty("Content-Type", "application/json; charset=UTF-8");

			con.setDoInput(true);
			con.setDoOutput(true)

			OutputStream os = con.getOutputStream();

			os.write(data.getBytes());
			os.flush();

			int responseCode = con.getResponseCode();
			System.out.println("Response Code: " + responseCode)
		} catch (Exception e) {
			e.printStackTrace()
		}
  }
}