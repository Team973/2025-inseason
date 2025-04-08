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

public abstract class UploadLimelightRightConfig extends DefaultTask {
	private String fileName = "config"

	@Option(option = "file-name", description = "The name for the config file.")
	void setFileName(String fileName) {
			this.fileName = fileName
	}

	@TaskAction
  void downloadLimelightLeftConfig() {
		try {
			File configFile = new File("config/limelight-right/" + fileName + ".txt")
			Scanner reader = new Scanner(configFile);
			String data = ""

			while (reader.hasNextLine()) {
				data += reader.nextLine()
			}

     	reader.close();

			URL url = new URL("http://10.9.73.12:5807/upload-pipeline")
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