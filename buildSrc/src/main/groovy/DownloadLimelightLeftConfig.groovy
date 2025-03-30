import org.gradle.api.DefaultTask
import org.gradle.api.tasks.TaskAction
import java.io.BufferedReader
import java.io.InputStream
import java.io.InputStreamReader
import java.io.FileWriter
import java.io.IOException
import java.net.HttpURLConnection
import java.net.URL

public abstract class DownloadLimelightLeftConfig extends DefaultTask {
	@TaskAction
  void downloadLimelightLeftConfig() {
		try {
			URL url = new URL("")
			HttpURLConnection con = (HttpURLConnection) url.openConnection()
			con.setRequestMethod("GET")
			int responseCode = con.getResponseCode()

			System.out.println(responseCode)

			if (responseCode == 200) {
				BufferedReader reader = new BufferedReader(new InputStreamReader(con.getInputStream()))

				String inputLine
				StringBuffer response = new StringBuffer()

				while ((inputLine = reader.readLine()) != null) {
					response.append(inputLine)
				}

				reader.close()
				System.out.println(response.toString())

				FileWriter myWriter = new FileWriter("/config/limelight-left/config.txt")
      	myWriter.write(response.toString())
      	myWriter.close();
			}
		} catch (Exception e) {
			e.printStackTrace()
		}
  }
}