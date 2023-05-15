
import java.applet.AudioClip
import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;
import java.nio.charset.StandardCharsets

import java.nio.file.Files
import java.nio.file.Paths
import java.util.logging.Level
import java.util.logging.Logger

import javax.sound.sampled.AudioInputStream
import javax.sound.sampled.LineEvent
import javax.sound.sampled.LineListener

import com.neuronrobotics.bowlerstudio.AudioPlayer
import com.neuronrobotics.bowlerstudio.AudioStatus
import com.neuronrobotics.bowlerstudio.ISpeakingProgress 
import com.neuronrobotics.bowlerstudio.BowlerKernel
import com.neuronrobotics.bowlerstudio.BowlerStudio
import com.neuronrobotics.bowlerstudio.scripting.ScriptingEngine
import com.squareup.okhttp.MediaType
import com.squareup.okhttp.OkHttpClient
import com.squareup.okhttp.Request
import com.squareup.okhttp.RequestBody
import com.squareup.okhttp.Response

import javafx.application.Platform
import javafx.scene.control.Alert
import javafx.scene.control.Alert.AlertType
import javafx.scene.control.TextInputDialog
import marytts.LocalMaryInterface
import marytts.MaryInterface
import marytts.datatypes.MaryData
import marytts.exceptions.SynthesisException

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.reflect.TypeToken;
import java.lang.reflect.Type;

public class GPTInterface {
	Alert a;
	public final String AI_MODEL_NAME = "gpt-3.5-turbo";

	private String API_KEY;
	private static final String CHATGPT_API_URL = "https://api.openai.com/v1/chat/completions";
	Type TT_mapStringString = new TypeToken<HashMap<String, Object>>() {}.getType();
	Gson gson = new GsonBuilder().disableHtmlEscaping().setPrettyPrinting().create();
	int maxSize = 240
	
	//
	//	final TrustStrategy acceptingTrustStrategy = new TrustStrategy() {
	//		@Override
	//		public boolean isTrusted(X509Certificate[] arg0, String arg1) throws CertificateException {
	//			return true;
	//		}
	//
	//	};
	//	final SSLContext sslContext = SSLContexts.custom()
	//	.loadTrustMaterial(null, acceptingTrustStrategy)
	//	.build();
	//	final SSLConnectionSocketFactory sslsf =
	//	new SSLConnectionSocketFactory(sslContext, NoopHostnameVerifier.INSTANCE);
	//	final Registry<ConnectionSocketFactory> socketFactoryRegistry =
	//	RegistryBuilder.<ConnectionSocketFactory> create()
	//	.register("https", sslsf)
	//	.register("http", new PlainConnectionSocketFactory())
	//	.build();
	//
	//	final BasicHttpClientConnectionManager connectionManager =
	//	new BasicHttpClientConnectionManager(socketFactoryRegistry);
	public GPTInterface(String APIKey) {
		this.API_KEY = APIKey;
		Platform.runLater( {
			Alert alert = new Alert(AlertType.INFORMATION);
			a = alert;
			alert.setTitle("This Operation takes time");
			alert.setHeaderText("");
			alert.setContentText("Just chill out...");
			alert.showAndWait();
		});
	}

	public String request(String phrase) throws IOException {
		return request(phrase, 0.7f);
	}
	/*
	 * '{
     "model": "gpt-3.5-turbo",
     "messages": [{"role": "user", "content": "Say this is a test!"}],
     "temperature": 0.7
   }'
	 */
	public String request(String phrase, float randomness) throws IOException {
		if(Math.random()>0.5)
			phrase="Pretend you are an Fortuine teller that tells fortunes in dad jokes. Keep your response less than "+(maxSize*0.5)+" charecters. As a Fortuine teller respond to: "+phrase
		else
			phrase="Pretend you are an thoughtful Fortuine teller. Keep your response less than "+(maxSize*0.5)+" charecters. As a Fortuine teller make a thoughtful response to: "+phrase
			
		String requestBody = String.format("{\"model\":\"%s\",\"messages\":\"%s\",\"temperature\":%f}", AI_MODEL_NAME, phrase, randomness);
		HashMap<String,Object> message = new HashMap(); 
		HashMap<String,String> messages = new HashMap();
		messages.put("role", "user")
		messages.put("content", phrase)
		message.put("model", AI_MODEL_NAME)
		message.put("temperature", randomness)
		message.put("messages", Arrays.asList(messages))
		
		requestBody = gson.toJson(message, TT_mapStringString);
		
		
		println requestBody
		OkHttpClient client = new OkHttpClient()

		MediaType mediaType = MediaType.parse("application/json");
		RequestBody body = RequestBody.create(mediaType, requestBody);

		Request request = new Request.Builder()
				.url(CHATGPT_API_URL)
				.method("POST", body)
				.addHeader("Content-Type", "application/json")
				.addHeader("Authorization", "Bearer " + API_KEY)
				.build();
		
		try {
			Response response = client.newCall(request).execute();
			String jsonString = response.body().string();
			HashMap<String, Object> database = gson.fromJson(jsonString, TT_mapStringString);
			ArrayList<Object> choices = database.get("choices")
			HashMap<String, Object> firstChoice = choices.get(0)
			HashMap<String, Object> messageContent = firstChoice.get("message")
			String ret = messageContent.get("content").toString()
			println ret
			if(ret.length()>maxSize)
				ret=ret.substring(0, maxSize)
			return ret
		} catch (Exception e) {
			e.printStackTrace();
		}
	}
}

String keyLocation = ScriptingEngine.getWorkspace().getAbsolutePath()+"/gpt-key.txt"
if(!new File(keyLocation).exists()) {
	BowlerStudio.runLater({
		TextInputDialog dialog = new TextInputDialog("your OpenAI API Key here");
		dialog.setTitle("Enter your OpenAI Key ");
		dialog.setHeaderText("Create key here https://platform.openai.com/account/api-keys");
		dialog.setContentText("Please enter your key:");
		
		// Traditional way to get the response value.
		Optional<String> result = dialog.showAndWait();
		if (result.isPresent()){
			String resultGet = result.get()
			System.out.println("Your key: " + resultGet);
			new Thread({
				try {
					File myObj = new File(keyLocation);
					if (myObj.createNewFile()) {
					  System.out.println("File created: " + myObj.getName());
					} else {
					  System.out.println("File already exists.");
					}
					FileWriter myWriter = new FileWriter(keyLocation);
					myWriter.write(resultGet);
					myWriter.close();
					System.out.println("Successfully wrote key to the file.");
				  } catch (IOException e) {
					System.out.println("An error occurred.");
					e.printStackTrace();
				  }
				  
				  
			}).start()
		}
		
	})
	return;
}
println "Loading API key from "+keyLocation
String content = new String(Files.readAllBytes(Paths.get(keyLocation)));
println "API key: "+content
GPTInterface gpt = new GPTInterface(content)
String response  = gpt.request("What is my fortune?",0.9)
println "\n\nResponse\n"+response
AudioPlayer.setThreshhold(600/65535.0)
AudioPlayer.setLowerThreshhold(100/65535.0)
ISpeakingProgress sp ={double percent,AudioStatus status->

		println "Progress: "+percent+"% Status "+status+" "
		if(gpt.a!=null) {
			Platform.runLater( {
				gpt.a.setContentText((status==AudioStatus.attack)?"0":"-");
			});
		}
	}
BowlerKernel.speak(response, 100, 0, 201, 1.0, 1.0,sp)
Platform.runLater( {gpt.a.close();})
 


