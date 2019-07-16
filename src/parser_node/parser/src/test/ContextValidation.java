package test;

import java.io.BufferedReader;
import java.io.DataOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.Socket;

public class ContextValidation {
	
	public static boolean validate(String sceneId, String sentence){
		try {
	        Socket clientSocket = new Socket("localhost", 65436);
	        DataOutputStream outToServer = new DataOutputStream(clientSocket.getOutputStream());
	        BufferedReader inFromServer = new BufferedReader(new InputStreamReader(clientSocket.getInputStream()));
	
	        outToServer.writeBytes(sceneId + ";" + sentence +  '\n');
	        String result = inFromServer.readLine();
	        clientSocket.close();
	        System.out.println(sentence);
	        System.out.println(result);
	        return result.equals("1") ?  true : false;
		} catch(IOException e) {
	    	 e.printStackTrace();
	    	 System.exit(0);
	    	 return false;
	     }
	}
	
	public static void main(String[] args) throws IOException, ClassNotFoundException{
		boolean result = validate("51640", "put(behind(purple(block(_)), cylinder(_)))");
		System.out.println(result);
	}

}
