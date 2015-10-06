package utils;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

public class FileUtils {
	
	static public BufferedWriter OpenFileToAppend(String filename){
		// Write to the file
		File outFile = new File(filename);
		FileWriter fw = null;
		BufferedWriter bw = null;
		try {
			if (!outFile.exists()) {
				outFile.createNewFile();
			}
			fw = new FileWriter(outFile.getAbsoluteFile(), true); // to overwrite remove the second parameter TRUE
			bw = new BufferedWriter(fw);
		} catch (IOException e) {
			System.err.println("Failed to open/write the results to the file: " + outFile.getAbsolutePath() );
			e.printStackTrace();
		}
		return bw;
	}
	
	static public boolean AppendToFileAndClose(String filename, String thingToString){
		// Write to the file
		File outFile = new File(filename);
		FileWriter fw = null;
		BufferedWriter bw = null;
		try {
			if (!outFile.exists()) {
				outFile.createNewFile();
			}
			fw = new FileWriter(outFile.getAbsoluteFile(), true); // to overwrite remove the second parameter TRUE
			bw = new BufferedWriter(fw);
			bw.write(thingToString);
			bw.close();
		} catch (IOException e) {
			System.err.println("Failed to open/write the results to the file: " + outFile.getAbsolutePath() );			
			e.printStackTrace();
			return false;
		}
		return true;
	}
	
	static public BufferedWriter OpenFileToWrite(String filename, String firstString){
		// Write to the file
		File outFile = new File(filename);
		FileWriter fw = null;
		BufferedWriter bw = null;
		try {
			if (!outFile.exists()) {
				outFile.createNewFile();
			}
			fw = new FileWriter(outFile.getAbsoluteFile()); // to append add the second parameter TRUE
			bw = new BufferedWriter(fw);
			bw.write(firstString);
		} catch (IOException e) {
			System.err.println("Failed to open/write the results to the file: " + outFile.getAbsolutePath() );
			e.printStackTrace();
		}
		return bw;
	}
	
}
