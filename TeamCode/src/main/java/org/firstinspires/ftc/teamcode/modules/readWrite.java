package org.firstinspires.ftc.teamcode.modules;

import android.content.Context;
import android.util.Log;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.util.Scanner;

public class readWrite {

    public void writeToFile(String data) {
        try {
            File myFile = new File("/storage/emulated/0/tmp/data.txt");
            FileWriter myWriter = new FileWriter(myFile);
            myWriter.write(data);
            myWriter.flush();
            myWriter.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public String readFromFile() {
        StringBuilder text = new StringBuilder();
        File myFile = new File("/storage/emulated/0/tmp/data.txt");
        try {
            BufferedReader br = new BufferedReader(new FileReader(myFile));
            String line;
            while ((line = br.readLine()) != null) {
                text.append(line);
            }
            br.close();
        } catch (IOException e) { }
        String result = text.toString();
        return result;
    }
}
