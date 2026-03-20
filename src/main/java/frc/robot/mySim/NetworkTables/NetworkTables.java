package Canvas.NetworkTables;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.IOException;
import java.io.InputStreamReader;
import java.io.OutputStreamWriter;
import java.lang.reflect.Type;
import java.net.Socket;
import java.net.UnknownHostException;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.concurrent.ConcurrentHashMap;

import Canvas.Commands.CommandBase;
import Canvas.Commands.Commands;
import Canvas.NetworkTables.NetworkTablesEntry.BooleanArrayEntry;
import Canvas.NetworkTables.NetworkTablesEntry.BooleanEntry;
import Canvas.NetworkTables.NetworkTablesEntry.DoubleArrayEntry;
import Canvas.NetworkTables.NetworkTablesEntry.DoubleEntry;
import Canvas.NetworkTables.NetworkTablesEntry.IntegerArrayEntry;
import Canvas.NetworkTables.NetworkTablesEntry.IntegerEntry;
import Canvas.NetworkTables.NetworkTablesEntry.StringArrayEntry;
import Canvas.NetworkTables.NetworkTablesEntry.StringEntry;
import Canvas.Util.Profile;
import Canvas.gson.Gson;
import Canvas.gson.GsonBuilder;
import Canvas.gson.JsonElement;
import Canvas.gson.JsonNull;
import Canvas.gson.JsonObject;
import Canvas.gson.JsonParser;

public class NetworkTables {
    private static Socket socket;
    private static CommandBase connectionThread;
    private static boolean connected;

    private static final Gson gson = new GsonBuilder()
        .serializeNulls()
        .create();
    private static JsonObject sendData = new JsonObject();

    private static NetworkTable computerTable = new NetworkTable(true, "Computer");
    private static NetworkTable robotTable = new NetworkTable(false, "Robot");

    
    private NetworkTables(){
        
    }

    public static NetworkTable getComputerTables(){
        return computerTable;
    }

    public static NetworkTable getRobotTables(){
        return robotTable;
    }

    protected static void addSendChange(String totalID, Object value) {
        JsonElement element = gson.toJsonTree(value);
        sendData.add(totalID, element);
    }

    /**
     * As of now 6/4/2025, should only be called once per code run
     * @param port
     */
    public static void connect(int port){
        if (connectionThread != null && connectionThread.isRunning()){
            connectionThread.cancel();
        }
        connectionThread = Commands.timed(()->{
            try {
                socket = null;
                System.out.println("Waiting for connection");
                while(socket == null){
                    try {
                        socket = new Socket("zacks-pi.local", port);
                        // socket = new Socket("192.168.86.45", port);
                    } catch (Exception e) {
                        // TODO: handle exception
                    }
                }
                System.out.println("Connected");
                connected = true;
                BufferedReader in = new BufferedReader(new InputStreamReader(socket.getInputStream()));
                BufferedWriter out = new BufferedWriter(new OutputStreamWriter(socket.getOutputStream()));

                while(true){
                    String jsonString = gson.toJson(sendData);
                    // System.out.println(jsonString);
                    // sendData.entrySet().clear();
                    out.write(jsonString+"\n");
                    try {
                        out.flush();
                    } catch (Exception e) {
                        // TODO: handle exception
                    }
                    
                    
                    String response = "broke";
                    try {
                        response = in.readLine();
                    } catch (Exception e) {
                        System.out.println("HM");
                    }
                    JsonObject obj = null;

                    try {
                        obj = JsonParser.parseString(response).getAsJsonObject();
                    } catch (Exception e) {
                        // TODO: handle exception
                    }
                    badIfLogic(obj);
                }
                
                // socket.close();
            } catch (UnknownHostException e) {
                connected = false;
                e.printStackTrace();
            } catch (IOException e) {
                connected = false;
                e.printStackTrace();
            }
        },20);
        connectionThread.schedule();
    }

    private static void badIfLogic(JsonObject obj){
        if (obj == null){
            return;
        }
        for (Entry<String, JsonElement> e : obj.entrySet()){
            String totalKey = e.getKey();
            boolean endsWithEntry = totalKey.lastIndexOf("/") == totalKey.length() - 1;

            String[] keys = totalKey.split("/");
            
            
            NetworkTable currentTable = robotTable;
            for (int i = 1; i < keys.length; i++){
                if (i+1 >= keys.length && endsWithEntry){
                    JsonElement element = e.getValue();

                    if (element.isJsonArray()){
                        if (element.toString().contains("\"")){
                            currentTable.entries.putIfAbsent(keys[i]+"SA", new StringArrayEntry(false, currentTable.getTotalPath() +"/"+ keys[i]+"SA"));
                            currentTable.getStringArrayEntry(keys[i]).value = gson.fromJson(element, String[].class);
                        }else if(element.toString().contains(".")){
                            currentTable.entries.putIfAbsent(keys[i]+"DA", new DoubleArrayEntry(false, currentTable.getTotalPath() +"/"+ keys[i]+"DA"));
                            currentTable.getDoubleArrayEntry(keys[i]).value = gson.fromJson(element, double[].class);
                        }else if(element.toString().contains("true") || element.toString().contains("false")){
                            currentTable.entries.putIfAbsent(keys[i]+"BA", new BooleanArrayEntry(false, currentTable.getTotalPath() +"/"+ keys[i]+"BA"));
                            currentTable.getBooleanArrayEntry(keys[i]).value = gson.fromJson(element, boolean[].class);
                        }else{
                            currentTable.entries.putIfAbsent(keys[i]+"IA", new IntegerArrayEntry(false, currentTable.getTotalPath() +"/"+ keys[i]+"IA"));
                            currentTable.getIntegerArrayEntry(keys[i]).value = gson.fromJson(element, int[].class);
                        }
                    }else{
                        if (element.toString().contains("\"")){
                            currentTable.entries.putIfAbsent(keys[i]+"SE", new StringEntry(false, currentTable.getTotalPath() +"/"+ keys[i]+"SE"));
                            currentTable.getStringEntry(keys[i]).value = gson.fromJson(element, String.class);
                        }else if(element.toString().contains(".")){
                            currentTable.entries.putIfAbsent(keys[i]+"DE", new DoubleEntry(false, currentTable.getTotalPath() +"/"+ keys[i]+"DE"));
                            currentTable.getDoubleEntry(keys[i]).value = gson.fromJson(element, Double.class);
                        }else if(element.toString().contains("true") || element.toString().contains("false")){
                            currentTable.entries.putIfAbsent(keys[i]+"BE", new BooleanEntry(false, currentTable.getTotalPath() +"/"+ keys[i]+"BE"));
                            currentTable.getBooleanEntry(keys[i]).value = gson.fromJson(element, Boolean.class);
                        }else{
                            currentTable.entries.putIfAbsent(keys[i]+"IE", new IntegerEntry(false, currentTable.getTotalPath() +"/"+ keys[i]+"IE"));
                            currentTable.getIntegerEntry(keys[i]).value = gson.fromJson(element, Integer.class);
                        }
                    }
                    
                    continue;
                }
                currentTable.subTables.putIfAbsent(keys[i], new NetworkTable(false, currentTable.getTotalPath() + "/" + keys[i]));
                currentTable = currentTable.getSubTable(keys[i]);
            }
        }
    }

    public static boolean isConnected(){
        return connected;
    }
    
}
