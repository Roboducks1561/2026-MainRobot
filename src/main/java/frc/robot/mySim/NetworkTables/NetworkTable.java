package Canvas.NetworkTables;

import java.util.List;
import java.util.concurrent.ConcurrentHashMap;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Supplier;

import Canvas.NetworkTables.NetworkTablesEntry.BooleanArrayEntry;
import Canvas.NetworkTables.NetworkTablesEntry.BooleanEntry;
import Canvas.NetworkTables.NetworkTablesEntry.DoubleArrayEntry;
import Canvas.NetworkTables.NetworkTablesEntry.DoubleEntry;
import Canvas.NetworkTables.NetworkTablesEntry.IntegerArrayEntry;
import Canvas.NetworkTables.NetworkTablesEntry.IntegerEntry;
import Canvas.NetworkTables.NetworkTablesEntry.StringArrayEntry;
import Canvas.NetworkTables.NetworkTablesEntry.StringEntry;
import Canvas.Util.ParamSupplier;

public class NetworkTable {
    protected ConcurrentHashMap<String, NetworkTable> subTables = new ConcurrentHashMap<>();
    protected ConcurrentHashMap<String, NetworkTablesEntry> entries = new ConcurrentHashMap<>();

    private final boolean mutateAble;
    private final String totalPath;

    protected NetworkTable(boolean mutateAble, String totalPath){
        this.mutateAble = mutateAble;
        this.totalPath = totalPath;

        if (mutateAble){
            NetworkTables.addSendChange(totalPath, null);
        }
    }

    public List<NetworkTable> subTables(){
        return List.copyOf(subTables.values());
    }

    public List<NetworkTablesEntry> entries(){
        return List.copyOf(entries.values());
    }

    private NetworkTablesEntry preformChecks(String fullID, ParamSupplier<NetworkTablesEntry, Boolean, String> entryCreator){
        fullID = fullID.replaceAll("/", "");
        NetworkTablesEntry tryEntry = entryCreator.accept(mutateAble, fullID);
        if (mutateAble){
            if (entries.putIfAbsent(fullID, tryEntry) == null && totalPath.indexOf("Computer") == 0){
                NetworkTables.addSendChange(totalPath + "/" + fullID + "/", null);
            }
        }

        NetworkTablesEntry entry = entries.get(fullID);

        if (entry == null || entry.getClass() != tryEntry.getClass()){
            return tryEntry;
        }

        return entry;
    }

    public NetworkTable getSubTable(String id) {
        id = id.replaceAll("/", "");

        if (subTables.putIfAbsent(id, new NetworkTable(mutateAble, totalPath + "/" + id)) == null && totalPath.indexOf("Computer") == 0){
            NetworkTables.addSendChange(totalPath + "/" + id, null);
        }
        
        
        return subTables.get(id);
    }

    public BooleanEntry getBooleanEntry(String id) {
        NetworkTablesEntry entry = preformChecks(id + "BE", (a, b)->new BooleanEntry(a, totalPath + "/" + b));

        return (BooleanEntry)entry;
    }

    public BooleanArrayEntry getBooleanArrayEntry(String id) {
        NetworkTablesEntry entry = preformChecks(id + "BA", (a, b)->new BooleanArrayEntry(a, totalPath + "/" + b));

        return (BooleanArrayEntry)entry;
    }

    public IntegerEntry getIntegerEntry(String id) {
        NetworkTablesEntry entry = preformChecks(id + "IE", (a, b)->new IntegerEntry(a, totalPath + "/" + b));

        return (IntegerEntry)entry;
    }

    public IntegerArrayEntry getIntegerArrayEntry(String id) {
        NetworkTablesEntry entry = preformChecks(id + "IA", (a, b)->new IntegerArrayEntry(a, totalPath + "/" + b));

        return (IntegerArrayEntry)entry;
    }

    public DoubleEntry getDoubleEntry(String id) {
        NetworkTablesEntry entry = preformChecks(id + "DE", (a, b)->new DoubleEntry(a, totalPath + "/" + b));

        return (DoubleEntry)entry;
    }

    public DoubleArrayEntry getDoubleArrayEntry(String id) {
        NetworkTablesEntry entry = preformChecks(id + "DA", (a, b)->new DoubleArrayEntry(a, totalPath + "/" + b));

        return (DoubleArrayEntry)entry;
    }

    public StringEntry getStringEntry(String id) {
        NetworkTablesEntry entry = preformChecks(id + "SE", (a, b)->new StringEntry(a, totalPath + "/" + b));

        return (StringEntry)entry;
    }

    public StringArrayEntry getStringArrayEntry(String id) {
        NetworkTablesEntry entry = preformChecks(id + "SA", (a, b)->new StringArrayEntry(a, totalPath + "/" + b));

        return (StringArrayEntry)entry;
    }

    public String getTotalPath(){
        return totalPath;
    }
}
