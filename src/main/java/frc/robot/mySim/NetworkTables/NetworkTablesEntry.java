package Canvas.NetworkTables;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collection;
import java.util.Collections;
import java.util.function.IntFunction;

public abstract class NetworkTablesEntry{
    protected final boolean mutateAble;
    protected final String totalPath;

    public NetworkTablesEntry(boolean mutateAble, String totalPath){
        this.mutateAble = mutateAble;
        this.totalPath = totalPath;
    }

    public String getTotalPath(){
        return totalPath;
    }

    public static abstract class NetworkEntrySetup<T> extends NetworkTablesEntry{
        protected T value = null;

        public NetworkEntrySetup(boolean mutateAble, String totalPath) {
            super(mutateAble, totalPath);
        }

        public T getValue(T defaultVal){
            if (value == null){
                return defaultVal;
            }
            return value;
        }

        public void setValue(T value){
            if (!mutateAble){
                return;
            }
            if (this.value != value && (this.value == null || !this.value.equals(value))){
                NetworkTables.addSendChange(totalPath + "/", value);
                this.value = value;
            }
        }

        public abstract void setFromString(String value);

        public abstract String getDisplayValue();
    }
    
    public static class BooleanEntry extends NetworkEntrySetup<Boolean>{
        protected BooleanEntry(boolean mutateAble, String totalPath) {
            super(mutateAble, totalPath);
        }

        @Override
        public void setFromString(String value) {
            if (value.equals("true")){
                setValue(true);
            }else if (value.equals("false")){
                setValue(false);
            }
        }

        @Override
        public String getDisplayValue() {
            return value.toString();
        }
    }

    public static class BooleanArrayEntry extends NetworkEntrySetup<boolean[]>{
        protected BooleanArrayEntry(boolean mutateAble, String totalPath) {
            super(mutateAble, totalPath);
        }

        @Override
        public void setFromString(String value) {
            value = value.replace(" ", "");
            ArrayList<Boolean> array = new ArrayList<>();
            while (value.indexOf(",") != -1){
                String val = value.substring(0, value.indexOf(","));
                if (val.contains("true")){
                    array.add(true);
                }else{
                    array.add(false);
                }
                value = value.substring(value.indexOf(",")+1);
            }
            if (value.contains("true")){
                array.add(true);
            }else{
                array.add(false);
            }
            boolean[] arr = new boolean[array.size()];
            for (int i = 0; i < array.size(); i++){
                arr[i] = array.get(i);
            }
            setValue(arr);
        }

        @Override
        public String getDisplayValue() {
            if (value.length < 1){
                return "";
            }
            String submit = "";
            for (int i = 0; i < value.length-1; i++){
                submit+=value[i]+", ";
            }
            submit+=value[value.length-1];
            return submit;
        }
    }

    public static class IntegerEntry extends NetworkEntrySetup<Integer>{
        protected IntegerEntry(boolean mutateAble, String totalPath) {
            super(mutateAble, totalPath);
        }

        @Override
        public void setFromString(String value) {
            try {
                setValue(Integer.parseInt(value));
            } catch (Exception e) {
                setValue(0);
            }
        }

        @Override
        public String getDisplayValue() {
            return value.toString();
        }
    }

    public static class IntegerArrayEntry extends NetworkEntrySetup<int[]>{
        protected IntegerArrayEntry(boolean mutateAble, String totalPath) {
            super(mutateAble, totalPath);
        }

        @Override
        public void setFromString(String value) {
            value = value.replace(" ", "");
            ArrayList<Integer> array = new ArrayList<>();
            while (value.indexOf(",") != -1){
                String val = value.substring(0, value.indexOf(","));
                try {
                    array.add(Integer.parseInt(val));
                } catch (Exception e) {
                    array.add(0);
                }
                value = value.substring(value.indexOf(",")+1);
            }
            try {
                array.add(Integer.parseInt(value));
            } catch (Exception e) {
                array.add(0);
            }
            int[] arr = new int[array.size()];
            for (int i = 0; i < array.size(); i++){
                arr[i] = array.get(i);
            }
            setValue(arr);
        }

        @Override
        public String getDisplayValue() {
            if (value.length < 1){
                return "";
            }
            String submit = "";
            for (int i = 0; i < value.length-1; i++){
                submit+=value[i]+", ";
            }
            submit+=value[value.length-1];
            return submit;
        }
    }

    public static class DoubleEntry extends NetworkEntrySetup<Double>{
        protected DoubleEntry(boolean mutateAble, String totalPath) {
            super(mutateAble, totalPath);
        }

        @Override
        public void setFromString(String value) {
            try {
                setValue(Double.parseDouble(value));
            } catch (Exception e) {
                setValue(0.0);
            }
            
        }

        @Override
        public String getDisplayValue() {
            return value.toString();
        }
    }

    public static class DoubleArrayEntry extends NetworkEntrySetup<double[]>{
        protected DoubleArrayEntry(boolean mutateAble, String totalPath) {
            super(mutateAble, totalPath);
        }

        @Override
        public void setFromString(String value) {
            value = value.replace(" ", "");
            ArrayList<Double> array = new ArrayList<>();
            while (value.indexOf(",") != -1){
                String val = value.substring(0, value.indexOf(","));
                try {
                    array.add(Double.parseDouble(val));
                } catch (Exception e) {
                    array.add(0.0);
                }
                value = value.substring(value.indexOf(",")+1);
            }
            try {
                array.add(Double.parseDouble(value));
            } catch (Exception e) {
                array.add(0.0);
            }
            double[] arr = new double[array.size()];
            for (int i = 0; i < array.size(); i++){
                arr[i] = array.get(i);
            }
            setValue(arr);
        }

        @Override
        public String getDisplayValue() {
            if (value.length < 1){
                return "";
            }
            String submit = "";
            for (int i = 0; i < value.length-1; i++){
                submit+=value[i]+", ";
            }
            submit+=value[value.length-1];
            return submit;
        }
    }

    public static class StringEntry extends NetworkEntrySetup<String>{
        protected StringEntry(boolean mutateAble, String totalPath) {
            super(mutateAble, totalPath);
        }

        @Override
        public void setFromString(String value) {
            setValue(value);
        }

        @Override
        public String getDisplayValue() {
            return value;
        }
    }

    public static class StringArrayEntry extends NetworkEntrySetup<String[]>{
        protected StringArrayEntry(boolean mutateAble, String totalPath) {
            super(mutateAble, totalPath);
        }

        @Override
        public void setFromString(String value) {
            value.replace(" ", "");
            ArrayList<String> array = new ArrayList<>();
            while (value.indexOf(",") != -1){
                String val = value.substring(0, value.indexOf(","));
                
                array.add(val);

                value = value.substring(value.indexOf(",")+1);
            }
            array.add(value);

            String[] arr = new String[array.size()];
            for (int i = 0; i < array.size(); i++){
                arr[i] = array.get(i);
            }
            setValue(arr);
        }

        @Override
        public String getDisplayValue() {
            if (value.length < 1){
                return "";
            }
            String submit = "";
            for (int i = 0; i < value.length-1; i++){
                submit+=value[i]+", ";
            }
            submit+=value[value.length-1];
            return submit;
        }
    }
}
