package evacSim.agents;

import java.awt.BasicStroke;
import java.awt.Color;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import org.jfree.chart.ChartFrame;
import org.jfree.chart.ChartUtilities;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.axis.NumberAxis;
import org.jfree.chart.plot.CategoryPlot;
import org.jfree.chart.plot.PlotOrientation;
import org.jfree.chart.plot.XYPlot;
import org.jfree.chart.renderer.xy.XYLineAndShapeRenderer;
import org.jfree.data.statistics.DefaultBoxAndWhiskerCategoryDataset;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;
import org.jfree.chart.ChartFactory;
import org.jfree.chart.renderer.category.BoxAndWhiskerRenderer;
import org.jfree.chart.plot.ValueMarker;
import repast.simphony.engine.schedule.ScheduledMethod;
import repast.simphony.engine.schedule.ScheduleParameters;
import repast.simphony.engine.environment.RunEnvironment;
import repast.simphony.parameter.Parameters;


public class EvacuationData {

    private static final double SECONDS_PER_TICK = 0.5;

    // Datos por agente
    public static class AgentTimes { // (segundos)
        public final String agent;
        public final double preS, moveS, totalS;
        public AgentTimes(String agent, double preS, double moveS, double totalS) {
            this.agent = agent; this.preS = preS; this.moveS = moveS; this.totalS = totalS;
        }
    }

    private static final List<AgentTimes> AGENT_TIMES = new ArrayList<>();
    private static final List<Double> TOTAL_TIMES = new ArrayList<>();

    // Guardar tiempos de cada agente
    public static synchronized void recordAgentTimes(String agentName, double preMovementSec, double movementSec, double totalSec) {
        AGENT_TIMES.add(new AgentTimes(agentName, preMovementSec, movementSec, totalSec));
        TOTAL_TIMES.add(totalSec);
    }

    public static synchronized void resetAgentTimes() {
        AGENT_TIMES.clear();
        TOTAL_TIMES.clear();
    }

    public List<Double> getTotalTimes() {
        synchronized (EvacuationData.class) {
            return new ArrayList<>(TOTAL_TIMES);
        }
    }

    // Configuración
    private final int totalAgents;
    private final String csvDetailsPath;
    private final String csvEvacuatedData;
    private final String chartEvacuatedData;
    private final String boxplotPath;

    private final XYSeries series = new XYSeries("Evacuados");
    private final List<DataPoint> dataPoints = new ArrayList<>();
    private int lastEvacuated = 0;

    // Número de experimento actual
    private int experimentNumber = -1;
    private String scenarioCode = null;

    public EvacuationData(int totalAgents) {
        this(totalAgents,
             "./output/evacuation_details.csv",
             "./output/evacuation_data.csv",
             "./output/evacuation_data.png",
             "./output/boxplot.png");
    }

    public EvacuationData(int totalAgents, String csvDetailsPath, String csvEvacuatedPath, String chartEvacuatedPath, String boxplotPath) {
        this.totalAgents = totalAgents;
        this.csvDetailsPath = csvDetailsPath;
        this.csvEvacuatedData = csvEvacuatedPath;
        this.chartEvacuatedData = chartEvacuatedPath;
        this.boxplotPath = boxplotPath;
        resetAgentTimes();
        
        // Obtener información del escenario desde los parámetros
        readScenarioFromParameters();
    }

    //Lee los parámetros de velocidad y número de experimento desde RunEnvironment
    private void readScenarioFromParameters() {
        try {
            Parameters params = RunEnvironment.getInstance().getParameters();
            
            // Leer porcentajes de velocidad
            double pct_0_5 = ((Number) params.getValue("param_04_percentSpeed_0_5")).doubleValue();
            double pct_1_0 = ((Number) params.getValue("param_05_percentSpeed_1_0")).doubleValue();
            double pct_1_5 = 100.0 - pct_0_5 - pct_1_0;
            
            // id del escenario (formato: XXX,YYY,ZZZ)
            int p1 = (int) Math.round(pct_0_5);
            int p2 = (int) Math.round(pct_1_0);
            int p3 = (int) Math.round(pct_1_5);
            this.scenarioCode = String.format("%03d%03d%03d", p1, p2, p3);
            
            this.experimentNumber = detectNextExperiment();            
        } catch (Exception e) {
            System.err.println("ERROR leyendo parámetros del escenario: " + e.getMessage());
            this.scenarioCode = "000000000"; // Default
            this.experimentNumber = 0;
        }
    }

    // Detecta el siguiente número de experimento
    private int detectNextExperiment() {
        String consolidatedPath = csvEvacuatedData.replace(".csv", 
            "_escenario" + scenarioCode + ".csv");
        File f = new File(consolidatedPath);
        
        // Si el archivo no existe, es el experimento 0
        if (!f.exists()) {
            return 0;
        }
        
        try (BufferedReader r = new BufferedReader(new FileReader(f))) {
            String headerLine = r.readLine();
            if (headerLine == null) {
                return 0;
            }
            
            String[] headerParts = headerLine.split(";");
            int totalExperimentColumns = headerParts.length - 2;
            String firstDataLine = r.readLine();
            if (firstDataLine == null) {
                return 0;
            }
            String[] parts = firstDataLine.split(";", -1);
            
            // Experimentos completados
            int filledExperiments = 0;
            for (int i = 2; i < parts.length; i++) {
                String value = parts[i].trim();
                if (!value.isEmpty()) {
                    filledExperiments++;
                } else {
                    // se encuentra la primera columna vacía
                    break;
                }
            }
            return filledExperiments;
            
        } catch (IOException e) {
            System.err.println("ERROR detectando siguiente experimento: " + e.getMessage());
            return 0;
        }
    }

    public void setExperimentNumber(int expNum) {
        if (expNum >= 0) {
            this.experimentNumber = expNum;
        }
    }

    
    
    @ScheduledMethod(start = 1, interval = 1, priority = ScheduleParameters.LAST_PRIORITY)
    public void collectData() {
        int tick = (int) RunEnvironment.getInstance().getCurrentSchedule().getTickCount();
        double timeSec = tick * SECONDS_PER_TICK;

        int evacuated = Math.min(ContextCreator.EVACUATED_TOTAL, totalAgents);
        evacuated = Math.max(evacuated, lastEvacuated);
        lastEvacuated = evacuated;

        dataPoints.add(new DataPoint(tick, timeSec, evacuated));
        series.add(timeSec, evacuated);
        
        int reachedTarget = ContextCreator.REACHED_TARGET_TOTAL;
        // Mostar graficos al terminar el experimento automaticamente y generar csv
        if (reachedTarget >= totalAgents) {
            saveCsvDetails();
            appendToConsolidatedCsv();
            //generateEvacuatedChart();
            //generateBoxPlot();
            
            // Pausar la simulación
            RunEnvironment.getInstance().pauseRun();
        }
    }

    // Agrega datos al CSV consolidado.
    private void appendToConsolidatedCsv() {
        try {
            String consolidatedPath = csvEvacuatedData.replace(".csv", "_escenario" + scenarioCode + ".csv");
            File f = new File(consolidatedPath);
            
            boolean fileExists = f.exists();
            
            if (f.getParentFile() != null) f.getParentFile().mkdirs();
            
            // Si el archivo no existe, crearlo
            if (!fileExists) {
                try (BufferedWriter w = new BufferedWriter(new FileWriter(f))) {
                    // Header con la primera columna de experimento
                    w.write("tick;time_seconds;evacuated_count" + experimentNumber + "\n");
                    
                    // Escribir datos
                    for (DataPoint dp : dataPoints) {
                        w.write(dp.tick + ";" + 
                               String.format(java.util.Locale.US, "%.2f", dp.timeSeconds) + ";" +
                               dp.evacuatedCount + "\n");
                    }
                }
                System.out.println("CSV consolidado creado: " + f.getPath() + " (exp " + experimentNumber + ")");
                return;
            }
            
            // Leer archivo existente
            List<String> lines = new ArrayList<>();
            try (BufferedReader r = new BufferedReader(new FileReader(f))) {
                String line;
                while ((line = r.readLine()) != null) {
                    lines.add(line);
                }
            }
            
            if (lines.isEmpty()) {
                System.err.println("ERROR: Archivo CSV vacío");
                return;
            }
            
            // Determinar cuántos experimentos ya existen
            String headerLine = lines.get(0);
            String[] headerParts = headerLine.split(";");
            int existingExperiments = headerParts.length - 2;
            
 
            Map<Integer, Integer> tickToEvacuated = new HashMap<>();
            int maxTickThisExperiment = 0;
            for (DataPoint dp : dataPoints) {
                tickToEvacuated.put(dp.tick, dp.evacuatedCount);
                if (dp.tick > maxTickThisExperiment) {
                    maxTickThisExperiment = dp.tick;
                }
            }
            
            // Determinar el tick máximo en el archivo
            int maxTickInFile = 0;
            if (lines.size() > 1) {
                String lastLine = lines.get(lines.size() - 1);
                String[] parts = lastLine.split(";");
                if (parts.length >= 1) {
                    maxTickInFile = Integer.parseInt(parts[0].trim());
                }
            }
            
            int maxTickTotal = Math.max(maxTickInFile, maxTickThisExperiment);
            
            // Determinar si agregar nueva columna
            boolean needNewColumn = experimentNumber >= existingExperiments;
            
            // Reescribir archivo
            try (BufferedWriter w = new BufferedWriter(new FileWriter(f))) {
                // Procesar header
                if (needNewColumn) {
                    w.write(headerLine + ";evacuated_count" + experimentNumber + "\n");
                } else {
                    // Header sin cambios
                    w.write(headerLine + "\n");
                }
                
                // Procesar líneas de datos
                for (int i = 1; i < lines.size(); i++) {
                    String line = lines.get(i);
                    String[] parts = line.split(";", -1);
                    
                    if (parts.length < 2) {
                        w.write(line + "\n");
                        continue;
                    }
                    
                    int tick = Integer.parseInt(parts[0].trim());
                    
                    StringBuilder newLine = new StringBuilder();
                    newLine.append(parts[0]).append(";").append(parts[1]); // tick;time_seconds
                    
                    // Agregar columnas de experimentos existentes
                    for (int col = 0; col < existingExperiments; col++) {
                        newLine.append(";");
                        if (col == experimentNumber) {
                            // Actualizar columna
                            Integer value = tickToEvacuated.get(tick);
                            if (value != null) {
                                newLine.append(value);
                            }
                        } else if (parts.length > (2 + col)) {
                            // Mantener valor existente
                            newLine.append(parts[2 + col]);
                        }
                    }
                    
                    if (needNewColumn) {
                        newLine.append(";");
                        Integer value = tickToEvacuated.get(tick);
                        if (value != null) {
                            newLine.append(value);
                        }
                    }
                    
                    w.write(newLine.toString() + "\n");
                }
               
                if (maxTickThisExperiment > maxTickInFile) {
                    for (int tick = maxTickInFile + 1; tick <= maxTickThisExperiment; tick++) {
                        double timeSec = tick * SECONDS_PER_TICK;
                        w.write(tick + ";" + String.format(java.util.Locale.US, "%.2f", timeSec));
                        
                        // Columnas vacías para experimentos anteriores
                        for (int col = 0; col < existingExperiments; col++) {
                            w.write(";");
                            if (col == experimentNumber) {
                                Integer value = tickToEvacuated.get(tick);
                                if (value != null) {
                                    w.write(String.valueOf(value));
                                }
                            }
                        }
                        
                        if (needNewColumn) {
                            w.write(";");
                            Integer value = tickToEvacuated.get(tick);
                            if (value != null) {
                                w.write(String.valueOf(value));
                            }
                        }
                        
                        w.write("\n");
                    }
                }
            }
            
            System.out.println("CSV consolidado actualizado: exp " + experimentNumber + 
                             " (total experimentos: " + (needNewColumn ? existingExperiments + 1 : existingExperiments) + 
                             ", filas: " + dataPoints.size() + ") - " + f.getPath());
            
        } catch (IOException e) {
            System.err.println("ERROR guardando CSV consolidado: " + e.getMessage());
            e.printStackTrace();
        } catch (Exception e) {
            System.err.println("ERROR inesperado en CSV consolidado: " + e.getMessage());
            e.printStackTrace();
        }
    }

    // CSV evacuación por agente
    private void saveCsvDetails() {
        try {
            File f = new File(csvDetailsPath);
            if (f.getParentFile() != null) f.getParentFile().mkdirs();
            try (BufferedWriter w = new BufferedWriter(new FileWriter(f))) {
                w.write("agent;pre_movement_s;movement_s;total_s\n");
                synchronized (EvacuationData.class) {
                    for (AgentTimes r : AGENT_TIMES) {
                        w.write(r.agent + ";" +
                                String.format(java.util.Locale.US, "%.2f", r.preS) + ";" +
                                String.format(java.util.Locale.US, "%.2f", r.moveS) + ";" +
                                String.format(java.util.Locale.US, "%.2f", r.totalS) + "\n");
                    }
                }
            }
            System.out.println("CSV de detalle guardado en: " + f.getPath());
        } catch (IOException e) {
            System.err.println("ERROR guardando CSV detalle: " + e.getMessage());
        }
    }

    // Gráfico evacuados
    private void generateEvacuatedChart() {
        try {
            XYSeriesCollection dataset = new XYSeriesCollection(series);
            JFreeChart chart = ChartFactory.createXYLineChart(
                    "Evacuados en el tiempo",
                    "Tiempo (s)",
                    "Evacuados (acumulado)",
                    dataset,
                    PlotOrientation.VERTICAL,
                    false, false, false
            );

            XYPlot plot = chart.getXYPlot();
            plot.setBackgroundPaint(Color.WHITE);
            plot.setDomainGridlinePaint(Color.LIGHT_GRAY);
            plot.setRangeGridlinePaint(Color.LIGHT_GRAY);

            XYLineAndShapeRenderer r = new XYLineAndShapeRenderer();
            r.setSeriesPaint(0, new Color(34, 139, 34));
            r.setSeriesStroke(0, new java.awt.BasicStroke(2.5f));
            r.setSeriesShapesVisible(0, false);
            plot.setRenderer(r);

            plot.getRangeAxis().setRange(0, Math.max(1, totalAgents));

            // Agregar timestamp al nombre del archivo
            String timestamp = new SimpleDateFormat("yyyyMMdd_HHmmss").format(new Date());
            File out = new File(chartEvacuatedData.replace(".png", "_" + timestamp + ".png"));
            if (out.getParentFile() != null) out.getParentFile().mkdirs();
            ChartUtilities.saveChartAsPNG(out, chart, 1100, 650);
            System.out.println("Gráfico evacuados guardado en: " + out.getPath());

            ChartFrame frame = new ChartFrame("Evacuados", chart);
            frame.pack();
            frame.setVisible(true);

        } catch (Exception e) {
            System.err.println("ERROR generando gráfico evacuados: " + e.getMessage());
            e.printStackTrace();
        }
    }

    // Boxplot
    private void generateBoxPlot() {
        try {
            DefaultBoxAndWhiskerCategoryDataset ds = new DefaultBoxAndWhiskerCategoryDataset();
            synchronized (EvacuationData.class) {
                ds.add(TOTAL_TIMES, "Evacuación", "Tiempo total (s)");
            }

            JFreeChart box = ChartFactory.createBoxAndWhiskerChart(
                    "Distribución del tiempo total de evacuación",
                    "",
                    "Tiempo total (s)",
                    ds,
                    false
            );

            CategoryPlot plot = box.getCategoryPlot();
            NumberAxis range = (NumberAxis) plot.getRangeAxis();
            
            // calcula min y max reales de tu lista
            double min = TOTAL_TIMES.stream().mapToDouble(Double::doubleValue).min().orElse(0);
            double max = TOTAL_TIMES.stream().mapToDouble(Double::doubleValue).max().orElse(0);

            // fija el rango del eje para incluir outliers
            range.setRange(min * 0.98, max * 1.02);
            range.setAutoRangeIncludesZero(false);
            
            plot.setBackgroundPaint(Color.WHITE);
            plot.setRangeGridlinePaint(Color.LIGHT_GRAY);

            BoxAndWhiskerRenderer renderer = (BoxAndWhiskerRenderer) plot.getRenderer();
      
            renderer.setMeanVisible(false);
            renderer.setMedianVisible(true);
            renderer.setSeriesPaint(0, new Color(46, 134, 171));
            renderer.setWhiskerWidth(0.2);
            renderer.setUseOutlinePaintForWhiskers(true);

            // media
            double mean = TOTAL_TIMES.stream().mapToDouble(Double::doubleValue).average().orElse(0.0);
            ValueMarker meanMarker = new ValueMarker(mean);
            meanMarker.setPaint(Color.BLACK);
            meanMarker.setStroke(new BasicStroke(1.0f));
            plot.addRangeMarker(meanMarker);

            // Guardar como png
            String timestamp = new SimpleDateFormat("yyyyMMdd_HHmmss").format(new Date());
            File out = new File(boxplotPath.replace(".png", "_" + timestamp + ".png"));
            if (out.getParentFile() != null) out.getParentFile().mkdirs();
            ChartUtilities.saveChartAsPNG(out, box, 800, 600);
            System.out.println("Boxplot guardado en: " + out.getPath());

            // Mostrar en pantalla
            ChartFrame frame = new ChartFrame("Boxplot tiempo total", box);
            frame.pack();
            frame.setVisible(true);

        } catch (Exception e) {
            System.err.println("ERROR generando boxplot: " + e.getMessage());
            e.printStackTrace();
        }
    }

    private static class DataPoint {
        final int tick;
        final double timeSeconds;
        final int evacuatedCount;
        DataPoint(int tick, double timeSeconds, int evacuatedCount) {
            this.tick = tick; this.timeSeconds = timeSeconds; this.evacuatedCount = evacuatedCount;
        }
    }
}