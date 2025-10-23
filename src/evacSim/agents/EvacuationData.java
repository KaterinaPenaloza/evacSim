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

/**
 * Recolecta:
 * - Evacuados vs tiempo (CSV consolidado con múltiples experimentos)
 * - Boxplot del tiempo total de evacuación
 * - CSV de detalle y acumulado
 */
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

    // recoger tiempos de cada agente
    public static synchronized void recordAgentTimes(String agentName, double preMovementSec, double movementSec, double totalSec) {
        AGENT_TIMES.add(new AgentTimes(agentName, preMovementSec, movementSec, totalSec));
        TOTAL_TIMES.add(totalSec); // Agregar al listado
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

    // Configuracion
    private final int totalAgents;
    private final String csvDetailsPath;
    private final String csvEvacuatedData;
    private final String chartEvacuatedData;
    private final String boxplotPath;

    private final XYSeries series = new XYSeries("Evacuados");
    private final List<DataPoint> dataPoints = new ArrayList<>();
    private int lastEvacuated = 0;

    // Número de experimento actual (se obtiene automáticamente)
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

    /**
     * Lee los parámetros de velocidad y número de experimento desde RunEnvironment
     * Detecta automáticamente el siguiente experimento disponible leyendo el CSV
     */
    private void readScenarioFromParameters() {
        try {
            Parameters params = RunEnvironment.getInstance().getParameters();
            
            // Leer porcentajes de velocidad
            double pct_0_5 = ((Number) params.getValue("param_04_percentSpeed_0_5")).doubleValue();
            double pct_1_0 = ((Number) params.getValue("param_05_percentSpeed_1_0")).doubleValue();
            double pct_1_5 = 100.0 - pct_0_5 - pct_1_0;
            
            // Crear código del escenario (formato: XXX,YYY,ZZZ sin comas)
            int p1 = (int) Math.round(pct_0_5);
            int p2 = (int) Math.round(pct_1_0);
            int p3 = (int) Math.round(pct_1_5);
            this.scenarioCode = String.format("%03d%03d%03d", p1, p2, p3);
            
            // Detectar automáticamente el siguiente experimento disponible
            this.experimentNumber = detectNextExperiment();
            
            System.out.println("DEBUG: Escenario detectado: " + scenarioCode + 
                             " (velocidades: " + p1 + "%, " + p2 + "%, " + p3 + "%)");
            System.out.println("DEBUG: Número de experimento auto-detectado: " + experimentNumber);
            
        } catch (Exception e) {
            System.err.println("ERROR leyendo parámetros del escenario: " + e.getMessage());
            this.scenarioCode = "000000100"; // Default
            this.experimentNumber = 0;
        }
    }

    /**
     * Detecta automáticamente el siguiente experimento disponible
     * leyendo el CSV consolidado y viendo qué columnas ya tienen datos
     * @return número del siguiente experimento disponible (0-9)
     */
    private int detectNextExperiment() {
        String consolidatedPath = csvEvacuatedData.replace(".csv", 
            "_escenario" + scenarioCode + ".csv");
        File f = new File(consolidatedPath);
        
        // Si el archivo no existe, es el experimento 0
        if (!f.exists()) {
            return 0;
        }
        
        try (BufferedReader r = new BufferedReader(new FileReader(f))) {
            // Leer header
            String headerLine = r.readLine();
            if (headerLine == null) {
                return 0;
            }
            
            // Leer primera línea de datos
            String firstDataLine = r.readLine();
            if (firstDataLine == null) {
                return 0;
            }
            
            String[] parts = firstDataLine.split(";");
            
            // Contar cuántas columnas de datos tienen información
            // Formato: tick;time_seconds;evacuated_count0;evacuated_count1;...
            // Entonces parts[2] es evacuated_count0, parts[3] es evacuated_count1, etc.
            int filledExperiments = 0;
            for (int i = 2; i < parts.length && i < 12; i++) { // 12 = 2 + 10 experimentos
                String value = parts[i].trim();
                if (!value.isEmpty()) {
                    filledExperiments++;
                } else {
                    // Encontramos la primera columna vacía
                    break;
                }
            }
            
            // El siguiente experimento es el número de experimentos ya completados
            int nextExp = filledExperiments;
            
            // Validar que esté en rango
            if (nextExp < 0) nextExp = 0;
            if (nextExp > 9) {
                System.out.println("ADVERTENCIA: Ya se completaron los 10 experimentos para el escenario " + scenarioCode);
                System.out.println("Se sobrescribirá el experimento 9.");
                nextExp = 9;
            }
            
            return nextExp;
            
        } catch (IOException e) {
            System.err.println("ERROR detectando siguiente experimento: " + e.getMessage());
            return 0;
        }
    }

    /**
     * Permite establecer manualmente el número de experimento
     * (útil si se ejecuta desde código externo)
     */
    public void setExperimentNumber(int expNum) {
        if (expNum >= 0 && expNum <= 9) {
            this.experimentNumber = expNum;
            System.out.println("DEBUG: Número de experimento establecido manualmente: " + expNum);
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
        if (reachedTarget >= totalAgents) {
            saveCsvDetails();
            appendToConsolidatedCsv(); // NUEVO: CSV consolidado
            generateEvacuatedChart();
            generateBoxPlot();
            
            // Pausar la simulación
            RunEnvironment.getInstance().pauseRun();
        }
    }

    /**
     * NUEVO: Agrega los datos de este experimento al CSV consolidado del escenario
     * Formato: tick;time_seconds;evacuated_count0;evacuated_count1;...;evacuated_count9
     */
    private void appendToConsolidatedCsv() {
        try {
            String consolidatedPath = csvEvacuatedData.replace(".csv", 
                "_escenario" + scenarioCode + ".csv");
            File f = new File(consolidatedPath);
            
            boolean fileExists = f.exists();
            
            if (f.getParentFile() != null) f.getParentFile().mkdirs();
            
            // Si es el primer experimento (0) o el archivo no existe, crear archivo con headers
            if ((experimentNumber == 0 || !fileExists)) {
                try (BufferedWriter w = new BufferedWriter(new FileWriter(f))) {
                    // Escribir header
                    w.write("tick;time_seconds");
                    for (int i = 0; i < 10; i++) {
                        w.write(";evacuated_count" + i);
                    }
                    w.write("\n");
                    
                    // Escribir datos del primer experimento
                    for (DataPoint dp : dataPoints) {
                        w.write(dp.tick + ";" + String.format(java.util.Locale.US, "%.2f", dp.timeSeconds));
                        // Escribir datos en la columna correspondiente
                        for (int i = 0; i < 10; i++) {
                            if (i == experimentNumber) {
                                w.write(";" + dp.evacuatedCount);
                            } else {
                                w.write(";");
                            }
                        }
                        w.write("\n");
                    }
                }
                System.out.println("CSV consolidado creado/reiniciado: " + f.getPath() + " (exp " + experimentNumber + ")");
            } else {
                // Leer archivo existente
                List<String> lines = new ArrayList<>();
                try (BufferedReader r = new BufferedReader(new FileReader(f))) {
                    String line;
                    while ((line = r.readLine()) != null) {
                        lines.add(line);
                    }
                }
                
                // Crear mapa de tick -> evacuados para este experimento
                Map<Integer, Integer> tickToEvacuated = new HashMap<>();
                int maxTickThisExperiment = 0;
                for (DataPoint dp : dataPoints) {
                    tickToEvacuated.put(dp.tick, dp.evacuatedCount);
                    if (dp.tick > maxTickThisExperiment) {
                        maxTickThisExperiment = dp.tick;
                    }
                }
                
                // Determinar el tick máximo que existe en el archivo
                int maxTickInFile = 0;
                if (lines.size() > 1) {
                    String lastLine = lines.get(lines.size() - 1);
                    String[] parts = lastLine.split(";");
                    if (parts.length >= 1) {
                        maxTickInFile = Integer.parseInt(parts[0].trim());
                    }
                }
                
                // El tick máximo total es el mayor entre el archivo y este experimento
                int maxTickTotal = Math.max(maxTickInFile, maxTickThisExperiment);
                
                // Reescribir archivo con nueva columna y filas adicionales si es necesario
                try (BufferedWriter w = new BufferedWriter(new FileWriter(f))) {
                    // Procesar líneas existentes
                    for (int i = 0; i < lines.size(); i++) {
                        String line = lines.get(i);
                        
                        if (i == 0) {
                            // Header: ya está completo
                            w.write(line + "\n");
                        } else {
                            // Extraer tick de la línea
                            String[] parts = line.split(";", -1); // -1 para preservar campos vacíos
                            if (parts.length < 2) {
                                w.write(line + "\n");
                                continue;
                            }
                            
                            int tick = Integer.parseInt(parts[0].trim());
                            
                            // Reconstruir la línea
                            StringBuilder newLine = new StringBuilder();
                            newLine.append(parts[0]).append(";").append(parts[1]); // tick;time_seconds
                            
                            // Agregar las 10 columnas de datos
                            for (int col = 0; col < 10; col++) {
                                newLine.append(";");
                                if (col == experimentNumber) {
                                    // Esta es la columna de este experimento
                                    Integer value = tickToEvacuated.get(tick);
                                    if (value != null) {
                                        newLine.append(value);
                                    }
                                } else if (parts.length > (2 + col)) {
                                    // Mantener valor existente de otro experimento
                                    newLine.append(parts[2 + col]);
                                }
                            }
                            
                            w.write(newLine.toString() + "\n");
                        }
                    }
                    
                    // Si este experimento tiene más ticks que el archivo, agregar filas nuevas
                    if (maxTickThisExperiment > maxTickInFile) {
                        for (int tick = maxTickInFile + 1; tick <= maxTickThisExperiment; tick++) {
                            double timeSec = tick * SECONDS_PER_TICK;
                            w.write(tick + ";" + String.format(java.util.Locale.US, "%.2f", timeSec));
                            
                            // Agregar las 10 columnas
                            for (int col = 0; col < 10; col++) {
                                w.write(";");
                                if (col == experimentNumber) {
                                    Integer value = tickToEvacuated.get(tick);
                                    if (value != null) {
                                        w.write(String.valueOf(value));
                                    }
                                }
                                // Las demás columnas quedan vacías
                            }
                            w.write("\n");
                        }
                    }
                }
                System.out.println("CSV consolidado actualizado (exp " + experimentNumber + ", " + 
                                 dataPoints.size() + " filas): " + f.getPath());
            }
        } catch (IOException e) {
            System.err.println("ERROR guardando CSV consolidado: " + e.getMessage());
            e.printStackTrace();
        } catch (Exception e) {
            System.err.println("ERROR inesperado en CSV consolidado: " + e.getMessage());
            e.printStackTrace();
        }
    }

    // csv evacuación por agente (sin cambios)
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

    // Boxplot (sin cambios)
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
            range.setRange(min * 0.98, max * 1.02);   // o setLower/UpperMargin si prefieres
            // opcional: evita forzar el cero
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