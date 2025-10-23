package evacSim.agents;

import java.io.File;
import java.io.IOException;
import java.net.MalformedURLException;
import java.net.URL;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;

//import evacSim.utils.LoadingDialog;
//import evacSim.agents.LoadingDialog;


import org.geotools.data.shapefile.ShapefileDataStore;
import org.geotools.data.simple.SimpleFeatureIterator;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Envelope;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.LineString;
import org.locationtech.jts.geom.MultiLineString;
import org.locationtech.jts.geom.MultiPolygon;
import org.locationtech.jts.geom.Point;
import org.opengis.feature.simple.SimpleFeature;

import repast.simphony.context.Context;
import repast.simphony.context.space.gis.GeographyFactoryFinder;
import repast.simphony.context.space.grid.GridFactoryFinder;
import repast.simphony.dataLoader.ContextBuilder;
import repast.simphony.engine.environment.RunEnvironment;
import repast.simphony.parameter.Parameters;
import repast.simphony.space.gis.Geography;
import repast.simphony.space.gis.GeographyParameters;
import repast.simphony.space.grid.Grid;
import repast.simphony.space.grid.GridBuilderParameters;
import repast.simphony.space.grid.SimpleGridAdder;
import repast.simphony.space.grid.StickyBorders;

@SuppressWarnings("rawtypes")
public class ContextCreator implements ContextBuilder {
	int numAgents;
	private ZoneAgent initialZone;
	private ZoneAgent safeZone;

	// Constantes WGS84
	private static final double WGS84_A = 6378137.0;
	private static final double WGS84_F = 1.0 / 298.257223563;
	private static final double WGS84_E2 = WGS84_F * (2 - WGS84_F);
	// Punto de referencia
	private static double REF_LAT = 0;
	private static double REF_LON = 0;
		
	// Grilla
	public static final double CELL_SIZE_METERS = 1.0;
	static final int GRID_WIDTH;
	static final int GRID_HEIGHT;

	// Definición de límites geográficos
	private static final double MIN_LONGITUDE;
	private static final double MAX_LONGITUDE;
	private static final double MIN_LATITUDE;
	private static final double MAX_LATITUDE;

	private static final String INITIAL_ZONE_SHAPEFILE_PATH = "./data/map_data_initial_zones.shp";
	private static final String SAFE_ZONE_SHAPEFILE_PATH = "./data/map_data_safe_zones.shp";
	private static final String ROADS_SHAPEFILE_PATH = "./data/map_data_roads.shp";

	private static double AREA_WIDTH_METERS;
	private static double AREA_HEIGHT_METERS;
	private static double[] REF_ECEF;

	private static Map<GridPoint, Integer> mapCellGrid;
	private static Grid<GisAgent> agentGrid;
	private static Geography<Object> geography;
	public static List<GridPoint> safeZonePoints;
	
	// Evacuados
	public static int EVACUATED_TOTAL = 0;
	public static int REACHED_TARGET_TOTAL = 0;

	
	// Parámetros de velocidad
    private static double percentSpeed_0_5 = 33.33;
    private static double percentSpeed_1_0 = 33.33;
    private static double percentSpeed_1_5 = 33.34;
    
    private int preEvacuationTimeMin;
    private int preEvacuationTimeMax;
	

	// ========================================
	// |           Limites del mapa           |
	// ========================================
	static {
		long startTime = System.currentTimeMillis();
		
		// Leer límites del bbox y asignarlos
		BboxLimits bbox = readBboxLimits();

		MIN_LONGITUDE = bbox.minLon;
		MAX_LONGITUDE = bbox.maxLon;
		MIN_LATITUDE = bbox.minLat;
		MAX_LATITUDE = bbox.maxLat;

		// Validar límites
		validateBboxLimits(MIN_LONGITUDE, MAX_LONGITUDE, MIN_LATITUDE, MAX_LATITUDE);

		// Punto de referencia (el centro)
		REF_LAT = (MIN_LATITUDE + MAX_LATITUDE) / 2;
		REF_LON = (MIN_LONGITUDE + MAX_LONGITUDE) / 2;

		// Mostrar limites por consola
		System.out.printf("MIN_LONGITUDE = %.17f%n", MIN_LONGITUDE);
		System.out.printf("MAX_LONGITUDE = %.17f%n", MAX_LONGITUDE);
		System.out.printf("MIN_LATITUDE  = %.17f%n", MIN_LATITUDE);
		System.out.printf("MAX_LATITUDE  = %.17f%n", MAX_LATITUDE);

		// Convertir de grados a metros
		MetricDimensions metrics = convertToMeters(MIN_LONGITUDE, MAX_LONGITUDE, MIN_LATITUDE, MAX_LATITUDE, REF_LAT);
		AREA_WIDTH_METERS = metrics.widthMeters;
		AREA_HEIGHT_METERS = metrics.heightMeters;

		GRID_WIDTH = (int) Math.ceil(AREA_WIDTH_METERS / CELL_SIZE_METERS);
		GRID_HEIGHT = (int) Math.ceil(AREA_HEIGHT_METERS / CELL_SIZE_METERS);

		// N° de celdas
		System.out.printf("DEBUG: Tamaño de la grilla: %d x %d = %d celdas%n", GRID_WIDTH, GRID_HEIGHT, (long) GRID_WIDTH * GRID_HEIGHT);

		mapCellGrid = new HashMap<>();
		REF_ECEF = geoToECEF(REF_LAT, REF_LON);
		
		long endTime = System.currentTimeMillis();
	    long duration = endTime - startTime;
	    System.out.println("[TIMESTAMP] BBOX - Duración: " + duration + "ms");
	}

	/*************** Leer los limites del archivo ***************/
	private static BboxLimits readBboxLimits() {
		String bboxPath = "./data/bbox_values.txt";
		double minLon = 0, maxLon = 0, minLat = 0, maxLat = 0;

		try (java.io.BufferedReader br = new java.io.BufferedReader(new java.io.FileReader(bboxPath))) {
			String s1 = br.readLine();
			String s2 = br.readLine();
			String s3 = br.readLine();
			String s4 = br.readLine();

			if (s1 == null || s2 == null || s3 == null || s4 == null) {
				throw new IllegalArgumentException("bbox_values.txt incompleto");
			}

			minLon = Double.parseDouble(s1.trim());
			maxLon = Double.parseDouble(s2.trim());
			minLat = Double.parseDouble(s3.trim());
			maxLat = Double.parseDouble(s4.trim());
			
		} catch (Exception e) {
			System.err.println("[ContextCreator] No se pudo leer " + bboxPath + ". Causa: " + e);
		}

		return new BboxLimits(minLon, maxLon, minLat, maxLat);
	}

	/*************** Validar los limites del mapa ***************/
	private static void validateBboxLimits(double minLon, double maxLon, double minLat, double maxLat) {
		if (!(minLon < maxLon) || !(minLat < maxLat)) {
			throw new IllegalStateException("Bounding box inválido. Revisar ./data/bbox_values.txt");
		}
	}

	/*************** Convertir a metros ***************/
	private static MetricDimensions convertToMeters(double minLon, double maxLon, double minLat, double maxLat,
			double refLat) {
		double refLatRad = Math.toRadians(refLat);
		double widthMeters = Math.toRadians(maxLon - minLon) * Math.cos(refLatRad) * WGS84_A;
		double heightMeters = Math.toRadians(maxLat - minLat) * WGS84_A;

		return new MetricDimensions(widthMeters, heightMeters);
	}
	
	// ===================================
	// |             BUILD               |
	// ===================================
	@Override
	public Context build(Context context) {
		
	    long startTime = System.currentTimeMillis();
	    EVACUATED_TOTAL = 0;
	    REACHED_TARGET_TOTAL = 0;

	    // 1) Cargar parámetros
	    Parameters params = RunEnvironment.getInstance().getParameters();
	    
	    //Agentes
	    int numAgentsPreset = (Integer) params.getValue("param_01_numAgentsPreset");
	    int numAgentsCustom = (Integer) params.getValue("param_02_numAgentsCustom");
	    	// Si se escribió uno personalizado, usar ese valor; si no, usar el seleccionado de la lista
	    numAgents = (numAgentsCustom > 0) ? numAgentsCustom : numAgentsPreset;
	    System.out.println("DEBUG: Número de agentes seleccionado: " + numAgents);

	    	//Velocidad
	    percentSpeed_0_5 = (Double) params.getValue("param_04_percentSpeed_0_5");
	    percentSpeed_1_0 = (Double) params.getValue("param_05_percentSpeed_1_0");
	    
	    	// Validar porcentajes
	    double[] validatedPercentages = validateSpeedPercent(percentSpeed_0_5, percentSpeed_1_0);
	    percentSpeed_0_5 = validatedPercentages[0];
	    percentSpeed_1_0 = validatedPercentages[1];
	    percentSpeed_1_5 = validatedPercentages[2];
	    
	 // *** NUEVO: Leer y parsear parámetro de preevacuación ***
	    String preEvacTimeStr = (String) params.getValue("param_preEvacuationTime");
	    parsePreEvacuationTime(preEvacTimeStr);
	    

		// 2) Crear geography y geometry
		GeometryFactory fac = new GeometryFactory();
		geography = createGeography(context);

		// 3) Crear grilla de agentes
		agentGrid = createAgentGrid(context);

		// 4) Inicialización de zonas y carreteras
		initializeZonesAndRoads(context, fac);

		// 5) Inicializar agentes
		initializeAgents(context, fac);

		// 6) Recolectar datos
		addDataCollector(context);

		new LegendHandler().addLegendToDisplay(context);
		
		long endTime = System.currentTimeMillis();
	    long duration = endTime - startTime;
	    System.out.println("[TIMESTAMP] BUILD - Duración: " + duration + "ms");
	    
		return context;
	}
	
    private double[] validateSpeedPercent(double percentSpeed_0_5, double percentSpeed_1_0) {
        double percentSpeed_1_5 = 100.0 - percentSpeed_0_5 - percentSpeed_1_0;
        
        if (percentSpeed_0_5 < 0 || percentSpeed_1_0 < 0 || percentSpeed_1_5 < 0) {
            System.err.println("ERROR: Los porcentajes de velocidad deben sumar 100%");
            return new double[] {33.33, 33.33, 33.34};
        }
        
        return new double[] {percentSpeed_0_5, percentSpeed_1_0, percentSpeed_1_5};
    }
    
    private void parsePreEvacuationTime(String timeStr) {
        try {
            String[] parts = timeStr.split(",");
            if (parts.length != 2) {
                throw new IllegalArgumentException("Formato inválido");
            }
            
            preEvacuationTimeMin = Integer.parseInt(parts[0].trim());
            preEvacuationTimeMax = Integer.parseInt(parts[1].trim());
            
            // Validar que min <= max
            if (preEvacuationTimeMin > preEvacuationTimeMax) {
                int temp = preEvacuationTimeMin;
                preEvacuationTimeMin = preEvacuationTimeMax;
                preEvacuationTimeMax = temp;
                System.out.println("ADVERTENCIA: Valores min/max invertidos. Se corrigieron automáticamente.");
            }
            
            System.out.println("DEBUG: Tiempo preevacuación: " + preEvacuationTimeMin + "-" + preEvacuationTimeMax + " segundos");
            
        } catch (Exception e) {
            // Valores por defecto en caso de error
            preEvacuationTimeMin = 30;
            preEvacuationTimeMax = 60;
            System.err.println("ERROR: Formato inválido en param_preEvacuationTime. Use formato 'min,max' (ej: 30,60). Usando valores por defecto: 30,60");
        }
    }

	/*************** 2) Crear geography **************/
	private Geography<Object> createGeography(Context context) {
		long startTime = System.currentTimeMillis();
		
		GeographyParameters geoParams = new GeographyParameters();
		Geography<Object> geo = GeographyFactoryFinder.createGeographyFactory(null).createGeography("Geography",
				context, geoParams);
		System.out.println("DEBUG: Geography creada.");
		
		long endTime = System.currentTimeMillis();
	    long duration = endTime - startTime;
	    System.out.println("[TIMESTAMP] Creategeography - Duración: " + duration + "ms");
	    
		return geo;
	}

	/********** 3) Crear grilla de agentes **********/
	private Grid<GisAgent> createAgentGrid(Context context) {
		long startTime = System.currentTimeMillis();
		
		Grid<GisAgent> grid = GridFactoryFinder.createGridFactory(null).createGrid("AgentGrid", context,
				new GridBuilderParameters<>(new StickyBorders(), new SimpleGridAdder<>(), true, GRID_WIDTH, GRID_HEIGHT));
		System.out.println("DEBUG: Grilla para Agentes creada.");
		
		long endTime = System.currentTimeMillis();
	    long duration = endTime - startTime;
	    System.out.println("[TIMESTAMP] createAgentGrid - Duración: " + duration + "ms");
	    
		return grid;
	}

	/*** 4) Inicialización de zonas y carreteras ****/
	private void initializeZonesAndRoads(Context context, GeometryFactory fac) {
		long startTime = System.currentTimeMillis();
		System.out.println("DEBUG: Cargando shapefile de zonas y marcando celdas...");

		// 4.1) Carreteras
		loadAndMarkRoads();
				
		// 4.2) Zona inicial
		this.initialZone = loadAndMarkInitialZone(context);

		// 4.3) Zona segura
		this.safeZone = loadAndMarkSafeZone(context);

		// 4.4) Obtener puntos de la zona segura
		collectSafeZonePoints(geography, this.safeZone, fac);

		// 4.5) Validar zonas
		validateZones();
		
		long endTime = System.currentTimeMillis();
	    long duration = endTime - startTime;
	    System.out.println("[TIMESTAMP] InitializeZonesAndRoads - Duración: " + duration + "ms");
	}
	
	// 4.1) Carreteras
	private void loadAndMarkRoads() {
		long startTime = System.currentTimeMillis();
		
		List<SimpleFeature> roadFeatures = loadFeaturesFromShapefile(ROADS_SHAPEFILE_PATH);
		if (roadFeatures.isEmpty()) {
			System.err.println("ERROR: El shapefile " + ROADS_SHAPEFILE_PATH + " está vacío.");
			return;
		}

		for (SimpleFeature feature : roadFeatures) {
			Geometry geom = (Geometry) feature.getDefaultGeometry();
			if (geom instanceof LineString) {
				markGridCellsForLineString((LineString) geom, MapCell.TYPE_ROAD);
			} else if (geom instanceof MultiLineString) {
				MultiLineString mls = (MultiLineString) geom;
				for (int i = 0; i < mls.getNumGeometries(); i++) {
					Geometry singleGeom = mls.getGeometryN(i);
					if (singleGeom instanceof LineString) {
						markGridCellsForLineString((LineString) singleGeom, MapCell.TYPE_ROAD);
					}
				}
			}
		}
		System.out.println("DEBUG: Celdas para carreteras marcadas.");
		long endTime = System.currentTimeMillis();
	    long duration = endTime - startTime;
	    System.out.println("[TIMESTAMP] Cargar y marcar carreteras - Duración: " + duration + "ms");
	}

	// 4.2) Zona inicial
	private ZoneAgent loadAndMarkInitialZone(Context context) {
		long startTime = System.currentTimeMillis();
		
		List<SimpleFeature> features = loadFeaturesFromShapefile(INITIAL_ZONE_SHAPEFILE_PATH);
		if (features.isEmpty()) {
			throw new IllegalStateException(INITIAL_ZONE_SHAPEFILE_PATH + " está vacío.");
		}

		Geometry geometry = extractGeometryFromFeature(features.iterator().next());
		ZoneAgent zone = new ZoneAgent("initial");
		context.add(zone);
		geography.move(zone, geometry);
		
		markGridCellsForZone(geometry, MapCell.TYPE_INITIAL_ZONE, MapCell.TYPE_ROAD_IN_INITIAL);
		//markGridCellsForGeometry(geometry, MapCell.TYPE_INITIAL_ZONE);

		System.out.println("DEBUG: Celdas para zona inicial marcadas.");
		long endTime = System.currentTimeMillis();
	    long duration = endTime - startTime;
	    System.out.println("[TIMESTAMP] Zona inicial - Duración: " + duration + "ms");
	    
		return zone;
	}

	// 4.3) Zona segura
	private ZoneAgent loadAndMarkSafeZone(Context context) {
		long startTime = System.currentTimeMillis();
		
		List<SimpleFeature> features = loadFeaturesFromShapefile(SAFE_ZONE_SHAPEFILE_PATH);
		if (features.isEmpty()) {
			throw new IllegalStateException(SAFE_ZONE_SHAPEFILE_PATH + " está vacío.");
		}

		Geometry geometry = extractGeometryFromFeature(features.iterator().next());
		ZoneAgent zone = new ZoneAgent("safe");
		context.add(zone);
		geography.move(zone, geometry);
		
		markGridCellsForZone(geometry, MapCell.TYPE_SAFE_ZONE, MapCell.TYPE_ROAD_IN_SAFE);
		//markGridCellsForGeometry(geometry, MapCell.TYPE_SAFE_ZONE);

		System.out.println("DEBUG: Celdas para zona segura marcadas.");
		long endTime = System.currentTimeMillis();
	    long duration = endTime - startTime;
	    System.out.println("[TIMESTAMP] Zona segura - Duración: " + duration + "ms");
	    
		return zone;
	}
	
	public static GridPoint getSafeZoneTarget() {
	    if (safeZonePoints == null || safeZonePoints.isEmpty()) {
	        System.err.println("ERROR: No hay puntos transitables disponibles en la zona segura.");
	        return null;
	    }
	    
	    int randomIndex = repast.simphony.random.RandomHelper.nextIntFromTo(0, safeZonePoints.size() - 1);
	    return safeZonePoints.get(randomIndex);
	}
	
	private void markGridCellsForZone(Geometry geometry, int zoneType, int roadInZoneType) {
	    GeometryFactory fac = new GeometryFactory();
	    int cellsMarked = 0;
	    int roadsCombined = 0;
	    
	    for (int x = 0; x < GRID_WIDTH; x++) {
	        for (int y = 0; y < GRID_HEIGHT; y++) {
	            Coordinate cellCenterGeoCoord = mapGridToGeo(x, y);
	            Point cellCenterPoint = fac.createPoint(cellCenterGeoCoord);
	            
	            if (geometry.contains(cellCenterPoint) || geometry.intersects(cellCenterPoint)) {
	                GridPoint gp = new GridPoint(x, y);
	                Integer currentType = mapCellGrid.get(gp);
	                
	                // Si ya es carretera, cambiar a carretera-en-zona
	                if (currentType != null && currentType == MapCell.TYPE_ROAD) {
	                    mapCellGrid.put(gp, roadInZoneType);
	                    roadsCombined++;
	                } else if (currentType == null || currentType == MapCell.TYPE_EMPTY) {
	                    // Solo zona si está vacío
	                    mapCellGrid.put(gp, zoneType);
	                    cellsMarked++;
	                }
	            }
	        }
	    }
	    
	    System.out.println("DEBUG: Celdas de zona marcadas: " + cellsMarked + 
	                      ", Carreteras combinadas: " + roadsCombined);
	}
	
	// 4.4) Obtener puntos de zona segura
	public static void collectSafeZonePoints(Geography geography, ZoneAgent safeZone, GeometryFactory fac) {
	    long startTime = System.currentTimeMillis();
	    
	    safeZonePoints = new ArrayList<>();
	    
	    Geometry safeZoneGeom = geography.getGeometry(safeZone);
	    if (safeZoneGeom == null) {
	        System.err.println("ERROR: Geometría de la zona segura es nula.");
	        return;
	    }

	    Envelope bbox = safeZoneGeom.getEnvelopeInternal();
	    GridPoint minGp = mapGeoToGrid(new Coordinate(bbox.getMinX(), bbox.getMinY()));
	    GridPoint maxGp = mapGeoToGrid(new Coordinate(bbox.getMaxX(), bbox.getMaxY()));
	    
	    // Recorrer las celdas dentro del bounding box
	    for (int x = minGp.getX(); x <= maxGp.getX(); x++) {
	        for (int y = minGp.getY(); y <= maxGp.getY(); y++) {
	            // Verificar si está dentro de los límites del grid
	            if (x >= 0 && x < GRID_WIDTH && y >= 0 && y < GRID_HEIGHT) {
	                int cellType = getMapCell(x, y);
	                
	                if (MapCell.isTraversable(cellType)) {
	                    Coordinate cellCenterGeoCoord = mapGridToGeo(x, y);
	                    Point cellCenterPoint = fac.createPoint(cellCenterGeoCoord);
	                    
	                    
	                    // Verificar si el punto está dentro de la zona segura
	                    if (safeZoneGeom.contains(cellCenterPoint)) {
	                        safeZonePoints.add(new GridPoint(x, y));
	                    }
	                }
	            }
	        }
	    }
	    
	    System.out.println("DEBUG: " + safeZonePoints.size() + 
	                      " puntos transitables encontrados en zona segura.");
	    
	    if (safeZonePoints.isEmpty()) {
	        System.err.println("ERROR: No se encontraron puntos transitables en la zona segura.");
	    }
	    
	    long endTime = System.currentTimeMillis();
	    long duration = endTime - startTime;
	    System.out.println("[TIMESTAMP] Recopilación de puntos transitables - Duración: " + duration + "ms");
	}
	// 4.4) Carreteras

	// 4.5) Validar zonas
	private void validateZones() {
		long startTime = System.currentTimeMillis();
		
		if (initialZone == null || geography.getGeometry(initialZone) == null) {
			throw new IllegalStateException("Initial Zone no creada o sin geometría.");
		}
		if (safeZone == null || geography.getGeometry(safeZone) == null) {
			throw new IllegalStateException("Safe Zone no encontrada o sin geometría.");
		}
		long endTime = System.currentTimeMillis();
	    long duration = endTime - startTime;
	    System.out.println("[TIMESTAMP] Validar Zonas - Duración: " + duration + "ms");
	}

	
	/**** 5) Inicializar agentes en la grilla ****/
	private void initializeAgents(Context context, GeometryFactory fac) {
		long startTime = System.currentTimeMillis();
		System.out.println("DEBUG: Iniciando creación de agentes.");

		// 5.1 Buscar celdas disponibles dentro de zona inicial (transitables)
		List<GridPoint> spawnCells = findCellsInInitialZone(fac);

		// 5.2 Posicionar agentes
		spawnAgents(context, fac, spawnCells);

		long endTime = System.currentTimeMillis();
	    long duration = endTime - startTime;
	    System.out.println("[TIMESTAMP] Posicionar Agentes - Duración: " + duration + "ms");
	}
	
	// 5.1) Buscar celdas de carreteras dentro de zona inicial
	private List<GridPoint> findCellsInInitialZone(GeometryFactory fac) {
		long startTime = System.currentTimeMillis();

		List<GridPoint> spawnCells = new ArrayList<>();
	    Geometry initialZoneGeometry = geography.getGeometry(initialZone);
	    
	    Envelope bbox = initialZoneGeometry.getEnvelopeInternal();
	    GridPoint minGp = mapGeoToGrid(new Coordinate(bbox.getMinX(), bbox.getMinY()));
	    GridPoint maxGp = mapGeoToGrid(new Coordinate(bbox.getMaxX(), bbox.getMaxY()));

	    for (int x = minGp.getX(); x <= maxGp.getX(); x++) {
	        for (int y = minGp.getY(); y <= maxGp.getY(); y++) {
	            int cellType = getMapCell(x, y);
	            
	            // Solo agregar si es carretera en zona inicial
	            if (MapCell.isValidSpawnCell(cellType)) {
	                spawnCells.add(new GridPoint(x, y));
	            }
	        }
	    }
	    
	    if (spawnCells.isEmpty()) {
	        throw new IllegalStateException(
	            "No hay carreteras (TYPE_ROAD_IN_INITIAL) en zona inicial. ");
	    }

	    //System.out.println("DEBUG: " + spawnCells.size() + " celdas disponibles para spawn.");
		long endTime = System.currentTimeMillis();
	    long duration = endTime - startTime;
	    System.out.println("[TIMESTAMP] findCellsInitialZone - Duración: " + duration + "ms");

		return spawnCells;
	}
	
	// 5.2) Posicionar agentes
	private void spawnAgents(Context context, GeometryFactory fac, List<GridPoint> spawnCells) {
		long startTime = System.currentTimeMillis();
		int cnt = 0;
		for (int i = 0; i < numAgents; i++) {
			GridPoint agentStartGridPoint = spawnCells
					.get(repast.simphony.random.RandomHelper.nextIntFromTo(0, spawnCells.size() - 1));

			GisAgent agent = new GisAgent("Agent " + cnt, safeZone, mapCellGrid, agentGrid, preEvacuationTimeMin, preEvacuationTimeMax);
			context.add(agent);
			agentGrid.moveTo(agent, agentStartGridPoint.getX(), agentStartGridPoint.getY());

			Coordinate geoCoord = mapGridToGeo(agentStartGridPoint.getX(), agentStartGridPoint.getY());
			geography.move(agent, fac.createPoint(geoCoord));
			cnt++;
		}
		
		long endTime = System.currentTimeMillis();
	    long duration = endTime - startTime;
	    System.out.println("[TIMESTAMP] Posicionar agentes - Duración: " + duration + "ms");
	}
	
	
	
/********** 6) Recolección de datos **********/
	private void addDataCollector(Context context) {
		EvacuationData dataCollector = new EvacuationData(numAgents);
		context.add(dataCollector);
	}


	// ===============================
	// |          Geometry           |
	// ===============================
	// ====================================================
	// |     Extraer geometrías de features del shp       |
	// ====================================================
private Geometry extractGeometryFromFeature(SimpleFeature feature) {
		Geometry rawGeometry = (Geometry) feature.getDefaultGeometry();
		Geometry geometryToUse = rawGeometry;

		if (rawGeometry instanceof MultiPolygon) {
			MultiPolygon mp = (MultiPolygon) rawGeometry;
			if (mp.getNumGeometries() > 0) {
				geometryToUse = mp.getGeometryN(0);
			}
		}

		return geometryToUse;
	}


	// ========================================
	// |            Marcar celdas             |
	// ========================================
	private void markGridCellsForGeometry(Geometry geometry, int cellType) {
		GeometryFactory fac = new GeometryFactory();
		for (int x = 0; x < GRID_WIDTH; x++) {
			for (int y = 0; y < GRID_HEIGHT; y++) {
				Coordinate cellCenterGeoCoord = mapGridToGeo(x, y);
				Point cellCenterPoint = fac.createPoint(cellCenterGeoCoord);
				if (geometry.contains(cellCenterPoint) || geometry.intersects(cellCenterPoint)) {
					mapCellGrid.put(new GridPoint(x, y), cellType);
				}
			}
		}
	}
	

	private void markGridCellsForLineString(LineString line, int cellType) {
	    Coordinate[] coords = line.getCoordinates();
	    if (coords.length < 2) return;
	    
	    for (int i = 0; i < coords.length - 1; i++) {
	        GridPoint p1 = mapGeoToGrid(coords[i]);
	        GridPoint p2 = mapGeoToGrid(coords[i + 1]);
	        
	        // Bresenham para trazar la línea entre puntos
	        markLineBresenham(p1.getX(), p1.getY(), p2.getX(), p2.getY(), cellType);
	    }
	}
	
	// Marcar las lineas de los caminos

	// Marcar las lineas de las carreteras
	private void markLineBresenham(int x0, int y0, int x1, int y1, int cellType) {
	    int dx = Math.abs(x1 - x0);
	    int dy = Math.abs(y1 - y0);
	    int sx = x0 < x1 ? 1 : -1;
	    int sy = y0 < y1 ? 1 : -1;
	    int err = dx - dy;
	    
	    while (true) {
	        if (x0 >= 0 && x0 < GRID_WIDTH && y0 >= 0 && y0 < GRID_HEIGHT) {
	            mapCellGrid.put(new GridPoint(x0, y0), cellType);
	        }
	        
	        if (x0 == x1 && y0 == y1) break;
	        
	        int e2 = 2 * err;
	        if (e2 > -dy) { err -= dy; x0 += sx; }
	        if (e2 < dx) { err += dx; y0 += sy; }
	    }
	}
	

	// ========================================
	// |            Cargar features           |
	// ========================================
	private List<SimpleFeature> loadFeaturesFromShapefile(String filename) {
		URL url = null;
		try {
			url = new File(filename).toURI().toURL();
		} catch (MalformedURLException e1) {
			e1.printStackTrace();
		}

		List<SimpleFeature> features = new ArrayList<>();

		SimpleFeatureIterator fiter = null;
		ShapefileDataStore store = null;
		try {
			store = new ShapefileDataStore(url);
			fiter = store.getFeatureSource().getFeatures().features();

			while (fiter.hasNext()) {
				features.add(fiter.next());
			}
		} catch (IOException e) {
			System.err.println("Error al cargar el shapefile " + filename + ": " + e.getMessage());
			e.printStackTrace();
		} finally {
			if (fiter != null) {
				fiter.close();
			}
			if (store != null) {
				store.dispose();
			}
		}

		return features;
	}
	
	// ====================================
	// |         Transformaciones         |
	// ====================================
	private static double[] geoToECEF(double lat, double lon) {
		double latRad = Math.toRadians(lat);
		double lonRad = Math.toRadians(lon);

		double sinLat = Math.sin(latRad);
		double cosLat = Math.cos(latRad);
		double sinLon = Math.sin(lonRad);
		double cosLon = Math.cos(lonRad);

		double N = WGS84_A / Math.sqrt(1 - WGS84_E2 * sinLat * sinLat);

		double X = N * cosLat * cosLon;
		double Y = N * cosLat * sinLon;
		double Z = N * (1 - WGS84_E2) * sinLat;

		return new double[] { X, Y, Z };
	}

	private static double[] ecefToENU(double[] ecef, double[] refECEF, double refLat, double refLon) {
		double deltaX = ecef[0] - refECEF[0];
		double deltaY = ecef[1] - refECEF[1];
		double deltaZ = ecef[2] - refECEF[2];

		double refLatRad = Math.toRadians(refLat);
		double refLonRad = Math.toRadians(refLon);

		double sinLat = Math.sin(refLatRad);
		double cosLat = Math.cos(refLatRad);
		double sinLon = Math.sin(refLonRad);
		double cosLon = Math.cos(refLonRad);

		double east = -sinLon * deltaX + cosLon * deltaY;
		double north = -sinLat * cosLon * deltaX - sinLat * sinLon * deltaY + cosLat * deltaZ;
		double up = cosLat * cosLon * deltaX + cosLat * sinLon * deltaY + sinLat * deltaZ;

		return new double[] { east, north, up };
	}
	

	private static double[] enuToECEF(double east, double north, double up, double[] refECEF, double refLat,
			double refLon) {
		double refLatRad = Math.toRadians(refLat);
		double refLonRad = Math.toRadians(refLon);

		double sinLat = Math.sin(refLatRad);
		double cosLat = Math.cos(refLatRad);
		double sinLon = Math.sin(refLonRad);
		double cosLon = Math.cos(refLonRad);

		double deltaX = -sinLon * east - sinLat * cosLon * north + cosLat * cosLon * up;
		double deltaY = cosLon * east - sinLat * sinLon * north + cosLat * sinLon * up;
		double deltaZ = cosLat * north + sinLat * up;

		return new double[] { refECEF[0] + deltaX, refECEF[1] + deltaY, refECEF[2] + deltaZ };
	}


	private static double[] ecefToGeo(double[] ecef) {
		double X = ecef[0];
		double Y = ecef[1];
		double Z = ecef[2];

		double p = Math.sqrt(X * X + Y * Y);
		double lon = Math.atan2(Y, X);
		double lat = Math.atan2(Z, p * (1 - WGS84_E2));
		double h = 0;

		for (int i = 0; i < 3; i++) {
			double sinLat = Math.sin(lat);
			double N = WGS84_A / Math.sqrt(1 - WGS84_E2 * sinLat * sinLat);
			h = p / Math.cos(lat) - N;
			lat = Math.atan2(Z, p * (1 - WGS84_E2 * N / (N + h)));
		}
		return new double[] { Math.toDegrees(lat), Math.toDegrees(lon), h };
	}
	

	public static GridPoint mapGeoToGrid(Coordinate geoCoord) {
		double[] pointECEF = geoToECEF(geoCoord.y, geoCoord.x);
		double[] enu = ecefToENU(pointECEF, REF_ECEF, REF_LAT, REF_LON);

		double shiftX = enu[0] + AREA_WIDTH_METERS / 2.0;
		double shiftY = enu[1] + AREA_HEIGHT_METERS / 2.0;

		int gridX = (int) Math.round(shiftX / CELL_SIZE_METERS);
		int gridY = (int) Math.round(shiftY / CELL_SIZE_METERS);

		gridX = Math.max(0, Math.min(gridX, GRID_WIDTH - 1));
		gridY = Math.max(0, Math.min(gridY, GRID_HEIGHT - 1));

		return new GridPoint(gridX, gridY);
	}
	public static Coordinate mapGridToGeo(int gridX, int gridY) {
		double east = (gridX + 0.5) * CELL_SIZE_METERS - AREA_WIDTH_METERS / 2;
		double north = (gridY + 0.5) * CELL_SIZE_METERS - AREA_HEIGHT_METERS / 2;
		double up = 0.0;

		double[] ecef = enuToECEF(east, north, up, REF_ECEF, REF_LAT, REF_LON);
		double[] geo = ecefToGeo(ecef);

		return new Coordinate(geo[1], geo[0]);
	}

/*	public static void testTransformations() {
		System.out.println("=== PRUEBA DE TRANSFORMACIONES ===");
		System.out.printf("Área: %.1f x %.1f metros\n", AREA_WIDTH_METERS, AREA_HEIGHT_METERS);
		System.out.printf("Punto de referencia: %.8f, %.8f\n", REF_LAT, REF_LON);

		Coordinate[] testPoints = { new Coordinate(MIN_LONGITUDE, MIN_LATITUDE),
				new Coordinate(MAX_LONGITUDE, MIN_LATITUDE), new Coordinate(MAX_LONGITUDE, MAX_LATITUDE),
				new Coordinate(MIN_LONGITUDE, MAX_LATITUDE), new Coordinate(REF_LON, REF_LAT) };

		String[] names = { "SW", "SE", "NE", "NW", "Centro" };

		for (int i = 0; i < testPoints.length; i++) {
			Coordinate original = testPoints[i];
			GridPoint grid = mapGeoToGrid(original);
			Coordinate reconstructed = mapGridToGeo(grid.getX(), grid.getY());

			double errorMeters = distanceInMeters(original, reconstructed);

			System.out.printf("%s: (%.8f, %.8f) -> (%d, %d) -> (%.8f, %.8f) | Error: %.3f m\n", names[i], original.x,
					original.y, grid.getX(), grid.getY(), reconstructed.x, reconstructed.y, errorMeters);
		}
	}
*/

/*    private static double distanceInMeters(Coordinate c1, Coordinate c2) {
		double[] ecef1 = geoToECEF(c1.y, c1.x);
		double[] ecef2 = geoToECEF(c2.y, c2.x);

		double dx = ecef1[0] - ecef2[0];
		double dy = ecef1[1] - ecef2[1];
		double dz = ecef1[2] - ecef2[2];

		return Math.sqrt(dx * dx + dy * dy + dz * dz);
	}
*/
    
	/*************** Estructuras **************/// ===============================
	// |       Estructuras           |
	// ===============================
    public static class GridPoint {
		private final int x;
		private final int y;

		public GridPoint(int x, int y) {
			this.x = x;
			this.y = y;
		}

		public int getX() {
			return x;
		}

		public int getY() {
			return y;
		}

		@Override
		public boolean equals(Object o) {
			if (this == o) {
				return true;
			}
			if (o == null || getClass() != o.getClass()) {
				return false;
			}
			GridPoint gridPoint = (GridPoint) o;
			return x == gridPoint.x && y == gridPoint.y;
		}

		@Override
		public int hashCode() {
			return 31 * x + y;
		}
	}

	
    private static class BboxLimits {
		final double minLon;
		final double maxLon;
		final double minLat;
		final double maxLat;

		BboxLimits(double minLon, double maxLon, double minLat, double maxLat) {
			this.minLon = minLon;
			this.maxLon = maxLon;
			this.minLat = minLat;
			this.maxLat = maxLat;
		}
	}

    private static class MetricDimensions {
		final double widthMeters;
		final double heightMeters;

		MetricDimensions(double widthMeters, double heightMeters) {
			this.widthMeters = widthMeters;
			this.heightMeters = heightMeters;
		}
	}
	
	// ===============================
	// |          Getters            |
	// ===============================
	/*************** Getters ***************/
public ZoneAgent getInitialZone() {
		return initialZone;
	}

	public ZoneAgent getSafeZone() {
		return safeZone;
	}

	public static int getMapCell(int x, int y) {
		if (x >= 0 && x < GRID_WIDTH && y >= 0 && y < GRID_HEIGHT) {
			return mapCellGrid.getOrDefault(new GridPoint(x, y), 0);
		}
		return 0;
	}

    public static double getSpeedByDistribution(Random random) {
        double rand = random.nextDouble() * 100.0;
        
        if (rand < percentSpeed_0_5) {
            return 0.5;
        } else if (rand < percentSpeed_0_5 + percentSpeed_1_0) {
            return 1.0;
        } else {
            return 1.5;
        }
    }
    
	public static synchronized void incrementEvacuated() {
		EVACUATED_TOTAL++;	
	}
	public static synchronized void incrementReachedTarget() {
	    REACHED_TARGET_TOTAL++;
	}
	
}