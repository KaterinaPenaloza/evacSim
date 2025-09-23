package evacSim.agents;

import java.io.File;
import java.io.IOException;
import java.net.MalformedURLException;
import java.net.URL;
import java.util.ArrayList;
import java.util.List;

import org.geotools.data.shapefile.ShapefileDataStore;
import org.geotools.data.simple.SimpleFeatureIterator;
import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.LineString;
import org.locationtech.jts.geom.MultiLineString;
import org.locationtech.jts.geom.MultiPolygon;
import org.locationtech.jts.geom.Point;
import org.locationtech.jts.geom.Polygon;
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
import repast.simphony.space.grid.GridPoint;
import repast.simphony.space.grid.SimpleGridAdder;
import repast.simphony.space.grid.StickyBorders;

@SuppressWarnings("rawtypes")
public class ContextCreator implements ContextBuilder {
    int numAgents;
    private ZoneAgent initialZone;
    private ZoneAgent safeZone;

    // Grilla
 // Grilla: Calcular dinámicamente para celdas de ~5 metros
    public static final double CELL_SIZE_METERS = 5.0; // Tamaño deseado de cada celda
    private static final int GRID_WIDTH;
    private static final int GRID_HEIGHT;
    
    /* centro viña*/
    private static final String INITIAL_ZONE_SHAPEFILE_PATH = "./data/POLYGON.shp";
    private static final String SAFE_ZONE_SHAPEFILE_PATH = "./data/Zones2.shp";
    private static final String ROADS_SHAPEFILE_PATH = "./data/roads.shp";

    /*forestal*/ 
    //private static final String INITIAL_ZONE_SHAPEFILE_PATH = "./data/map_data_initial_zones.shp";
    //private static final String SAFE_ZONE_SHAPEFILE_PATH = "./data/map_data_safe_zones.shp";
    //private static final String ROADS_SHAPEFILE_PATH = "./data/map_data_roads.shp";

    // Definición de límites geográficos
    /*centro de viña*/
    private static final double MIN_LONGITUDE = -71.55435632438811;
    private static final double MAX_LONGITUDE = -71.54257547377763;
    private static final double MIN_LATITUDE = -33.026878836652145;
    private static final double MAX_LATITUDE = -33.00825336021646;

    /*forestal*/ 
    //private static final double MIN_LONGITUDE = -71.55738537364508;
    //private static final double MAX_LONGITUDE = -71.52906755695355;
    //private static final double MIN_LATITUDE = -33.067424934437135;
    //private static final double MAX_LATITUDE = -33.02551864106715;

    // Constantes WGS84
    private static final double WGS84_A = 6378137.0;           // Semi-eje mayor
    private static final double WGS84_F = 1.0 / 298.257223563; // Aplanamiento
    private static final double WGS84_E2 = WGS84_F * (2 - WGS84_F); // e²

    // Punto de referencia (centro del área)
    private static final double REF_LAT = (MIN_LATITUDE + MAX_LATITUDE) / 2;
    private static final double REF_LON = (MIN_LONGITUDE + MAX_LONGITUDE) / 2;

    // Dimensiones del área en metros (calculadas una vez)
    private static final double AREA_WIDTH_METERS;
    private static final double AREA_HEIGHT_METERS;
    private static final double[] REF_ECEF;

    
    static {
        // Calcular dimensiones reales del área en metros
    	// Transformación a radianes
        double refLatRad = Math.toRadians(REF_LAT);
        AREA_WIDTH_METERS = Math.toRadians(MAX_LONGITUDE - MIN_LONGITUDE) * Math.cos(refLatRad) * WGS84_A;
        AREA_HEIGHT_METERS = Math.toRadians(MAX_LATITUDE - MIN_LATITUDE) * WGS84_A;
        
        // Calcular tamaño de grilla para celdas de ~5 metros
        GRID_WIDTH = (int) Math.ceil(AREA_WIDTH_METERS / CELL_SIZE_METERS);
        GRID_HEIGHT = (int) Math.ceil(AREA_HEIGHT_METERS / CELL_SIZE_METERS);
        
        // Coordenadas ECEF del punto de referencia
        REF_ECEF = geoToECEF(REF_LAT, REF_LON);
    }
    
    private static Grid<MapCell> mapCellGrid;
    private static Grid<GisAgent> agentGrid;

    /**
     * Convierte coordenadas geográficas a ECEF
     */
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
        
        return new double[]{X, Y, Z};
    }

    /**
     * Convierte ECEF a coordenadas ENU usando punto de referencia
     */
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
        
        return new double[]{east, north, up};
    }

    /**
     * Convierte ENU a ECEF
     */
    private static double[] enuToECEF(double east, double north, double up, double[] refECEF, double refLat, double refLon) {
        double refLatRad = Math.toRadians(refLat);
        double refLonRad = Math.toRadians(refLon);
        
        double sinLat = Math.sin(refLatRad);
        double cosLat = Math.cos(refLatRad);
        double sinLon = Math.sin(refLonRad);
        double cosLon = Math.cos(refLonRad);
        
        // Matriz transpuesta de ECEF a ENU
        double deltaX = -sinLon * east - sinLat * cosLon * north + cosLat * cosLon * up;
        double deltaY = cosLon * east - sinLat * sinLon * north + cosLat * sinLon * up;
        double deltaZ = cosLat * north + sinLat * up;
        
        return new double[]{
            refECEF[0] + deltaX,
            refECEF[1] + deltaY,
            refECEF[2] + deltaZ
        };
    }

    /**
     * Convierte ECEF a coordenadas geográficas
     */
    private static double[] ecefToGeo(double[] ecef) {
        double X = ecef[0];
        double Y = ecef[1];
        double Z = ecef[2];
        
        double p = Math.sqrt(X * X + Y * Y);
        double lon = Math.atan2(Y, X);
        
        // Iteración para latitud (método mejorado)
        double lat = Math.atan2(Z, p * (1 - WGS84_E2));
        double h = 0;
        
        for (int i = 0; i < 3; i++) {
            double sinLat = Math.sin(lat);
            double N = WGS84_A / Math.sqrt(1 - WGS84_E2 * sinLat * sinLat);
            h = p / Math.cos(lat) - N;
            lat = Math.atan2(Z, p * (1 - WGS84_E2 * N / (N + h)));
        }
        
        return new double[]{Math.toDegrees(lat), Math.toDegrees(lon), h};
    }

    /**
     * Conversión mejorada de coordenadas geográficas a grilla
     */
    public static GridPoint mapGeoToGrid(Coordinate geoCoord) {
        double[] pointECEF = geoToECEF(geoCoord.y, geoCoord.x);
        double[] enu = ecefToENU(pointECEF, REF_ECEF, REF_LAT, REF_LON);
        
        // Normalizar a [0, 1] basado en las dimensiones del área
        double normX = (enu[0] + AREA_WIDTH_METERS / 2) / AREA_WIDTH_METERS;
        double normY = (enu[1] + AREA_HEIGHT_METERS / 2) / AREA_HEIGHT_METERS;
        
        // Mapear a grilla (usar GRID_WIDTH directamente)
        int gridX = (int) Math.round(normX * GRID_WIDTH);
        int gridY = (int) Math.round(normY * GRID_HEIGHT);
        
        // Asegurar límites
        gridX = Math.max(0, Math.min(gridX, GRID_WIDTH - 1));
        gridY = Math.max(0, Math.min(gridY, GRID_HEIGHT - 1));
        
        return new GridPoint(gridX, gridY);
    }

    /**
     * Conversión mejorada de coordenadas de grilla a geográficas
     */
    public static Coordinate mapGridToGeo(int gridX, int gridY) {
        // Usar centro de la celda para mayor precisión
        double normX = (gridX + 0.5) / GRID_WIDTH;
        double normY = (gridY + 0.5) / GRID_HEIGHT;
        
        // Convertir a coordenadas ENU en metros
        double east = normX * AREA_WIDTH_METERS - AREA_WIDTH_METERS / 2;
        double north = normY * AREA_HEIGHT_METERS - AREA_HEIGHT_METERS / 2;
        double up = 0.0;
        
        // Convertir ENU a ECEF
        double[] ecef = enuToECEF(east, north, up, REF_ECEF, REF_LAT, REF_LON);
        
        // Convertir ECEF a coordenadas geográficas
        double[] geo = ecefToGeo(ecef);
        
        return new Coordinate(geo[1], geo[0]); // lon, lat
    }

    /**
     * Método de prueba para verificar la precisión de las transformaciones
     */
    public static void testTransformations() {
        System.out.println("=== PRUEBA DE TRANSFORMACIONES ===");
        System.out.printf("Área: %.1f x %.1f metros\n", AREA_WIDTH_METERS, AREA_HEIGHT_METERS);
        System.out.printf("Punto de referencia: %.8f, %.8f\n", REF_LAT, REF_LON);
        
        // Probar esquinas del área
        Coordinate[] testPoints = {
            new Coordinate(MIN_LONGITUDE, MIN_LATITUDE), // SW
            new Coordinate(MAX_LONGITUDE, MIN_LATITUDE), // SE  
            new Coordinate(MAX_LONGITUDE, MAX_LATITUDE), // NE
            new Coordinate(MIN_LONGITUDE, MAX_LATITUDE), // NW
            new Coordinate(REF_LON, REF_LAT)            // Centro
        };
        
        String[] names = {"SW", "SE", "NE", "NW", "Centro"};
        
        for (int i = 0; i < testPoints.length; i++) {
            Coordinate original = testPoints[i];
            GridPoint grid = mapGeoToGrid(original);
            Coordinate reconstructed = mapGridToGeo(grid.getX(), grid.getY());
            
            double errorMeters = distanceInMeters(original, reconstructed);
            
            System.out.printf("%s: (%.8f, %.8f) -> (%d, %d) -> (%.8f, %.8f) | Error: %.3f m\n",
                names[i], original.x, original.y, grid.getX(), grid.getY(),
                reconstructed.x, reconstructed.y, errorMeters);
        }
    }

    /**
     * Calcula distancia aproximada entre dos coordenadas en metros
     */
    private static double distanceInMeters(Coordinate c1, Coordinate c2) {
        double[] ecef1 = geoToECEF(c1.y, c1.x);
        double[] ecef2 = geoToECEF(c2.y, c2.x);
        
        double dx = ecef1[0] - ecef2[0];
        double dy = ecef1[1] - ecef2[1];
        double dz = ecef1[2] - ecef2[2];
        
        return Math.sqrt(dx*dx + dy*dy + dz*dz);
    }

    /**
     * Calcula distancia usando la fórmula Haversine
     */
    private static double haversineDistance(Coordinate c1, Coordinate c2) {
        double lat1 = Math.toRadians(c1.y);
        double lon1 = Math.toRadians(c1.x);
        double lat2 = Math.toRadians(c2.y);
        double lon2 = Math.toRadians(c2.x);
        double dLat = lat2 - lat1;
        double dLon = lon2 - lon1;
        double a = Math.sin(dLat / 2) * Math.sin(dLat / 2) +
                   Math.cos(lat1) * Math.cos(lat2) * Math.sin(dLon / 2) * Math.sin(dLon / 2);
        double c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
        return 6371000 * c; // Radio promedio de la Tierra en metros
    }

    // Obtener MapCell
    public static MapCell getMapCell(int x, int y) {
        if (mapCellGrid != null && x >= 0 && x < GRID_WIDTH && y >= 0 && y < GRID_HEIGHT) {
            return mapCellGrid.getObjectAt(x, y);
        }
        return null;
    }

    @Override
    public Context build(Context context) {
        Parameters parm = RunEnvironment.getInstance().getParameters();
        numAgents = (Integer)parm.getValue("numAgents");

        // Ejecutar prueba de transformaciones
        testTransformations();

        GeographyParameters geoParams = new GeographyParameters();
        Geography geography = GeographyFactoryFinder.createGeographyFactory(null).createGeography("Geography", context, geoParams);
        GeometryFactory fac = new GeometryFactory();

        // GRILLA DE MAPCELL (TERRENO)
        mapCellGrid = GridFactoryFinder.createGridFactory(null).createGrid("MapCellGrid", context,
            GridBuilderParameters.singleOccupancy2D(
                new SimpleGridAdder<MapCell>(),
                new StickyBorders(),
                GRID_WIDTH, GRID_HEIGHT
            )
        );
        System.out.println("DEBUG: Grilla MapCells creada.");

        // Inicializar celdas (inicialmente no transitables)
        for (int x = 0; x < GRID_WIDTH; x++) {
            for (int y = 0; y < GRID_HEIGHT; y++) {
                MapCell cell = new MapCell();
                context.add(cell);
                mapCellGrid.moveTo(cell, x, y);
            }
        }

        // Inicialización de grilla para agentes
        agentGrid = GridFactoryFinder.createGridFactory(null).createGrid("AgentGrid", context,
            GridBuilderParameters.multiOccupancy2D(
                new SimpleGridAdder<GisAgent>(),
                new StickyBorders(),
                GRID_WIDTH, GRID_HEIGHT
            )
        );
        System.out.println("DEBUG: Grilla para Agentes creada.");

        // Carga de las Zonas
        System.out.println("DEBUG: Cargando shapefile de zonas y marcando celdas...");

        /**** Zona inicial ****/
        String initialZoneFilename = INITIAL_ZONE_SHAPEFILE_PATH;
        List<SimpleFeature> featuresForInitialZone = loadFeaturesFromShapefile(initialZoneFilename);

        if (featuresForInitialZone.isEmpty()) {
            System.err.println("ERROR: El shapefile " + initialZoneFilename + " está vacío. No se puede definir la zona inicial.");
            throw new IllegalStateException(initialZoneFilename + " está vacío.");
        }

        Geometry rawInitialGeometry = (Geometry)featuresForInitialZone.iterator().next().getDefaultGeometry();
        Geometry initialGeometryToUse = rawInitialGeometry;

        if (rawInitialGeometry instanceof MultiPolygon) {
            MultiPolygon mp = (MultiPolygon)rawInitialGeometry;
            if (mp.getNumGeometries() > 0) {
                Geometry firstPart = mp.getGeometryN(0);
                if (firstPart instanceof Polygon) {
                    initialGeometryToUse = firstPart;
                }
            }
        }
        this.initialZone = new ZoneAgent("initial");
        context.add(this.initialZone);
        geography.move(this.initialZone, initialGeometryToUse);

        markGridCellsForGeometry(initialGeometryToUse, MapCell.TYPE_INITIAL_ZONE);
        System.out.println("DEBUG: Celdas para zona inicial marcadas.");

        /**** Zona segura ****/
        String safeZoneFilename = SAFE_ZONE_SHAPEFILE_PATH;
        List<SimpleFeature> featuresForSafeZone = loadFeaturesFromShapefile(safeZoneFilename);

        if (featuresForSafeZone.isEmpty()) {
            System.err.println("ERROR: El shapefile " + safeZoneFilename + " está vacío. No se puede definir la zona segura.");
            throw new IllegalStateException(safeZoneFilename + " está vacío.");
        }

        Geometry rawSafeGeometry = (Geometry)featuresForSafeZone.iterator().next().getDefaultGeometry();
        Geometry safeGeometryToUse = rawSafeGeometry;

        if (rawSafeGeometry instanceof MultiPolygon) {
            MultiPolygon mp = (MultiPolygon)rawSafeGeometry;
            if (mp.getNumGeometries() > 0) {
                Geometry firstPart = mp.getGeometryN(0);
                if (firstPart instanceof Polygon) {
                    safeGeometryToUse = firstPart;
                }
            }
            if (safeGeometryToUse == null) {
                System.err.println("Advertencia: MultiPolygon de zona segura vacío o no contiene Polygons. Usando MultiPolygon completo si es válido.");
                safeGeometryToUse = rawSafeGeometry;
            }
        }
        this.safeZone = new ZoneAgent("safe");
        context.add(this.safeZone);
        geography.move(this.safeZone, safeGeometryToUse);

        markGridCellsForGeometry(safeGeometryToUse, MapCell.TYPE_SAFE_ZONE);
        System.out.println("DEBUG: Celdas para zona segura marcadas.");

        /**** Carga de las Carreteras ****/
        String roadsShapefile = ROADS_SHAPEFILE_PATH;
        List<SimpleFeature> roadFeatures = loadFeaturesFromShapefile(roadsShapefile);

        if (roadFeatures.isEmpty()) {
            System.err.println("ERROR: El shapefile " + roadsShapefile + " está vacío. No se puede marcar las carreteras en la grilla.");
        } else {
            for (SimpleFeature feature : roadFeatures) {
                Geometry geom = (Geometry)feature.getDefaultGeometry();

                // carreteras = lineas | multilineas
                if (geom instanceof LineString) {
                    markGridCellsForLineString((LineString) geom, MapCell.TYPE_ROAD);
                } else if (geom instanceof MultiLineString) {
                    MultiLineString mls = (MultiLineString) geom;
                    for (int i = 0; i < mls.getNumGeometries(); i++) {
                        Geometry singleGeom = mls.getGeometryN(i);
                        if (singleGeom instanceof LineString) {
                            markGridCellsForLineString((LineString) singleGeom, MapCell.TYPE_ROAD);
                        } else {
                            System.err.println("Advertencia: Geometría inesperada dentro de MultiLineString en " + roadsShapefile + ": " + singleGeom.getGeometryType());
                        }
                    }
                } else {
                    System.err.println("Advertencia: Geometría inesperada en " + roadsShapefile + ": " + geom.getGeometryType() + ". Se espera LineString o MultiLineString.");
                }
            }
            System.out.println("DEBUG: Celdas para carreteras marcadas.");
        }

        // Inicialización de Agentes en la Grilla
        System.out.println("DEBUG: Iniciando creación de agentes.");
        if (initialZone == null || geography.getGeometry(initialZone) == null) {
            System.err.println("ERROR: initial zone no encontrada o geometría no asignada. Los agentes no se podrán crear.");
            throw new IllegalStateException("Initial Zone no creada o sin geometría.");
        }
        if (safeZone == null || geography.getGeometry(safeZone) == null) {
            System.err.println("ERROR: safe zone no encontrada o geometría no asignada.");
            throw new IllegalStateException("Safe Zone no encontrada o sin geometría.");
        }

        // Marcar zonas transitables
        List<GridPoint> initialZoneTraversableCells = new ArrayList<>();
        Geometry initialZoneGeometry = geography.getGeometry(initialZone);

        for (int x = 0; x < GRID_WIDTH; x++) {
            for (int y = 0; y < GRID_HEIGHT; y++) {
                MapCell cell = mapCellGrid.getObjectAt(x,y);
                Coordinate cellCenterGeoCoord = mapGridToGeo(x, y);
                Point cellCenterPoint = fac.createPoint(cellCenterGeoCoord);

                if (cell != null && cell.isTraversable() && initialZoneGeometry.contains(cellCenterPoint)) {
                    initialZoneTraversableCells.add(new GridPoint(x, y));
                }
            }
        }

        if (initialZoneTraversableCells.isEmpty()) {
            System.err.println("ERROR: No se encontraron celdas transitables dentro de la zona inicial. Los agentes no se pueden posicionar.");
            throw new IllegalStateException("No hay celdas transitables en la zona inicial.");
        }
        System.out.println("DEBUG: " + initialZoneTraversableCells.size() + " celdas transitables en la zona inicial.");

        // Posicionar agentes en zonas transitables, dentro de la zona inicial
        int cnt=0;
        for (int i = 0; i < numAgents; i++) {
       
        	GridPoint agentStartGridPoint;
        	
        	if (i == 0) {  // El primer agente (agente 0) usa las coordenadas del test
                Coordinate startGeo = new Coordinate(-71.54265225, -33.03917003);
                agentStartGridPoint = mapGeoToGrid(startGeo);
                System.out.printf("Agente 0 - Punto inicial: (%.8f, %.8f) -> Grilla: (%d, %d)\n", 
                    startGeo.x, startGeo.y, agentStartGridPoint.getX(), agentStartGridPoint.getY());
            } else {  // El resto van random
                agentStartGridPoint = initialZoneTraversableCells.get(
                    repast.simphony.random.RandomHelper.nextIntFromTo(0, initialZoneTraversableCells.size() - 1)
                );
            }

        	
        	GisAgent agent = new GisAgent("Site " + cnt, safeZone, mapCellGrid, agentGrid);
            context.add(agent);
            agentGrid.moveTo(agent, agentStartGridPoint.getX(), agentStartGridPoint.getY());
            Coordinate geoCoord = mapGridToGeo(agentStartGridPoint.getX(), agentStartGridPoint.getY());
            geography.move(agent, fac.createPoint(geoCoord));


         // Configurar objetivo específico para el agente 0
            if (i == 0) {
                Coordinate targetGeo = new Coordinate(-71.54333889, -33.03889571);
                GridPoint targetGrid = mapGeoToGrid(targetGeo);
                agent.safeZoneGridTarget = targetGrid;
                System.out.printf("Agente 0 - Objetivo específico: (%.8f, %.8f) -> Grilla: (%d, %d)\n", 
                    targetGeo.x, targetGeo.y, targetGrid.getX(), targetGrid.getY());
            }
            
            
            cnt++;
        }

        // Ejecutar prueba de evacuación
        //testAgentEvacuation(context, geography, agentGrid, mapCellGrid);

        return context;
    }

    private void markGridCellsForGeometry(Geometry geometry, int cellType) {
        GeometryFactory fac = new GeometryFactory();

        for (int x = 0; x < GRID_WIDTH; x++) {
            for (int y = 0; y < GRID_HEIGHT; y++) {
                MapCell cell = mapCellGrid.getObjectAt(x,y);
                if (cell != null) {
                    Coordinate cellCenterGeoCoord = mapGridToGeo(x, y);
                    Point cellCenterPoint = fac.createPoint(cellCenterGeoCoord);
                    if (geometry.contains(cellCenterPoint) || geometry.intersects(cellCenterPoint)) {
                        cell.setType(cellType);
                    }
                }
            }
        }
    }

    // Marca las celdas de la grilla que son calles
    private void markGridCellsForLineString(LineString line, int cellType) {
        Coordinate[] coords = line.getCoordinates();
        if (coords.length < 2) {
            return;
        }

        GridPoint startGp = mapGeoToGrid(coords[0]);
        if (startGp.getX() >= 0 && startGp.getX() < GRID_WIDTH && startGp.getY() >= 0 && startGp.getY() < GRID_HEIGHT) {
            MapCell cell = mapCellGrid.getObjectAt(startGp.getX(), startGp.getY());
            if (cell != null) {
                cell.setType(cellType);
            }
        }

        // Marcar celdas para cada segmento de línea
        for (int i = 0; i < coords.length - 1; i++) {
            Coordinate p1 = coords[i];
            Coordinate p2 = coords[i+1];

            GridPoint gp1 = mapGeoToGrid(p1);
            GridPoint gp2 = mapGeoToGrid(p2);

            int minX = Math.min(gp1.getX(), gp2.getX());
            int maxX = Math.max(gp1.getX(), gp2.getX());
            int minY = Math.min(gp1.getY(), gp2.getY());
            int maxY = Math.max(gp1.getY(), gp2.getY());

            for (int x = minX; x <= maxX; x++) {
                for (int y = minY; y <= maxY; y++) {
                    if (x >= 0 && x < GRID_WIDTH && y >= 0 && y < GRID_HEIGHT) {
                        MapCell cell = mapCellGrid.getObjectAt(x, y);
                        if (cell != null) {
                            cell.setType(cellType);
                        }
                    }
                }
            }
        }
    }

    // Carga de features shp
    private List<SimpleFeature> loadFeaturesFromShapefile(String filename){
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

            while(fiter.hasNext()){
                features.add(fiter.next());
            }
        } catch (IOException e) {
            System.err.println("Error al cargar el shapefile " + filename + ": " + e.getMessage());
            e.printStackTrace();
        }
        finally{
            if (fiter != null) {
                fiter.close();
            }
            if (store != null) {
                store.dispose();
            }
        }

        return features;
    }

    public ZoneAgent getInitialZone() {
        return initialZone;
    }

    public ZoneAgent getSafeZone() {
        return safeZone;
    }

    /**
     * Test de evacuación de un agente
     */
    public void testAgentEvacuation(Context context, Geography geography, Grid<GisAgent> agentGrid, Grid<MapCell> mapCellGrid) {
        System.out.println("=== PRUEBA DE EVACUACIÓN DE AGENTE ===");
        
        // Verificar que safeZone esté inicializado
        if (this.safeZone == null) {
            System.err.println("Error: safeZone no está inicializado.");
            return;
        }

        // Definir coordenadas iniciales y finales
        Coordinate startGeo = new Coordinate(-71.54265225, -33.03917003); // SW
        Coordinate targetGeo = new Coordinate(-71.54333889, -33.03889571); // NE
       
        // Validar que el punto inicial esté en la zona inicial
        GeometryFactory fac = new GeometryFactory();
        Geometry initialZoneGeometry = geography.getGeometry(initialZone);
        Point startPoint = fac.createPoint(startGeo);
        if (!initialZoneGeometry.contains(startPoint)) {
            System.err.println("Advertencia: Punto inicial no está en la zona inicial. Verifica el shapefile.");
        }

        // Convertir coordenadas iniciales a grilla
        GridPoint startGrid = mapGeoToGrid(startGeo);
        System.out.printf("Punto inicial: (%.8f, %.8f) -> Grilla: (%d, %d)\n", 
            startGeo.x, startGeo.y, startGrid.getX(), startGrid.getY());

        // Verificar que el punto inicial esté en una celda transitable
        MapCell startCell = mapCellGrid.getObjectAt(startGrid.getX(), startGrid.getY());
        if (startCell == null || !startCell.isTraversable()) {
            System.err.println("Error: El punto inicial no está en una celda transitable. Tipo: " + 
                (startCell != null ? startCell.getType() : "null"));
            return;
        }

        // Convertir coordenadas finales a grilla
        GridPoint targetGrid = mapGeoToGrid(targetGeo);
        System.out.printf("Punto objetivo: (%.8f, %.8f) -> Grilla: (%d, %d)\n", 
            targetGeo.x, targetGeo.y, targetGrid.getX(), targetGrid.getY());

        // Verificar que el punto objetivo esté en una celda transitable
        MapCell targetCell = mapCellGrid.getObjectAt(targetGrid.getX(), targetGrid.getY());
        if (targetCell == null || !targetCell.isTraversable()) {
            System.err.println("Error: El punto objetivo no está en una celda transitable. Tipo: " + 
                (targetCell != null ? targetCell.getType() : "null"));
            return;
        }

        // Crear un agente y posicionarlo
        GisAgent agent = new GisAgent("TestAgent", this.safeZone, mapCellGrid, agentGrid);
        context.add(agent);
        agentGrid.moveTo(agent, startGrid.getX(), startGrid.getY());
        geography.move(agent, fac.createPoint(startGeo));

        // Inicializar la geografía del agente (como en GisAgent.step())
        agent.geography = geography;

        // Calcular el camino usando A* de GisAgent
        agent.safeZoneGridTarget = targetGrid;
        agent.calculatePathToSafeZone(startGrid, targetGrid);

        // Verificar si se encontró un camino
        GridPoint nextPoint = agent.getNextPathPoint();
        if (nextPoint == null) {
            System.err.println("Error: No se pudo encontrar un camino transitable al objetivo.");
            System.out.println("Depuración: Estado de celdas cercanas al punto inicial:");
            for (int x = Math.max(0, startGrid.getX() - 2); x <= Math.min(GRID_WIDTH - 1, startGrid.getX() + 2); x++) {
                for (int y = Math.max(0, startGrid.getY() - 2); y <= Math.min(GRID_HEIGHT - 1, startGrid.getY() + 2); y++) {
                    MapCell cell = mapCellGrid.getObjectAt(x, y);
                    String cellType = cell != null ? String.valueOf(cell.getType()) : "null";
                    String traversable = cell != null && cell.isTraversable() ? "transitable" : "no transitable";
                    System.out.printf("Celda (%d, %d): tipo=%s, %s\n", x, y, cellType, traversable);
                }
            }
            return;
        }

        // Simular el movimiento paso a paso
        List<GridPoint> pathTaken = new ArrayList<>();
        pathTaken.add(startGrid);
        while (nextPoint != null && !nextPoint.equals(targetGrid)) {
            if (agent.isValidStep(nextPoint)) {
                agentGrid.moveTo(agent, nextPoint.getX(), nextPoint.getY());
                Coordinate newGeoCoord = mapGridToGeo(nextPoint.getX(), nextPoint.getY());
                geography.move(agent, fac.createPoint(newGeoCoord));
                pathTaken.add(nextPoint);
                System.out.println("DEBUG: Agente movido a: (" + nextPoint.getX() + ", " + nextPoint.getY() + ")");
                agent.advancePathIndex();
                nextPoint = agent.getNextPathPoint();
            } else {
                System.err.println("Advertencia: Celda no transitable o congestionada en (" + nextPoint.getX() + ", " + nextPoint.getY() + "). Recalculando camino.");
                GridPoint currentGrid = agentGrid.getLocation(agent);
                agent.calculatePathToSafeZone(currentGrid, targetGrid);
                nextPoint = agent.getNextPathPoint();
                if (nextPoint == null) {
                    System.err.println("Error: No se pudo recalcular un camino transitable.");
                    break;
                }
            }
        }

        // Si llegó al objetivo, agregar el punto final
        GridPoint currentGrid = agentGrid.getLocation(agent);
        if (currentGrid.equals(targetGrid)) {
            pathTaken.add(targetGrid);
            System.out.println("DEBUG: Agente movido a: (" + targetGrid.getX() + ", " + targetGrid.getY() + ")");
        }

        // Imprimir el camino recorrido
        System.out.println("DEBUG: Camino recorrido: " + pathTaken);

        // Obtener posición final del agente
        GridPoint finalGrid = agentGrid.getLocation(agent);
        Coordinate finalGeo = mapGridToGeo(finalGrid.getX(), finalGrid.getY());
        System.out.printf("Punto final: Grilla: (%d, %d) -> (%.8f, %.8f)\n", 
            finalGrid.getX(), finalGrid.getY(), finalGeo.x, finalGeo.y);

        // Calcular distancia recorrida en metros
        double distanceMetersECEF = distanceInMeters(startGeo, finalGeo);
        double distanceMetersHaversine = haversineDistance(startGeo, finalGeo);
        System.out.printf("Distancia recorrida (ECEF): %.2f metros\n", distanceMetersECEF);
        System.out.printf("Distancia recorrida (Haversine): %.2f metros\n", distanceMetersHaversine);

        // Instrucciones para validar en Google Maps
        System.out.println("Instrucciones para validar en Google Maps:");
        System.out.printf("1. Abre Google Maps y busca el punto inicial: (%.8f, %.8f)\n", startGeo.y, startGeo.x);
        System.out.printf("2. Busca el punto final: (%.8f, %.8f)\n", finalGeo.y, finalGeo.x);
        System.out.printf("3. Usa la herramienta de medición de distancia (línea recta) y compara con %.2f metros (Haversine).", 
            distanceMetersHaversine);

        // Depuración: Estado de celdas cercanas al punto final
        System.out.println("Depuración: Estado de celdas cercanas al punto final:");
        for (int x = Math.max(0, finalGrid.getX() - 2); x <= Math.min(GRID_WIDTH - 1, finalGrid.getX() + 2); x++) {
            for (int y = Math.max(0, finalGrid.getY() - 2); y <= Math.min(GRID_HEIGHT - 1, finalGrid.getY() + 2); y++) {
                MapCell cell = mapCellGrid.getObjectAt(x, y);
                String cellType = cell != null ? String.valueOf(cell.getType()) : "null";
                String traversable = cell != null && cell.isTraversable() ? "transitable" : "no transitable";
                System.out.printf("Celda (%d, %d): tipo=%s, %s\n", x, y, cellType, traversable);
            }
        }
    }
}