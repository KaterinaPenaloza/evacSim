package evacSim.agents;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;

import org.locationtech.jts.geom.Coordinate;
import org.locationtech.jts.geom.Geometry;
import org.locationtech.jts.geom.GeometryFactory;
import org.locationtech.jts.geom.Point;

import repast.simphony.context.Context;
import repast.simphony.engine.environment.RunEnvironment;
import repast.simphony.engine.schedule.ScheduleParameters;
import repast.simphony.engine.schedule.ScheduledMethod;
import repast.simphony.essentials.RepastEssentials;
import repast.simphony.space.gis.Geography;
import repast.simphony.space.grid.Grid;
import repast.simphony.space.grid.GridPoint;
import repast.simphony.util.ContextUtils;

public class GisAgent {

	/********** Atributos ***********/
    // Atributos del agente (faltan mas)
    private String name;
    private ZoneAgent targetSafeZone;
    
    // Velocidad del agente celdas/segundo
    private double speed;					// velocidad fija de momento (1,4 m/s)
    private double distanceAccumulated = 0; // acumular fracciones de celdas
    
    // Variables de tiempo
    private int evacuationStartTick = -1;
    private int evacuationEndTick = -1;
    private boolean evacuationStarted = false;
    private double secondsPerTick;
    
    // Geografia del agente
    public Geography<GisAgent> geography;
    private Grid<MapCell> mapCellGrid;
    private Grid<GisAgent> agentGrid;		// Movimiento y posición de los agentes
    private GeometryFactory geometryFactory = new GeometryFactory();
    private Random random = new Random();

    // Pathfinding
    public GridPoint safeZoneGridTarget;	// Objetivo de la zona segura
    private List<GridPoint> currentPath;	// Camino calculado por A*
    private int pathIndex;

    // Estados de evacuación
    private boolean isEvacuated = false;
    private boolean inSafeZone = false;
    private List<GridPoint> safeZonePoints = new ArrayList<>();

    // Control de recálculos (arreglar)
    private int recalculationCount = 0;
    private int waitTicks = 0;
    private static final int MAX_RECALCULATIONS = 2;

    
    public GisAgent(String name, ZoneAgent targetSafeZone, Grid<MapCell> mapCellGrid, Grid<GisAgent> agentGrid) {
        this.name = name;
        this.targetSafeZone = targetSafeZone;
        this.mapCellGrid = mapCellGrid;
        this.agentGrid = agentGrid;
        this.secondsPerTick = (Double) RunEnvironment.getInstance().getParameters().getValue("secondsPerTick");
        this.speed = 1.4; // Velocidad promedio de caminata
        this.setCurrentPath(new ArrayList<>());
        this.setPathIndex(0);
    }

    
    /* Step method */
    @ScheduledMethod(start = 1, interval = 1, priority = ScheduleParameters.FIRST_PRIORITY)
    public void step() {
        if (isEvacuated) {
			return; // (deberia terminar la simulación(?)
		}
            
        // Inicio de evacuación
        if (!evacuationStarted) {
            evacuationStartTick = (int) RepastEssentials.GetTickCount();
            evacuationStarted = true;
        }
        
        // Iniciar geografia
        if (geography == null) {
            Context context = ContextUtils.getContext(this);
            geography = (Geography)context.getProjection("Geography");
        }

        GridPoint currentAgentGridPoint = agentGrid.getLocation(this);
        if (currentAgentGridPoint == null) {
            System.err.println("DEBUG (GisAgent " + name + "): No se pudo obtener la posición del agente en la grilla de agentes.");
            return;
        }

        if (safeZonePoints.isEmpty()) {
            findSafeZonePoints();
        }

        /********** Zona segura **********/
        // LLegar a la zona segura
        if (!inSafeZone && isInSafeZone(currentAgentGridPoint)) {
            inSafeZone = true;
            int currentTick = (int) RepastEssentials.GetTickCount();
            
            // Cambiar objetivo a punto de dispersión dentro de zona segura (revisar)
            GridPoint dispersionPoint = selectDispersionPoint();
            if (dispersionPoint != null && !dispersionPoint.equals(currentAgentGridPoint)) {
                safeZoneGridTarget = dispersionPoint;
                getCurrentPath().clear(); // recalculo de path
                setPathIndex(0);
                System.out.println("DEBUG (GisAgent " + name + "): Llegó a zona segura, dispersándose a: " + safeZoneGridTarget);
            } else {
                // Si ya está en una buena posición o no hay mejor lugar, terminar evacuacion
                isEvacuated = true;
                safeZoneGridTarget = null;
                getCurrentPath().clear();
                setPathIndex(0);
             
                evacuationEndTick = currentTick;
                int evacuationTimeTicks = evacuationEndTick - evacuationStartTick;
                double evacuationTimeSeconds = evacuationTimeTicks * secondsPerTick;
                System.out.println("DEBUG (GisAgent " + name + "): Evacuación completada inmediatamente - Tiempo: " + evacuationTimeTicks + " ticks (" + String.format("%.2f", evacuationTimeSeconds) + " segundos)");
                return;
            }
        }

        // Determinar punto objetivo en zona segura (entonces para que se dispersan?)
        if (!inSafeZone && safeZoneGridTarget == null) {
            try {
            	safeZoneGridTarget = selectRandomSafePoint();
                if (safeZoneGridTarget != null) {
                    System.out.println("DEBUG (GisAgent " + name + "): Objetivo de zona segura en grilla establecido: (" + safeZoneGridTarget.getX() + ", " + safeZoneGridTarget.getY() + ")");
                } else {
                    System.err.println("DEBUG (GisAgent " + name + "): No se pudo encontrar punto seguro accesible.");
                    return;
                }
            } catch (Exception e) {
                System.err.println("Error al intentar obtener objetivo de zona segura: " + e.getMessage());
                return;
            }
        }

        if (safeZoneGridTarget != null) {
            moveAlongCalculatedPath();
            // Verificar si completó la evacuación (llegó al punto de dispersión)
            if (inSafeZone && currentAgentGridPoint.equals(safeZoneGridTarget)) {
                isEvacuated = true;
                safeZoneGridTarget = null;
                getCurrentPath().clear();
                setPathIndex(0);
                evacuationEndTick = (int) RepastEssentials.GetTickCount();
                int evacuationTimeTicks = evacuationEndTick - evacuationStartTick;
                double evacuationTimeSeconds = evacuationTimeTicks * secondsPerTick;
                System.out.println("DEBUG (GisAgent " + name + "): Evacuación completada - llegó al punto de dispersión - Tiempo: " + 
                    evacuationTimeTicks + " ticks (" + String.format("%.2f", evacuationTimeSeconds) + " segundos)");
            }
        }
    }

    /********** Pathfinding **********/ 
    /* (optimizar y arreglar xc)
     * cambiar a cola de prioridad en vez de lista y cambiar la busqueda de nodos
     * implementar el sfm en vez de desviarse y wait
     * precomputar zonas seguras
     */
    private void moveAlongCalculatedPath() {
        GridPoint currentAgentGridPoint = agentGrid.getLocation(this);

        if (currentAgentGridPoint == null) {
            System.err.println("DEBUG (GisAgent " + name + "): No se pudo obtener la posición del agente en la grilla de agentes.");
            return;
        }

        // Verificar si debe esperar antes de recalcular
        if (waitTicks > 0) {
            waitTicks--;
            return;
        }

        // Si ya llegamos al objetivo, no recalcular
        if (currentAgentGridPoint.equals(safeZoneGridTarget)) {
            return; // Ya llegó
        }

        // Si el camino actual está terminado/vacío, o si nos desviamos, recalcular
        if (getCurrentPath().isEmpty() ||
            getPathIndex() >= getCurrentPath().size() ||
            (getPathIndex() < getCurrentPath().size() && !getCurrentPath().get(getPathIndex()).equals(currentAgentGridPoint))) {
            calculatePathToSafeZone(currentAgentGridPoint, safeZoneGridTarget);
        }

        // No hay ruta posible
        if (getCurrentPath().isEmpty()) {
            System.err.println("DEBUG (GisAgent " + name + "): No se pudo encontrar un camino transitable.");
            recalculationCount++;
            if (recalculationCount > MAX_RECALCULATIONS) {
                waitTicks = 3 + random.nextInt(10); // Esperar 3-12 ticks
                recalculationCount = 0;
                System.out.println("DEBUG (GisAgent " + name + "): Demasiados recálculos fallidos. Esperando " + waitTicks + " ticks.");
            }
            return;
        }

        GridPoint nextGridPoint;

        // Lógica de seguimiento de path
        if (currentAgentGridPoint.equals(getCurrentPath().get(getCurrentPath().size() - 1))) {
            nextGridPoint = currentAgentGridPoint;
        } else if (getPathIndex() + 1 < getCurrentPath().size()) {
            if (currentAgentGridPoint.equals(getCurrentPath().get(getPathIndex()))) {
                setPathIndex(getPathIndex() + 1);
                nextGridPoint = getCurrentPath().get(getPathIndex());
            } else {
                setPathIndex(0);
                nextGridPoint = getCurrentPath().get(getPathIndex());
                System.out.println("DEBUG (GisAgent " + name + "): Agente desviado o camino recalculado. Reiniciando pathIndex y moviendo al primer paso: " + nextGridPoint);
            }
        } else {
            System.err.println("DEBUG (GisAgent " + name + "): Error de lógica en pathIndex (fuera de límites). Recalculando camino.");
            calculatePathToSafeZone(currentAgentGridPoint, safeZoneGridTarget);
            if (!getCurrentPath().isEmpty()) {
                setPathIndex(0);
                nextGridPoint = getCurrentPath().get(getPathIndex());
            } else {
                return;
            }
        }

        // Calcular distancia recorrida en el tick
        double distancePerTick = speed * secondsPerTick;
        distanceAccumulated += distancePerTick;
        // Verificar si hay suficiente distancia acumulada para moverse a la siguiente celda
        // se acumula hasta 5 metros (tamaño de la celda) antes de avanzar, equivalente a unos 3 o 4 segundos por 1,4 m/s
        if (distanceAccumulated >= ContextCreator.CELL_SIZE_METERS && isValidStep(nextGridPoint)) {
            agentGrid.moveTo(this, nextGridPoint.getX(), nextGridPoint.getY());
            Coordinate newGeoCoord = ContextCreator.mapGridToGeo(nextGridPoint.getX(), nextGridPoint.getY());
            geography.move(this, geometryFactory.createPoint(newGeoCoord));
            distanceAccumulated -= ContextCreator.CELL_SIZE_METERS;
            recalculationCount = 0;
            System.out.println("DEBUG (GisAgent " + name + "): Distancia acumulada restante: " + String.format("%.2f", distanceAccumulated) + " m");
        } else if (distanceAccumulated < ContextCreator.CELL_SIZE_METERS) {
            System.out.println("DEBUG (GisAgent " + name + "): Acumulado " + String.format("%.2f", distanceAccumulated));
        } else {
            System.err.println("DEBUG (GisAgent " + name + "): Siguiente paso (" + nextGridPoint + ") no transitable o muy congestionado. Recalculando camino.");
            recalculationCount++;
            if (recalculationCount > MAX_RECALCULATIONS) {
                waitTicks = 5 + random.nextInt(10);
                recalculationCount = 0;
                System.out.println("DEBUG (GisAgent " + name + "): Demasiada congestión. Esperando " + waitTicks + " ticks.");
                return;
            }
            calculatePathToSafeZone(currentAgentGridPoint, safeZoneGridTarget);
        }
	}

    
    // Puntos del camino
    public GridPoint getNextPathPoint() {
        if (currentPath.isEmpty() || pathIndex >= currentPath.size()) {
            return null;
        }
        return currentPath.get(pathIndex);
    }

    public void advancePathIndex() {
        if (pathIndex < currentPath.size()) {
            pathIndex++;
        }
    }
    
    // Calculo del camino usando A*
    public void calculatePathToSafeZone(GridPoint start, GridPoint target) {
        getCurrentPath().clear();
        setPathIndex(0);

        if (start == null || target == null) {
            System.err.println("Error: Start o Target nulo.");
            return;
        }
        if (start.equals(target)) {
            getCurrentPath().add(start);
            System.out.println("DEBUG (GisAgent " + name + "): Punto de inicio = punto destino. No se calcula camino.");
            return;
        }

        Map<GridPoint, GridPoint> cameFrom = new HashMap<>();
        Map<GridPoint, Double> gScore = new HashMap<>();
        Map<GridPoint, Double> fScore = new HashMap<>();

        List<GridPoint> openList = new ArrayList<>();

        gScore.put(start, 0.0);
        fScore.put(start, calculateHeuristic(start, target));
        openList.add(start);

        GridPoint closestToTargetSoFar = start;
        double minDistanceFromTarget = calculateHeuristic(start, target);

        while (!openList.isEmpty()) {
            Collections.sort(openList, Comparator.comparingDouble(fScore::get));
            GridPoint current = openList.remove(0);

            double distCurrentToTarget = calculateHeuristic(current, target);
            if (distCurrentToTarget < minDistanceFromTarget) {
                minDistanceFromTarget = distCurrentToTarget;
                closestToTargetSoFar = current;
            }

            if (current.equals(target)) {
                setCurrentPath(reconstructPath(cameFrom, current));
                return;
            }

            // Vecinos de la celda actual
            for (GridPoint neighbor : getNeighbors(current)) {
                // Obtener las dimensiones de la grilla desde mapCellGrid
                int gridWidth = mapCellGrid.getDimensions().getWidth();
                int gridHeight = mapCellGrid.getDimensions().getHeight();

                // Asegurarse de que el vecino esté dentro de los límites de la grilla
                if (neighbor.getX() < 0 || neighbor.getX() >= gridWidth ||
                    neighbor.getY() < 0 || neighbor.getY() >= gridHeight) {
                    continue;
                }

                // Usar mapCellGrid para consultar la transitabilidad del vecino
                MapCell neighborCell = mapCellGrid.getObjectAt(neighbor.getX(), neighbor.getY());

                // Solo consideramos vecinos transitables con control básico de aglomeración //revisar
                if (neighborCell != null && neighborCell.isTraversable() && countAgentsAt(neighbor) < 4) {
                    double tentativeGScore = gScore.getOrDefault(current, Double.POSITIVE_INFINITY) + 1;

                    if (tentativeGScore < gScore.getOrDefault(neighbor, Double.POSITIVE_INFINITY)) {
                        cameFrom.put(neighbor, current);
                        gScore.put(neighbor, tentativeGScore);
                        fScore.put(neighbor, tentativeGScore + calculateHeuristic(neighbor, target));

                        if (!openList.contains(neighbor)) {
                            openList.add(neighbor);
                        }
                    }
                }
            }
        }

        // Si no se encontró un camino directo al target
        if (!closestToTargetSoFar.equals(start)) {
            setCurrentPath(reconstructPath(cameFrom, closestToTargetSoFar));
            System.err.println("DEBUG (GisAgent " + this.name + "): No se pudo encontrar un camino directo al objetivo. Se encontró el camino al punto transitable más cercano: " + closestToTargetSoFar);
        } else {
            System.err.println("DEBUG (GisAgent " + this.name + "): No se pudo encontrar un camino a la zona segura para el agente.");
        }
    }

    // Calcula la distancia euclidiana entre dos puntos
    private double calculateHeuristic(GridPoint p1, GridPoint p2) {
        return Math.sqrt(
            Math.pow(p1.getX() - p2.getX(), 2) +
            Math.pow(p1.getY() - p2.getY(), 2)
        );
    }

    private List<GridPoint> reconstructPath(Map<GridPoint, GridPoint> cameFrom, GridPoint current) {
        List<GridPoint> totalPath = new ArrayList<>();
        totalPath.add(current);
        while (cameFrom.containsKey(current)) {
            current = cameFrom.get(current);
            totalPath.add(current);
        }
        Collections.reverse(totalPath);
        return totalPath;
    }

    // Obtiene los 8 vecinos
    private List<GridPoint> getNeighbors(GridPoint p) {
        List<GridPoint> neighbors = new ArrayList<>();
        for (int dx = -1; dx <= 1; dx++) {
            for (int dy = -1; dy <= 1; dy++) {
                if (dx == 0 && dy == 0) {
                    continue;
                }
                int nx = p.getX() + dx;
                int ny = p.getY() + dy;

                neighbors.add(new GridPoint(nx, ny));
            }
        }
        return neighbors;
    }

    // encontrar puntos de zona segura
    private void findSafeZonePoints() {
        Geometry safeZoneGeom = geography.getGeometry(targetSafeZone);
        if (safeZoneGeom == null) {
			return;
		}

        int width = mapCellGrid.getDimensions().getWidth();
        int height = mapCellGrid.getDimensions().getHeight();
        GeometryFactory factory = new GeometryFactory();

        for (int x = 0; x < width; x++) {
            for (int y = 0; y < height; y++) {
                MapCell cell = mapCellGrid.getObjectAt(x, y);
                if (cell != null && cell.isTraversable()) {
                    Coordinate coord = ContextCreator.mapGridToGeo(x, y);
                    Point point = factory.createPoint(coord);

                    if (safeZoneGeom.contains(point)) {
                        safeZonePoints.add(new GridPoint(x, y));
                    }
                }
            }
        }
    }
    private GridPoint findClosestSafePoint(GridPoint from) {
        return safeZonePoints.stream()
            .min(Comparator.comparingDouble(p -> calculateHeuristic(from, p)))
            .orElse(null);
    }

    // Punto de dispersion //ver
    private GridPoint selectDispersionPoint() {
        GridPoint currentPos = agentGrid.getLocation(this);
        GridPoint centroid = calculateCentroid();

        // Calcular distancia al centroide
        double distanceToCentroid = calculateHeuristic(currentPos, centroid);

        // Solo evacuar inmediatamente si está cerca del centro Y con poca congestión // no se dispersa
        if (distanceToCentroid < 8 && countAgentsAt(currentPos) <= 1) {
            return null; // No necesita moverse
        }

        // Buscar punto disponible cerca del centroide
        for (int attempts = 0; attempts < 30; attempts++) {
            int x = centroid.getX() + random.nextInt(15) - 7;
            int y = centroid.getY() + random.nextInt(15) - 7;
            GridPoint candidate = new GridPoint(x, y);

            if (isValidPosition(candidate) && !candidate.equals(currentPos)) {
                return candidate;
            }
        }

        // Fallback: cualquier punto disponible que no sea su posición actual
        GridPoint fallback = safeZonePoints.stream()
            .filter(this::isValidPosition)
            .filter(p -> !p.equals(currentPos))
            .findFirst()
            .orElse(null);

        return fallback;
    }

    
    private GridPoint calculateCentroid() {
        if (safeZonePoints.isEmpty()) {
			return new GridPoint(0, 0);
		}
        int sumX = safeZonePoints.stream().mapToInt(GridPoint::getX).sum();
        int sumY = safeZonePoints.stream().mapToInt(GridPoint::getY).sum();
        return new GridPoint(sumX / safeZonePoints.size(), sumY / safeZonePoints.size());
    }

    
    private boolean isInSafeZone(GridPoint point) {
        MapCell cell = mapCellGrid.getObjectAt(point.getX(), point.getY());
        if (cell == null) {
			return false;
		}

        // Verificar por tipo de celda
        if (cell.getType() == MapCell.TYPE_SAFE_ZONE) {
            return true;
        }

        // Verificar por geometría si es necesario
        try {
            Geometry safeZoneGeom = geography.getGeometry(targetSafeZone);
            if (safeZoneGeom != null) {
                Coordinate coord = ContextCreator.mapGridToGeo(point.getX(), point.getY());
                Point geoPoint = geometryFactory.createPoint(coord);
                return safeZoneGeom.contains(geoPoint);
            }
        } catch (Exception e) {
            // Ignore
        }

        return false;
    }

    private boolean isValidPosition(GridPoint point) {
        return isWithinBounds(point) &&
               isTraversable(point.getX(), point.getY()) &&
               countAgentsAt(point) <= 1;
    }

    public boolean isValidStep(GridPoint point) {
        if (!isWithinBounds(point) || !isTraversable(point.getX(), point.getY())) {
            return false;
        }

        // Permitir hasta 2 agentes por celda para evitar bloqueos
        return countAgentsAt(point) < 2;
    }

    private boolean isWithinBounds(GridPoint point) {
        int width = mapCellGrid.getDimensions().getWidth();
        int height = mapCellGrid.getDimensions().getHeight();
        return point.getX() >= 0 && point.getX() < width &&
               point.getY() >= 0 && point.getY() < height;
    }

    private boolean isTraversable(int x, int y) {
        MapCell cell = mapCellGrid.getObjectAt(x, y);
        return cell != null && cell.isTraversable();
    }

    private int countAgentsAt(GridPoint point) {
        int count = 0;
        for (GisAgent agent : agentGrid.getObjectsAt(point.getX(), point.getY())) {
            count++;
        }
        return count;
    }

    private GridPoint selectRandomSafePoint() {
        if (safeZonePoints.isEmpty()) {
            return null;
        }
        return safeZonePoints.get(random.nextInt(safeZonePoints.size()));
    }

    // Getters
    public String getName() {
        return name;
    }

    public boolean isEvacuated() {
        return isEvacuated;
    }

    public boolean hasReachedSafeZone() {
        return inSafeZone;
    }

    @Override
    public String toString() {
        return name;
    }

	public int getPathIndex() {
		return pathIndex;
	}

	public void setPathIndex(int pathIndex) {
		this.pathIndex = pathIndex;
	}

	List<GridPoint> getCurrentPath() {
		return currentPath;
	}

	public void setCurrentPath(List<GridPoint> currentPath) {
		this.currentPath = currentPath;
	}
}