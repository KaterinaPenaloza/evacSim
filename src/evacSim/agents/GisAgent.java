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
    private String name;
    private ZoneAgent targetSafeZone;
    private double speed;  // m/s
    
    private int cellsMoved = 0;
    private double distanceAccumulated = 0;

    private boolean evacuationStarted = false;
    private double secondsPerTick = 0.5;
    
    // Tiempos
    private int totalStartTick = -1;
    private int movementStartTick = -1;
    private int evacuationEndTick = -1;

    // Métricas derivadas
    private double preMovementTime = 0.0;
    private double movementTime = 0.0;
    private double evacuationTime = 0.0;
    
    private int preEvacuationTicks = 0;  // Tiempo aleatorio antes de empezar a moverse
    private boolean preEvacuationComplete = false;
    private boolean isInitialized = false;
   
    public Geography<GisAgent> geography;
    private Map<ContextCreator.GridPoint, Integer> mapCellGrid;
    private Grid<GisAgent> agentGrid;
    private GeometryFactory geometryFactory = new GeometryFactory();
    public ContextCreator.GridPoint safeZoneGridTarget;
    private List<ContextCreator.GridPoint> currentPath;
    private int pathIndex;
    
    private boolean isEvacuated = false;
    private boolean hasReachedFinalTarget = false;
    private int recalculationCount = 0;
    private int waitTicks = 0;
    private static final int MAX_RECALCULATIONS = 2;
    // Sistema de espera antes de recalcular ruta
    private int waitingAttempts = 0;
    private static final int MAX_WAITING_ATTEMPTS = 5; // Intentos antes de recalcular
    private ContextCreator.GridPoint blockedPoint = null;
    
    private Random random = new Random();
    
    
    public GisAgent(String name, ZoneAgent targetSafeZone, Map<ContextCreator.GridPoint, Integer> mapCellGrid,
            Grid<GisAgent> agentGrid, int preEvacMinSeconds, int preEvacMaxSeconds) {
        this.name = name;
        this.targetSafeZone = targetSafeZone;
        this.mapCellGrid = mapCellGrid;
        this.agentGrid = agentGrid;
        this.currentPath = new ArrayList<>();
        this.pathIndex = 0;
        this.speed = ContextCreator.getSpeedByDistribution(random);
        // Tiempo de pre-evacuación aleatorio
        // Convertir segundos a ticks (2 ticks = 1 segundo)
        int minTicks = preEvacMinSeconds * 2;
        int maxTicks = preEvacMaxSeconds * 2;
        
        this.preEvacuationTicks = minTicks + random.nextInt(maxTicks - minTicks + 1);
    }

	// ========================================
	// |         step() método principal      |
	// ========================================
    @ScheduledMethod(start = 1, interval = 1, priority = ScheduleParameters.FIRST_PRIORITY)
    public void step() {
    	
    	// 0) Inicializar (geography y target)
    	if (!isInitialized) {
            if (!performInitialization()) {
                return;
            }
        }
    	
    	// 1) Pre movimiento
    	if (!handlePreEvacuation()) {
            return;
        }
    	
    	// 2) Iniciar cronometro de evacuación
        initializeEvacuationTimer();
        
        // 3) Obtener posicion del agente
        ContextCreator.GridPoint currentPoint = getCurrentGridPoint();
        if (currentPoint == null) {
            System.err.println("DEBUG (GisAgent " + name + "): No se pudo obtener la posición del agente.");
            return;
        }

        // 4) Verificar si llego a la zona segura (marcar como evacuado)
        if (!isEvacuated && isInSafeZone(currentPoint)) {
            isEvacuated = true;
            ContextCreator.incrementEvacuated();
            evacuationEndTick = (int) RepastEssentials.GetTickCount();
            
            // Cálculos de tiempo
            int preTicks = movementStartTick - totalStartTick;
            int moveTicks = evacuationEndTick - movementStartTick;
            int totalTicks = evacuationEndTick - totalStartTick;

            preMovementTime = preTicks * secondsPerTick;
            movementTime = moveTicks * secondsPerTick;
            evacuationTime = totalTicks * secondsPerTick;

            double distanceMovedMeters = cellsMoved * ContextCreator.CELL_SIZE_METERS;

            System.out.printf(
                "DEBUG (GisAgent %s): EVACUADO | Pre-mov: %.2f s | Mov: %.2f s | Total: %.2f s | Dist: %.1f m%n",
                name, preMovementTime, movementTime, evacuationTime, distanceMovedMeters
            );
        }
        
        
        // 5) Verificar si llegó al target final
        if (hasReachedTarget(currentPoint)) {
            if (!hasReachedFinalTarget) {
                hasReachedFinalTarget = true;
                ContextCreator.incrementReachedTarget();
                if (isEvacuated && evacuationTime > 0) {
                    EvacuationData.recordAgentTimes(name, preMovementTime, movementTime, evacuationTime);
                    System.out.printf(
                        "DEBUG (GisAgent %s): DATOS REGISTRADOS | Pre: %.2f s | Mov: %.2f s | Total: %.2f s%n",
                        name, preMovementTime, movementTime, evacuationTime
                    );
                }
            }
            return; // Dejar de moverse
        }
        
        if (hasReachedTarget(currentPoint)) {
            return;
        }

        // 6) Verificar tiempo de espera
        if (handleWaitTicks()) {
            return;
        }

        // 7) Acumular distancia
        accumulateDistance();
        
        if (!isEnoughDistance()) {
            return;
        }

        // 8) Continuar moviéndose hacia el target
        moveAlongCalculatedPath(currentPoint);
    }

    /**** 0) Inicializacion del agente ****/
    private boolean performInitialization() {
        System.out.println("DEBUG (GisAgent " + name + "): Inicialización de agente");
        
        // Obtener contexto y geografía
        if (!initializeGeography()) return false;
        if (!assignSafeZoneTarget()) return false;
        
        // Tiempo
        if (totalStartTick < 0) {
            totalStartTick = (int) RepastEssentials.GetTickCount();
        }
        
        // Marcar como inicializado
        isInitialized = true;
        return true;
    }
    
    /**** 0.1) Inicializar agentes en la grilla ****/
    private boolean initializeGeography() {
        if (geography == null) {
            Context context = ContextUtils.getContext(this);
            geography = (Geography) context.getProjection("Geography");
        }
        return geography != null;
    }
    
    /**** 0.2) Asignar target ****/
    private boolean assignSafeZoneTarget() {
        if (safeZoneGridTarget == null) {
        	safeZoneGridTarget = ContextCreator.getSafeZoneTarget();
        	
        	if (safeZoneGridTarget == null) {
                System.err.println("DEBUG (GisAgent " + name + "): No se pudo obtener un punto target válido.");
                return false;
            }
            
        	System.out.println("DEBUG (GisAgent " + name + "): Objetivo asignado a punto: (" + 
                    safeZoneGridTarget.getX() + ", " + safeZoneGridTarget.getY() + ")");
        }
        return true;
    }
    
    /**** 1) Premovimiento ****/
    private boolean handlePreEvacuation() {
        if (!preEvacuationComplete) {
            if (preEvacuationTicks > 0) {
                preEvacuationTicks--;
                return false; // Aun no empieza a evacuar
            } else {
                preEvacuationComplete = true;
                System.out.println("DEBUG (GisAgent " + name + "): Iniciando evacuación.");
            }
        }
        return true;
    }
    
    /**** 2) Inicializar tiempo de evacuación ****/
    private void initializeEvacuationTimer() {
        if (!evacuationStarted) {
        	movementStartTick = (int) RepastEssentials.GetTickCount(); // Inicio de movimiento
            evacuationStarted = true;
        }
    }
    

    /**** 5) Si llegó al target ****/
    private boolean hasReachedTarget(ContextCreator.GridPoint currentPosition) {
        double distanceToTarget = calculateHeuristic(currentPosition, safeZoneGridTarget);
        return distanceToTarget <= 1.0;
    }
    
    /**** 6) WaitTicks ****/
    private boolean handleWaitTicks() {
        if (waitTicks > 0) {
            waitTicks--;
            return true;
        }
        return false;
    }
    
    /**** 7) Acumular distancia ****/
    private void accumulateDistance() {
        double distancePerTick = speed * secondsPerTick;
        distanceAccumulated += distancePerTick;
    }
    
    /**** Verificar si acumuló suficiente distancia para moverse ****/
    private boolean isEnoughDistance() {
        return distanceAccumulated >= ContextCreator.CELL_SIZE_METERS;
    }
   
    /**** 8) Moverse por el camino ****/
    private void moveAlongCalculatedPath(ContextCreator.GridPoint currentPoint) {
        
        //********* moverse al objetivo ********/
        // Validar si necesita recalcular el camino
        if (needsPathRecalculation(currentPoint)) {
            calculatePathToSafeZone(currentPoint, safeZoneGridTarget);
            waitingAttempts = 0; // Resetear intentos al calcular nuevo camino
            blockedPoint = null;
        }
        
        // Si no hay camino disponible, manejar el error
        if (currentPath.isEmpty()) {
            handleNoPathAvailable();
            return;
        }
        
        // Obtener el siguiente punto
        ContextCreator.GridPoint nextPoint = getNextPointInPath(currentPoint);
        if (nextPoint == null) {
            return;
        }
        
        // Intentar mover si acumuló suficiente distancia
        if (isEnoughDistance()) {
            if (isValidStep(nextPoint)) {
                // Camino libre, moverse
                moveAgent(nextPoint);
                waitingAttempts = 0; // Resetear intentos
                blockedPoint = null;
            } else {
                // Camino bloqueado, esperar antes de recalcular
                handleBlockedPath(currentPoint, nextPoint);
            }
        }
    }

    /**** 8.1) ****/
    private boolean needsPathRecalculation(ContextCreator.GridPoint currentPosition) {
        return currentPath.isEmpty() 
            || pathIndex >= currentPath.size()
            || (pathIndex < currentPath.size() && !currentPath.get(pathIndex).equals(currentPosition));
    }
    
    /**** 8.2) ****/
    private void handleNoPathAvailable() {
        System.err.println("DEBUG (GisAgent " + name + "): No se pudo encontrar un camino transitable.");
        recalculationCount++;
        if (recalculationCount > MAX_RECALCULATIONS) {
            waitTicks = 3 + random.nextInt(10);
            recalculationCount = 0;
            System.out.println("DEBUG (GisAgent " + name + "): Demasiados recálculos fallidos. Esperando "
                    + waitTicks + " ticks.");
        }
    }
    
    /**** 8.3) ****/
    private ContextCreator.GridPoint getNextPointInPath(ContextCreator.GridPoint currentPosition) {
        // Si ya llegó al destino
        if (currentPosition.equals(currentPath.get(currentPath.size() - 1))) {
            return currentPosition;
        }
        
        // Si hay más puntos en el camino
        if (pathIndex + 1 < currentPath.size()) {
            if (currentPosition.equals(currentPath.get(pathIndex))) {
                pathIndex++;
                return currentPath.get(pathIndex);
            } else {
                pathIndex = 0;
                return currentPath.get(pathIndex);
            }
        }
        
        // Necesita recalcular
        calculatePathToSafeZone(currentPosition, safeZoneGridTarget);
        if (!currentPath.isEmpty()) {
            pathIndex = 0;
            return currentPath.get(pathIndex);
        }
        
        return null;
    }
    
    /**** 8.4) ****/
    private void handleBlockedPath(ContextCreator.GridPoint currentPosition, ContextCreator.GridPoint blockedNext) {
        // Si es el mismo punto bloqueado, incrementar contador
        if (blockedPoint != null && blockedPoint.equals(blockedNext)) {
            waitingAttempts++;
        } else {
            // Nuevo obstáculo, resetear contador
            blockedPoint = blockedNext;
            waitingAttempts = 1;
        }
        
        // Si aún no ha esperado suficiente, solo esperar
        if (waitingAttempts < MAX_WAITING_ATTEMPTS) {
            // Consumir parte de la distancia acumulada para simular intento de movimiento
            distanceAccumulated = Math.max(0, distanceAccumulated - (ContextCreator.CELL_SIZE_METERS * 0.5));
            return;
        }
        
        // Ya esperó suficiente, intentar recalcular
        System.out.println("DEBUG (GisAgent " + name + "): Esperó " + waitingAttempts + " intentos. Buscando camino alternativo.");
        recalculationCount++;
        
        if (recalculationCount > MAX_RECALCULATIONS) {
            // Demasiadas recalculaciones, esperar más tiempo (detención completa)
            waitTicks = 8 + random.nextInt(15);
            recalculationCount = 0;
            waitingAttempts = 0;
            blockedPoint = null;
            System.out.println("DEBUG (GisAgent " + name + "): Demasiada congestión. Esperando " + waitTicks + " ticks.");
            return;
        }
        
        // Recalcular camino
        calculatePathToSafeZone(currentPosition, safeZoneGridTarget);
        waitingAttempts = 0;
        blockedPoint = null;
    }
    
    
    // ========================================
   	// |             PATHFINDING              |
   	// ========================================
    public void calculatePathToSafeZone(ContextCreator.GridPoint start, ContextCreator.GridPoint target) {
    	// Reiniciar el camino
        currentPath.clear();
        pathIndex = 0;

        // Validar entradas
        if (!validatePathfindingInputs(start, target)) {
            return;
        }

        // Inicializar estructuras
        Map<ContextCreator.GridPoint, ContextCreator.GridPoint> cameFrom = new HashMap<>();
        Map<ContextCreator.GridPoint, Double> gScore = new HashMap<>(); // Costo desde el inicio
        Map<ContextCreator.GridPoint, Double> fScore = new HashMap<>(); // gScore + heurística
        List<ContextCreator.GridPoint> openList = new ArrayList<>(); // Nodos a explorar

        // Inicializar punto de inicio
        gScore.put(start, 0.0);
        fScore.put(start, calculateHeuristic(start, target));
        openList.add(start);

        // Encontrar un camino parcial si no se puede llegar la destino //***Revisar esto***
        ContextCreator.GridPoint closestToTargetSoFar = start;
        double minDistanceFromTarget = calculateHeuristic(start, target);

        // Ejecutar el algoritmo para encontrar el camino
        ContextCreator.GridPoint finalPoint = executePathFinding(target, cameFrom, gScore, fScore, openList, 
                closestToTargetSoFar, minDistanceFromTarget);
        

        if (!finalPoint.equals(start)) {
            currentPath = reconstructPath(cameFrom, finalPoint);
        }
    }

    private ContextCreator.GridPoint executePathFinding(
            ContextCreator.GridPoint target,
            Map<ContextCreator.GridPoint, ContextCreator.GridPoint> cameFrom,
            Map<ContextCreator.GridPoint, Double> gScore,
            Map<ContextCreator.GridPoint, Double> fScore,
            List<ContextCreator.GridPoint> openList,
            ContextCreator.GridPoint closestToTargetSoFar,
            double minDistanceFromTarget) {
        
        ContextCreator.GridPoint bestPoint = closestToTargetSoFar;
        double bestDistance = minDistanceFromTarget;
        
        while (!openList.isEmpty()) {
            // Ordenar por fScore (punto con menor costo primero)
            Collections.sort(openList, Comparator.comparingDouble(fScore::get));
            ContextCreator.GridPoint current = openList.remove(0);

            // Actualizar el punto más cercano al destino
            double distCurrentToTarget = calculateHeuristic(current, target);
            if (distCurrentToTarget < bestDistance) {
                bestDistance = distCurrentToTarget;
                bestPoint = current;
            }

            // Si llegamos al destino, terminar
            if (current.equals(target)) {
                return current;
            }

            // Explorar vecinos del punto actual
            exploreNeighbors(current, target, cameFrom, gScore, fScore, openList);
        }
        
        return bestPoint;
    }
    
    /*************** Valida las entradas del pathfinding ***************/
    private boolean validatePathfindingInputs(ContextCreator.GridPoint start, ContextCreator.GridPoint target) {
        if (start == null || target == null) {
            System.err.println("Error: Start o Target nulo.");
            return false;
        }
        if (start.equals(target)) {
            currentPath.add(start);
            return false;
        }
        return true;
    }
    
    /********** Calcula a distancia euclidiana entre dos puntos (heurística de A*) ***********/
    private double calculateHeuristic(ContextCreator.GridPoint p1, ContextCreator.GridPoint p2) {
        return Math.sqrt(Math.pow(p1.getX() - p2.getX(), 2) + Math.pow(p1.getY() - p2.getY(), 2));
    }

    /*********** Reconstruye el camino (desde el final hasta el origen) ***********/
    private List<ContextCreator.GridPoint> reconstructPath(
            Map<ContextCreator.GridPoint, ContextCreator.GridPoint> cameFrom, ContextCreator.GridPoint current) {
        List<ContextCreator.GridPoint> totalPath = new ArrayList<>();
        totalPath.add(current);
        while (cameFrom.containsKey(current)) {
            current = cameFrom.get(current);
            totalPath.add(current);
        }
        Collections.reverse(totalPath);
        return totalPath;
    }

    /*************** Explora los vecinos del agente ***************/
    private void exploreNeighbors(
            ContextCreator.GridPoint current,
            ContextCreator.GridPoint target,
            Map<ContextCreator.GridPoint, ContextCreator.GridPoint> cameFrom,
            Map<ContextCreator.GridPoint, Double> gScore,
            Map<ContextCreator.GridPoint, Double> fScore,
            List<ContextCreator.GridPoint> openList) {
        
        for (ContextCreator.GridPoint neighbor : getNeighbors(current)) {
            // Validar límites del grid
            if (!isWithinBounds(neighbor)) {
                continue;
            }

            // Verificar si es transitable y no está congestionado
            if (isTraversable(neighbor.getX(), neighbor.getY()) && countAgentsAt(neighbor) < 1) {
                // Calcular nuevo costo para llegar a este vecino
                double tentativeGScore = gScore.getOrDefault(current, Double.POSITIVE_INFINITY) + 1;
                
                // Si este es un mejor camino, actualizarlo
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
    
    // ========================================
 	// |         Métodos del agente           |
 	// ========================================
    /*************** Mover el agente ***************/
    private void moveAgent(ContextCreator.GridPoint nextPoint) {
    	if (!isValidStep(nextPoint)) {
            System.err.println("ERROR (GisAgent " + name + "): Intento de mover a celda inválida!");
            return;
        }
    	
        agentGrid.moveTo(this, nextPoint.getX(), nextPoint.getY());
        Coordinate newGeoCoord = ContextCreator.mapGridToGeo(nextPoint.getX(), nextPoint.getY());
        geography.move(this, geometryFactory.createPoint(newGeoCoord));
        distanceAccumulated -= ContextCreator.CELL_SIZE_METERS;
        cellsMoved++;
        recalculationCount = 0;
    }
    
    /*************** Obtener vecinos del agente ***************/
    private List<ContextCreator.GridPoint> getNeighbors(ContextCreator.GridPoint p) {
        List<ContextCreator.GridPoint> neighbors = new ArrayList<>();
        for (int dx = -1; dx <= 1; dx++) {
            for (int dy = -1; dy <= 1; dy++) {
                if (dx == 0 && dy == 0) {
                    continue;
                }
                int nx = p.getX() + dx;
                int ny = p.getY() + dy;
                neighbors.add(new ContextCreator.GridPoint(nx, ny));
            }
        }
        return neighbors;
    }

    /*************** Verificar si el agente está en la zona segura ***************/
    private boolean isInSafeZone(ContextCreator.GridPoint point) {
        Integer cellType = mapCellGrid.getOrDefault(point, 0);
        
        if (MapCell.isSafeZone(cellType)) {
            return true;
        }

        try {
            Geometry safeZoneGeom = geography.getGeometry(targetSafeZone);
            if (safeZoneGeom != null) {
                Coordinate coord = ContextCreator.mapGridToGeo(point.getX(), point.getY());
                Point geoPoint = geometryFactory.createPoint(coord);
                return safeZoneGeom.contains(geoPoint);
            }
        } catch (Exception e) {
            // Ignorar
        }
        return false;
    }
    
    /*************** Obtener posición del agente ***************/
    private ContextCreator.GridPoint getCurrentGridPoint() {
        GridPoint repastGridPoint = agentGrid.getLocation(this);
        if (repastGridPoint == null) {
            return null;
        }
        return new ContextCreator.GridPoint(repastGridPoint.getX(), repastGridPoint.getY());
    }
    
    /*************** Eliminar agente ***************
    private void removeAgent() {
        System.out.println("DEBUG (GisAgent " + name + "): Llegó al centroide, removiendo...");
        Context context = ContextUtils.getContext(this);
        if (context != null) {
            context.remove(this);
        }
    }*/
    
    
    // ========================================
 	// |             Verificar                |
 	// ========================================
    /********* Verificar si paso es válido (la celda está vacia y es transitable) *********/
    public boolean isValidStep(ContextCreator.GridPoint point) {
        if (!isWithinBounds(point) || !isTraversable(point.getX(), point.getY())) {
            return false;
        }
        return countAgentsAt(point) < 1;
    }

    /*************** Verificar si el punto está dentro de los limites ***************/
    private boolean isWithinBounds(ContextCreator.GridPoint point) {
        return point.getX() >= 0 && point.getX() < ContextCreator.GRID_WIDTH && 
        	   point.getY() >= 0 && point.getY() < ContextCreator.GRID_HEIGHT;
    }

    /************* Verificar si la celda es transitable (ROAD o SAFE) ***************/
    /*
    private boolean isTraversable(int x, int y) {
        Integer cellType = mapCellGrid.getOrDefault(new ContextCreator.GridPoint(x, y), 0);
        return cellType == MapCell.TYPE_ROAD || cellType == MapCell.TYPE_SAFE_ZONE;
    }
    */
    private boolean isTraversable(int x, int y) {
        Integer cellType = mapCellGrid.getOrDefault(new ContextCreator.GridPoint(x, y), 0);
        return MapCell.isTraversable(cellType);
    }

    /*************** Contar agentes en la celda (deberia ser 1) ***************/
    private int countAgentsAt(ContextCreator.GridPoint point) {
        int count = 0;
        for (GisAgent agent : agentGrid.getObjectsAt(point.getX(), point.getY())) {
            count++;
        }
        return count;
    }

    
	/*************** Setters y Getters ***************/
    public String getName() {
        return name;
    }

    public boolean isEvacuated() {
        return isEvacuated;
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

    public List<ContextCreator.GridPoint> getCurrentPath() {
        return currentPath;
    }

    public void setCurrentPath(List<ContextCreator.GridPoint> currentPath) {
        this.currentPath = currentPath;
    }
    public double getSpeed() {
        return speed;
    }
}