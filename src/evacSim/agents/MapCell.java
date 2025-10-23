package evacSim.agents;

/**
 * Define los tipos de celdas y proporciona métodos de validación
 */
public class MapCell {
    public static final int TYPE_EMPTY = 0;
    public static final int TYPE_INITIAL_ZONE = 1;
    public static final int TYPE_SAFE_ZONE = 2;
    public static final int TYPE_ROAD = 3;
    public static final int TYPE_ROAD_IN_INITIAL = 4;  // Carretera dentro de zona inicial
    public static final int TYPE_ROAD_IN_SAFE = 5;     // Carretera dentro de zona segura
    
    /**
     * Verifica si una celda es transitable (solo carreteras y zonas con carreteras)
     */
    public static boolean isTraversable(int cellType) {
        return cellType == TYPE_ROAD 
            || cellType == TYPE_ROAD_IN_INITIAL 
            || cellType == TYPE_ROAD_IN_SAFE
            || cellType == TYPE_SAFE_ZONE;  // Permitir caminar en zona segura
    }
    
    /**
     * Verifica si una celda es válida para spawn (carretera en zona inicial)
     */
    public static boolean isValidSpawnCell(int cellType) {
        return cellType == TYPE_ROAD_IN_INITIAL;
    }
    
    /**
     * Verifica si una celda es zona segura o carretera en zona segura
     */
    public static boolean isSafeZone(int cellType) {
        return cellType == TYPE_SAFE_ZONE || cellType == TYPE_ROAD_IN_SAFE;
    }
    
    /**
     * Obtiene una descripción del tipo de celda
     */
    public static String getTypeName(int cellType) {
        switch(cellType) {
            case TYPE_EMPTY: return "EMPTY";
            case TYPE_INITIAL_ZONE: return "INITIAL_ZONE";
            case TYPE_SAFE_ZONE: return "SAFE_ZONE";
            case TYPE_ROAD: return "ROAD";
            case TYPE_ROAD_IN_INITIAL: return "ROAD_IN_INITIAL";
            case TYPE_ROAD_IN_SAFE: return "ROAD_IN_SAFE";
            default: return "UNKNOWN";
        }
    }
}
