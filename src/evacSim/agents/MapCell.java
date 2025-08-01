package evacSim.agents;

/* Tipos de celdas */
public class MapCell {
    public static final int TYPE_UNKNOWN = 0;
    public static final int TYPE_INITIAL_ZONE = 1;
    public static final int TYPE_SAFE_ZONE = 2;
    public static final int TYPE_ROAD = 3;

    private int type;

    public MapCell() {
        this.type = TYPE_UNKNOWN;
    }

    public void setType(int type) {
        this.type = type;
    }

    public int getType() {
        return type;
    }

    public boolean isTraversable() {
        // Celdas transitables road y safe zone
        return type == TYPE_ROAD || type == TYPE_SAFE_ZONE ;
    }
}