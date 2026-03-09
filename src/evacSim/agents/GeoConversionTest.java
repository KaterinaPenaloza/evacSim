package evacSim.agents;

import org.locationtech.jts.geom.Coordinate;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Locale;

public class GeoConversionTest {

    private static final String CITY_NAME = "Arica";

    public static void main(String[] args) {
        System.out.println("=== PRUEBA AUTOMÁTICA DE DISTANCIAS: GEO → ENU → GEO ===");
        System.out.println("Ciudad: " + CITY_NAME + "\n");

        // Punto inicial en Viña del Mar (debe estar DENTRO del bbox del mapa)
        double latInicial = -18.46657790987584;
        double lonInicial = -70.29835853223453;
        
        System.out.printf("Punto inicial: %.10f, %.10f\n", latInicial, lonInicial);
        System.out.printf("CELL_SIZE_METERS: %.3f m\n\n", ContextCreator.CELL_SIZE_METERS); 

        // Distancias a probar (en metros)
        double[] distancias = {10, 50, 100, 500, 1000, 5000, 10000};
        
        // Direcciones a probar (en grados)
        double[] direcciones = {0, 45, 90, 180, 270};
        String[] nombresDireccion = {"Norte", "Noreste", "Este", "Sur", "Oeste"};

        // Crear/limpiar archivo
        inicializarArchivo();

        // Ejecutar todas las combinaciones
        int contador = 0;
        for (double distancia : distancias) {
            for (int i = 0; i < direcciones.length; i++) {
                double direccion = direcciones[i];
                String nombre = nombresDireccion[i];
                
                contador++;
                System.out.printf("--- Prueba %d: %.0f m hacia %s (%.0f°) ---\n", 
                                contador, distancia, nombre, direccion);
                
                ejecutarPrueba(latInicial, lonInicial, distancia, direccion, nombre);
                System.out.println();
            }
        }

        System.out.println("=== PRUEBAS COMPLETADAS ===");
        System.out.printf("Total de pruebas: %d\n", contador);
        System.out.println("✅ Todos los resultados guardados en prueba_distancias.csv");
        System.out.println("\nAhora puedes:");
        System.out.println("1. Abrir prueba_distancias.csv en Excel");
        System.out.println("2. Verificar algunos puntos en Google Maps");
        System.out.println("3. Agregar una columna 'Distancia_Google_m' con tus mediciones");
    }

    private static void ejecutarPrueba(double latInicial, double lonInicial, 
                                      double distanciaMetros, double direccionGrados,
                                      String nombreDireccion) {
        // 1) Convertir punto inicial de GEO a GRID
        ContextCreator.GridPoint gridInicial = ContextCreator.mapGeoToGrid(
            new Coordinate(lonInicial, latInicial)
        );

        // 2) Calcular desplazamiento en ENU (metros)
        double anguloRad = Math.toRadians(direccionGrados);
        double deltaEast = distanciaMetros * Math.sin(anguloRad);
        double deltaNorth = distanciaMetros * Math.cos(anguloRad);

        // 3) Convertir desplazamiento de metros a celdas
        int deltaCeldasX = (int) Math.round(deltaEast / ContextCreator.CELL_SIZE_METERS);
        int deltaCeldasY = (int) Math.round(deltaNorth / ContextCreator.CELL_SIZE_METERS);

        // 4) Calcular punto final en GRID
        int gridFinalX = gridInicial.getX() + deltaCeldasX;
        int gridFinalY = gridInicial.getY() + deltaCeldasY;

        // 5) Convertir punto final de GRID a GEO
        Coordinate coordFinal = ContextCreator.mapGridToGeo(gridFinalX, gridFinalY);
        double latFinal = coordFinal.y;
        double lonFinal = coordFinal.x;

        // 6) Calcular distancias
        double distanciaSimulador = Math.sqrt(
            Math.pow(deltaCeldasX * ContextCreator.CELL_SIZE_METERS, 2) + 
            Math.pow(deltaCeldasY * ContextCreator.CELL_SIZE_METERS, 2)
        );

        double distanciaHaversine = haversineDistance(latInicial, lonInicial, latFinal, lonFinal);

        // Mostrar resultados en consola
        System.out.printf("  Grid inicial: (%d, %d) → Grid final: (%d, %d)\n", 
                         gridInicial.getX(), gridInicial.getY(), gridFinalX, gridFinalY);
        System.out.printf("  Δ celdas: (%d, %d)\n", deltaCeldasX, deltaCeldasY);
        System.out.printf("  Punto final: %.10f, %.10f\n", latFinal, lonFinal);
        System.out.printf("  Distancia simulador: %.3f m\n", distanciaSimulador);
        System.out.printf("  Distancia Haversine: %.3f m\n", distanciaHaversine);

        // 7) Guardar en CSV
        guardarEnExcel(latInicial, lonInicial, latFinal, lonFinal, 
                      distanciaSimulador, distanciaHaversine, direccionGrados, nombreDireccion);
    }

    private static void inicializarArchivo() {
        String archivo = "prueba_distancias.csv";
        try {
            java.io.File file = new java.io.File(archivo);
            
            // Solo crear cabecera si el archivo NO existe o está vacío
            if (!file.exists() || file.length() == 0) {
                FileWriter fw = new FileWriter(archivo, false);
                fw.write("Ciudad,Lat_Inicial,Lon_Inicial,Lat_Final,Lon_Final," +
                        "Distancia_Simulador_m,Distancia_Haversine_m,Direccion_grados,Direccion_nombre\n");
                fw.close();
                System.out.println("✅ Archivo creado con cabecera: " + archivo + "\n");
            } else {
                System.out.println("✅ Archivo existente encontrado. Se agregarán datos al final.\n");
            }
        } catch (IOException e) {
            System.err.println("❌ Error al verificar archivo: " + e.getMessage());
            e.printStackTrace();
        }
    }

    private static void guardarEnExcel(double lat1, double lon1, double lat2, double lon2,
                                      double distSimulador, double distHaversine, 
                                      double direccion, String nombreDireccion) {
        String archivo = "prueba_distancias.csv";
        
        try {
            FileWriter fw = new FileWriter(archivo, true); // true = append
            
            String linea = String.format(Locale.US, 
                "%s,%.10f,%.10f,%.10f,%.10f,%.3f,%.3f,%.1f,%s\n",
                CITY_NAME, lat1, lon1, lat2, lon2, distSimulador, distHaversine, 
                direccion, nombreDireccion);
            
            fw.write(linea);
            fw.close();
            
        } catch (IOException e) {
            System.err.println("❌ Error al guardar: " + e.getMessage());
            e.printStackTrace();
        }
    }

    private static double haversineDistance(double lat1, double lon1, double lat2, double lon2) {
        final double R = 6371008.8; // Radio de la Tierra en metros (WGS84)
        double phi1 = Math.toRadians(lat1);
        double phi2 = Math.toRadians(lat2);
        double dphi = Math.toRadians(lat2 - lat1);
        double dlambda = Math.toRadians(lon2 - lon1);

        double a = Math.sin(dphi / 2) * Math.sin(dphi / 2) +
                  Math.cos(phi1) * Math.cos(phi2) * 
                  Math.sin(dlambda / 2) * Math.sin(dlambda / 2);
        double c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
        
        return R * c;
    }
}