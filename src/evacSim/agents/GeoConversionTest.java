package evacSim.agents;

import org.locationtech.jts.geom.Coordinate;
import repast.simphony.space.grid.GridPoint;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Locale;

public class GeoConversionTest {

	private static final String CITY_NAME = "Viña del mar";
	// === PUNTOS A COMPARAR (puedes cambiarlos o pasarlos por args) ===

	private static final double LAT1 = -33.0192943073785;
	private static final double LON1 = -71.5522698210275;

	private static final double LAT2 = -33.01934890614157;
	private static final double LON2 = -71.55119763066084;

	public static void main(String[] args) {
		double lat1 = LAT1, lon1 = LON1, lat2 = LAT2, lon2 = LON2;
		String city = CITY_NAME;

		// Permitir pasar 4 argumentos: lat1 lon1 lat2 lon2
		if (args != null && args.length == 4) {
			lat1 = Double.parseDouble(args[0]);
			lon1 = Double.parseDouble(args[1]);
			lat2 = Double.parseDouble(args[2]);
			lon2 = Double.parseDouble(args[3]);
		}

		System.out.println("=== TEST: Dos coordenadas GEO -> GRID y comparación de distancias ===");
		System.out.printf("Punto A (lat,lon): %.8f, %.8f%n", lat1, lon1);
		System.out.printf("Punto B (lat,lon): %.8f, %.8f%n", lat2, lon2);

		// 1) GEO -> GRID
		evacSim.agents.ContextCreator.GridPoint gA = ContextCreator.mapGeoToGrid(new Coordinate(lon1, lat1));
		evacSim.agents.ContextCreator.GridPoint gB = ContextCreator.mapGeoToGrid(new Coordinate(lon2, lat2));

		System.out.printf("Grid A (x,y): %d, %d%n", gA.getX(), gA.getY());
		System.out.printf("Grid B (x,y): %d, %d%n", gB.getX(), gB.getY());

		// 2) Δx, Δy y distancias por grilla
		int dx = gB.getX() - gA.getX();
		int dy = gB.getY() - gA.getY();
		double cellSize = ContextCreator.CELL_SIZE_METERS;

		double distCells = Math.sqrt(dx * dx + dy * dy);
		double distMetersGrid = distCells * cellSize;

		System.out.println("\n=== Distancias en GRID ===");
		System.out.printf("Δx (celdas): %d; Δy (celdas): %d%n", dx, dy);
		System.out.printf("Distancia euclidiana (celdas): %.3f%n", distCells);
		System.out.printf("CELL_SIZE_METERS: %.3f m%n", cellSize);
		System.out.printf("Distancia por grilla (m): %.3f%n", distMetersGrid);
		System.out.printf("Componentes E-O (m): %.3f; N-S (m): %.3f%n", Math.abs(dx) * cellSize,
				Math.abs(dy) * cellSize);

		// 3) GRID -> GEO (reconstrucción de ambos, útil para copiar en Google)
		Coordinate backA = ContextCreator.mapGridToGeo(gA.getX(), gA.getY());
		Coordinate backB = ContextCreator.mapGridToGeo(gB.getX(), gB.getY());
		System.out.println("\n=== Copiar en Google Maps (Medir distancia) ===");
		System.out.printf("A: %.8f, %.8f%n", backA.y, backA.x);
		System.out.printf("B: %.8f, %.8f%n", backB.y, backB.x);

		// 4) (Opcional) Distancia geodésica Haversine (WGS84) para referencia
		double distHaversine = 100.22;
		System.out.println("\n=== Referencia geodésica (Haversine) ===");
		System.out.printf("Distancia Haversine (m): %.3f%n", distHaversine);

		// 5) Error relativo vs Haversine (aprox. a lo que verás en Google)
		double errAbs = Math.abs(distMetersGrid - distHaversine);
		double errPct = (distHaversine > 0) ? (errAbs / distHaversine * 100.0) : 0.0;
		System.out.printf("Error absoluto (m): %.3f%n", errAbs);
		System.out.printf("Error relativo (%%): %.3f%n", errPct);

		// 6) Guardar en un archivo CSV acumulativo (append)
		String csvLine = String.format(Locale.US, "%s,%.8f,%.8f,%.8f,%.8f,%d,%d,%.3f,%.3f,%.3f,%.3f,%.3f%n", city, lat1,
				lon1, lat2, lon2, dx, dy, cellSize, distMetersGrid, distHaversine, errAbs, errPct);

		String fileName = "geoConvTest_results.csv";
		try (FileWriter fw = new FileWriter(fileName, true); PrintWriter pw = new PrintWriter(fw)) {

			// Si el archivo está vacío, agrega cabecera
			if (new java.io.File(fileName).length() == 0) {
				pw.println("city,lat1,lon1,lat2,lon2,dx,dy,cell_size,dist_grid_m,dist_haversine_m,err_abs_m,err_pct");
			}

			pw.print(csvLine);
			System.out.println("\n✅ Línea guardada en " + fileName);

		} catch (IOException e) {
			System.err.println("❌ Error al escribir el archivo: " + e.getMessage());
		}
	}

	/**
	 * Distancia Haversine (m) con radio WGS84 medio ~ 6371e3 m. Es una buena
	 * aproximación a lo que te mostrará Google en distancias "medir".
	 */
	private static double haversineMeters(double lat1, double lon1, double lat2, double lon2) {
		final double R = 6371008.8; // radio medio de la Tierra en metros (WGS84)
		double phi1 = Math.toRadians(lat1);
		double phi2 = Math.toRadians(lat2);
		double dphi = Math.toRadians(lat2 - lat1);
		double dlambda = Math.toRadians(lon2 - lon1);

		double a = Math.sin(dphi / 2) * Math.sin(dphi / 2)
				+ Math.cos(phi1) * Math.cos(phi2) * Math.sin(dlambda / 2) * Math.sin(dlambda / 2);
		double c = 2 * Math.atan2(Math.sqrt(a), Math.sqrt(1 - a));
		return R * c;
	}
}
