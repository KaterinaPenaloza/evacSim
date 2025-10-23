package evacSim.styles;

import java.awt.Color;
import java.awt.Font;
import java.awt.image.BufferedImage;
import java.util.HashMap;
import java.util.Map;

import evacSim.agents.GisAgent;
import gov.nasa.worldwind.WorldWind;
import gov.nasa.worldwind.avlist.AVKey;
import gov.nasa.worldwind.render.BasicWWTexture;
import gov.nasa.worldwind.render.Material;
import gov.nasa.worldwind.render.Offset;
import gov.nasa.worldwind.render.WWTexture;
import repast.simphony.visualization.gis3D.PlaceMark;
import repast.simphony.visualization.gis3D.style.MarkStyle;

/**
 * Style for GisAgents. This style changes the appearance of the GisAgents
 * based on their speed value.
 * 
 * Speed colors:
 * - 0.5 m/s -> BROWN-ORANGE (naranjo oscuro casi café)
 * - 1.0 m/s -> YELLOW (amarillo)
 * - 1.5 m/s -> DARK BLUE (azul oscuro)
 * 
 * @author Eric Tatara
 */
public class GisAgentStyle implements MarkStyle<GisAgent> {

	private Offset labelOffset;
	private Map<String, WWTexture> textureMap;

	// Constantes de velocidad
	private static final double SPEED_SLOW = 0.5;
	private static final double SPEED_MEDIUM = 1.0;
	private static final double SPEED_FAST = 1.5;
	
	// Colores por velocidad
	private static final Color COLOR_SLOW = new Color(189, 94, 0); // naranjo oscuro casi café
	private static final Color COLOR_MEDIUM = new Color(244, 235, 113); // amarillo
	private static final Color COLOR_FAST = new Color(48, 0, 138); // azul oscuro

	// Color del borde (negro)
	private static final Color BORDER_COLOR = Color.BLACK;
	private static final float BORDER_WIDTH = 4.0f; // Borde más grueso
	
	public GisAgentStyle() {
		labelOffset = new Offset(1.2d, 0.6d, AVKey.FRACTION, AVKey.FRACTION);
		textureMap = new HashMap<>();

		// Crear texturas para cada velocidad con bordes gruesos
		createTextures();
	}

	/**
	 * Crea las texturas para cada velocidad con bordes gruesos y negros
	 */
	private void createTextures() {
		// Textura naranjo oscuro para velocidad 0.5 m/s (lenta)
		BufferedImage imageSlow = createCircle(COLOR_SLOW);
		textureMap.put("speed_0.5", new BasicWWTexture(imageSlow));

		// Textura amarilla para velocidad 1.0 m/s (media)
		BufferedImage imageMedium = createCircle(COLOR_MEDIUM);
		textureMap.put("speed_1.0", new BasicWWTexture(imageMedium));

		// Textura azul oscuro para velocidad 1.5 m/s (rápida)
		BufferedImage imageFast = createCircle(COLOR_FAST);
		textureMap.put("speed_1.5", new BasicWWTexture(imageFast));
	}

	/**
	 * Crea un círculo con borde negro grueso
	 */
	private BufferedImage createCircle(Color fillColor) {
		int size = 60; // Tamaño aumentado para mejor visibilidad
		BufferedImage image = new BufferedImage(size, size, BufferedImage.TYPE_INT_ARGB);
		java.awt.Graphics2D g2d = image.createGraphics();
		
		// Activar antialiasing para bordes suaves
		g2d.setRenderingHint(java.awt.RenderingHints.KEY_ANTIALIASING, 
		                     java.awt.RenderingHints.VALUE_ANTIALIAS_ON);
		
		int center = size / 2;
		int radius = (int)(size * 0.4); // Radio del círculo
		
		// Dibujar el círculo de relleno
		g2d.setColor(fillColor);
		g2d.fillOval(center - radius, center - radius, radius * 2, radius * 2);
		
		// Dibujar el borde negro grueso
		g2d.setColor(BORDER_COLOR);
		g2d.setStroke(new java.awt.BasicStroke(BORDER_WIDTH));
		g2d.drawOval(center - radius, center - radius, radius * 2, radius * 2);
		
		g2d.dispose();
		return image;
	}

	/**
	 * Obtiene la clave de textura según la velocidad del agente
	 */
	private String getTextureKey(double speed) {
		if (Math.abs(speed - SPEED_SLOW) < 0.01) {
			return "speed_0.5";
		} else if (Math.abs(speed - SPEED_MEDIUM) < 0.01) {
			return "speed_1.0";
		} else if (Math.abs(speed - SPEED_FAST) < 0.01) {
			return "speed_1.5";
		}
		// Por defecto, usar velocidad media
		return "speed_1.0";
	}

	@Override
	public PlaceMark getPlaceMark(GisAgent agent, PlaceMark mark) {
		if (mark == null) {
			mark = new PlaceMark();
		}

		mark.setAltitudeMode(WorldWind.RELATIVE_TO_GROUND);
		mark.setLineEnabled(false);

		return mark;
	}

	@Override
	public double getElevation(GisAgent agent) {
		return 0;
	}

	/**
	 * Retorna la textura según la velocidad del agente
	 */
	@Override
	public WWTexture getTexture(GisAgent agent, WWTexture texture) {
		String textureKey = getTextureKey(agent.getSpeed());
		return textureMap.get(textureKey);
	}

	@Override
	public double getScale(GisAgent agent) {
		return 0.25; // Ligeramente más grande para mejor visibilidad
	}

	@Override
	public double getHeading(GisAgent agent) {
		return 0;
	}

	@Override
	public String getLabel(GisAgent agent) {
		// Opcional: mostrar la velocidad como etiqueta
		// return String.format("%.1f m/s", agent.getSpeed());
		return null;
	}

	@Override
	public Color getLabelColor(GisAgent agent) {
		return Color.YELLOW;
	}

	@Override
	public Offset getLabelOffset(GisAgent agent) {
		return labelOffset;
	}

	@Override
	public Font getLabelFont(GisAgent obj) {
		return null;
	}

	@Override
	public double getLineWidth(GisAgent agent) {
		return 0;
	}

	@Override
	public Material getLineMaterial(GisAgent obj, Material lineMaterial) {
		if (lineMaterial == null) {
			lineMaterial = new Material(Color.RED);
		}
		return lineMaterial;
	}

	@Override
	public Offset getIconOffset(GisAgent obj) {
		return Offset.CENTER;
	}
}