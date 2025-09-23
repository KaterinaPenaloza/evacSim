package evacSim.styles;

import java.awt.Color;

import evacSim.agents.ZoneAgent;
import gov.nasa.worldwind.render.SurfacePolygon;
import gov.nasa.worldwind.render.SurfaceShape;
import repast.simphony.visualization.gis3D.style.SurfaceShapeStyle;


public class ZoneStyle implements SurfaceShapeStyle<ZoneAgent>{

	@Override
	public SurfaceShape getSurfaceShape(ZoneAgent object, SurfaceShape shape) {
		return new SurfacePolygon();
	}

	@Override
	public Color getFillColor(ZoneAgent zone) {
		if ("safe".equals(zone.getName())) {
			return Color.GREEN;
		} else if ("initial".equals(zone.getName())) {
			return Color.RED;
		}
		// Color por defecto
		return Color.CYAN;
	}

	@Override
	public double getFillOpacity(ZoneAgent obj) {
		return 0.25;
	}

	@Override
	public Color getLineColor(ZoneAgent obj) {
	    return Color.BLACK;
	}

	@Override
	public double getLineOpacity(ZoneAgent obj) {
	    return 0.5;
	}

	@Override
	public double getLineWidth(ZoneAgent obj) {
	    return 2.0;
	}
}