# Map Converter

Este módulo prepara los datos geoespaciales necesarios para la simulación de evacuación.

Convierte un archivo **GeoJSON** con zonas definidas por el usuario en **Shapefiles**, y además descarga o procesa datos de **OpenStreetMap** para extraer la red de carreteras del área de estudio.

Los archivos generados son utilizados posteriormente por el modelo de simulación.

---

## Flujo de procesamiento

 ![Flujo](/docs/diagrama_secuencia_preprocesamiento.png)

---

## Archivos de entrada

El script espera un archivo GeoJSON que contenga polígonos con propiedades que indiquen el tipo de zona.

Tipos de zona:

- `initial` → zonas donde comienzan los agentes
- `safe` → zonas de evacuación seguras

El archivo puede crearse usando herramientas como **geojson.io** (recomendado).

Ejemplo incluido:

* map.geojson


---

## Archivos generados

El script genera varios archivos en el directorio de salida:

* map_data_initial_zones.shp → zonas iniciales de evacuación
* map_data_safe_zones.shp → zonas seguras
* map_data_roads.shp → red de carreteras extraída desde OpenStreetMap
* bbox_values.txt → límites geográficos utilizados

---

## Uso

Ejecutar el script principal:

```bash
python geojson_to_shp_converter.py --geojson map.geojson [--osm_pbf chile-latest.osm.pbf] [--output_dir ./]
```

Los archivos PBF de OpenStreetMap pueden descargarse desde  
[Geofabrik – South America downloads](https://download.geofabrik.de/south-america.html).

Si no se proporciona un archivo PBF de OpenStreetMap, el script descargará los datos automáticamente usando la API de OSM.

---

## Dependencias

Python:

- geopandas
- geojson

Herramientas externas:

- osmconvert
- osmfilter
- wget
- WSL (en sistemas Windows)

## Visualización del mapa

El proyecto incluye un archivo de proyecto de **QGIS**:
* map.qgz

Este archivo permite visualizar las zonas y la red de carreteras generadas.

Ejemplo:

![Mapa de zonas de evacuación](/docs/mapa_ejemplo.png)
