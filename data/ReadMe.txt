# Data

Esta carpeta contiene los datos geoespaciales utilizados como entrada para el simulador de evacuación.

Los archivos corresponden principalmente a mapas urbanos en formato **Shapefile**, que describen la red vial, las zonas iniciales de los agentes y las zonas seguras del entorno simulado.
Estos datos son utilizados por el modelo de simulación para construir el entorno espacial donde se desplazan los agentes durante la evacuación.

---

## Descripción de los datos

### Red vial

Archivos que contienen la estructura de calles y caminos del entorno urbano. Estos archivos describen la red de caminos utilizada por los agentes para desplazarse durante la evacuación.

* map_data_roads.shp
* map_data_roads.dbf
* map_data_roads.shx
* map_data_roads.prj

### Zonas iniciales

Archivos que definen las zonas donde se generan inicialmente los agentes en la simulación. Estas zonas representan el área donde se ubica la población al inicio del escenario.

* map_data_initial_zones.shp
* map_data_initial_zones.dbf
* map_data_initial_zones.shx
* map_data_initial_zones.prj

### Zonas seguras

Archivos que contienen las zonas de evacuación o puntos seguros hacia donde se dirigen los agentes.

* map_data_safe_zones.shp
* map_data_safe_zones.dbf
* map_data_safe_zones.shx
* map_data_safe_zones.prj

### Bounding Box

Este archivo contiene los límites espaciales del mapa (bounding box) utilizados para definir el área de simulación.

* bbox_values.txt


### GeoJSON

Versión del mapa en formato **GeoJSON**, utilizada para visualización o procesamiento geoespacial adicional.

* map.geojson

---

## Uso en el simulador

Durante la ejecución del modelo:

1. Se carga la red vial del entorno urbano.
2. Se generan los agentes en las zonas iniciales.
3. Los agentes navegan por la red de calles.
4. El objetivo es alcanzar alguna de las zonas seguras definidas en el mapa.

Estos datos permiten construir el **entorno espacial realista** sobre el cual se ejecuta la simulación basada en agentes.
