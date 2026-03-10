import geojson
import subprocess as sub
import json
import getopt
import sys
import os
import pathlib
from geojson import FeatureCollection, Feature, Point, Polygon, LineString, MultiLineString
import geopandas as gpd

def uso():
    sys.exit("Uso: " + sys.argv[0] + " --geojson <file_geojson> [--osm_pbf <file_openstreemap>] [--output_dir <directory>]")

argv = sys.argv[1:]
file_geojson = ""
file_osm_pbf = ""
output_directory = "." # Directorio de salida por defecto
# hacer que el directorio de salida sea /data para que repast lo lea directamente

try:
    opts, args = getopt.getopt(argv, "g:p:o:", ["geojson=", "osm_pbf=", "output_dir="])
except getopt.GetoptError as err:
    print(err)
    uso()

for opt, arg in opts:
    if opt in ["-g", "--geojson"]:
        file_geojson = arg
    elif opt in ["-p", "--osm_pbf"]:
        file_osm_pbf = arg
    elif opt in ["-o", "--output_dir"]:
        output_directory = arg

if file_geojson == "":
    uso()

pathlib.Path(output_directory).mkdir(parents=True, exist_ok=True)
filename_base = "map_data" # Nombre base para los archivos de salida
offset = 0.01 * 1  # 0.01 grados para el bounding box

try:
    with open(file_geojson, 'r') as data_file:
        data = json.load(data_file)
except FileNotFoundError:
    sys.exit(f"Error: El archivo GeoJSON '{file_geojson}' no se encontró.")
except json.JSONDecodeError:
    sys.exit(f"Error: El archivo GeoJSON '{file_geojson}' no es un JSON válido.")

fcollection = FeatureCollection(data['features'])

# --- PASO 1: Separar zonas y calcular bounding box ---
initial_zones_features = []
safe_zones_features = []
all_zone_coords = []

for feature in fcollection["features"]:
    if feature["geometry"]["type"] == "Polygon" or feature["geometry"]["type"] == "MultiPolygon":
        if "zoneType" in feature["properties"]:
            if feature["properties"]["zoneType"] == "initial":
                initial_zones_features.append(feature)
            elif feature["properties"]["zoneType"] == "safe":
                safe_zones_features.append(feature)
            # Recopilar todas las coordenadas para el bounding box
            if feature["geometry"]["type"] == "Polygon":
                for ring in feature["geometry"]["coordinates"]:
                    all_zone_coords.extend(ring)
            elif feature["geometry"]["type"] == "MultiPolygon":
                for poly_coords in feature["geometry"]["coordinates"]:
                    for ring in poly_coords:
                        all_zone_coords.extend(ring)

# Calcular bounding box a partir de todas las coordenadas de las zonas
if not all_zone_coords:
    sys.exit("Error: No se encontraron zonas (initial/safe) válidas en el GeoJSON de entrada.")

min_lon = min(c[0] for c in all_zone_coords)
min_lat = min(c[1] for c in all_zone_coords)
max_lon = max(c[0] for c in all_zone_coords)
max_lat = max(c[1] for c in all_zone_coords)

min_lon -= offset
min_lat -= offset
max_lon += offset
max_lat += offset

boxCoords = f"{min_lon},{min_lat},{max_lon},{max_lat}"

print("\n--- Procesamiento de Zonas y Bounding Box ---")
print(f"Caja Geográfica calculada:")
print(f"  min_lon: {min_lon:.15f}; min_lat: {min_lat:.15f}; max_lon: {max_lon:.15f}; max_lat: {max_lat:.15f}")

bbox_file = os.path.join(output_directory, "bbox_values.txt")
with open(bbox_file, "w") as f:
    f.write(f"{min_lon:.15f}\n")
    f.write(f"{max_lon:.15f}\n")
    f.write(f"{min_lat:.15f}\n")
    f.write(f"{max_lat:.15f}\n")
print(f"[OK] bbox_values.txt -> {bbox_file}")

# Guardar GeoJSONs de zonas separadas
output_initial_geojson_path = os.path.join(output_directory, f"{filename_base}_initial_zones.geojson")
output_safe_geojson_path = os.path.join(output_directory, f"{filename_base}_safe_zones.geojson")

with open(output_initial_geojson_path, 'w') as f:
    geojson.dump(FeatureCollection(initial_zones_features), f, indent=2)
print(f"  Zonas iniciales guardadas en: {output_initial_geojson_path}")

with open(output_safe_geojson_path, 'w') as f:
    geojson.dump(FeatureCollection(safe_zones_features), f, indent=2)
print(f"  Zonas seguras guardadas en: {output_safe_geojson_path}")


# --- PASO 2: Obtener datos OSM (descargar o usar PBF existente) ---
print("\n--- Obtención y Procesamiento de Datos OSM ---")

# Rutas para Windows y WSL
current_windows_dir = pathlib.Path(os.getcwd())
output_windows_dir = pathlib.Path(output_directory).resolve()

def to_wsl_path(windows_path):
    abs_windows_path = pathlib.Path(windows_path).resolve()
    wsl_drive = abs_windows_path.drive.replace(':', '').lower()
    wsl_path_parts = abs_windows_path.as_posix().replace(str(abs_windows_path.drive), '')
    return f"/mnt/{wsl_drive}{wsl_path_parts}"

wsl_output_dir = to_wsl_path(output_windows_dir)
wsl_geojson_path = to_wsl_path(file_geojson)
wsl_osm_pbf_path_input = to_wsl_path(file_osm_pbf) if file_osm_pbf else ""


temp_osm_path_windows = os.path.join(output_windows_dir, f"{filename_base}_temp.osm")
temp_pbf_path_windows = os.path.join(output_windows_dir, f"{filename_base}_temp.pbf")
temp_osm_path_wsl = to_wsl_path(temp_osm_path_windows)
temp_pbf_path_wsl = to_wsl_path(temp_pbf_path_windows)

# Descargar o recortar PBF
if file_osm_pbf and os.path.exists(file_osm_pbf):
    print(f"1.1) Usando PBF existente: {file_osm_pbf}")
    osmconvert_cmd = ['wsl', 'osmconvert', wsl_osm_pbf_path_input, '-b=' + boxCoords, '-o=' + temp_pbf_path_wsl]
    print(f"  DEBUG (WSL): {' '.join(osmconvert_cmd)}")
    result = sub.run(osmconvert_cmd, capture_output=True, text=True)
    print("  WSL stdout:", result.stdout)
    print("  WSL stderr:", result.stderr)
    if result.returncode != 0:
        sys.exit(f"ERROR: osmconvert terminó con código de error {result.returncode} al recortar PBF.")
    if not os.path.exists(temp_pbf_path_windows) or os.stat(temp_pbf_path_windows).st_size == 0:
        sys.exit(f"ERROR: PBF recortado '{temp_pbf_path_windows}' está vacío o no se generó. Revisa coordenadas o PBF de entrada.")
else:
    print("1.1) Descargando datos de OpenStreetMap via API...")
    osmURL = f"https://api.openstreetmap.org/api/0.6/map?bbox={boxCoords}"
    wget_cmd = ['wsl', 'wget', osmURL, '-O', temp_osm_path_wsl]
    print(f"  DEBUG (WSL): {' '.join(wget_cmd)}")
    result = sub.run(wget_cmd, capture_output=True, text=True)
    print("  WSL stdout:", result.stdout)
    print("  WSL stderr:", result.stderr)
    if result.returncode != 0:
        sys.exit(f"ERROR: wget terminó con código de error {result.returncode} al descargar OSM.")
    if not os.path.exists(temp_osm_path_windows) or os.stat(temp_osm_path_windows).st_size == 0:
        sys.exit(f"Error: Archivo OSM descargado '{temp_osm_path_windows}' está vacío o no se generó.")

    print(f"1.2) Convirtiendo OSM a PBF: {temp_osm_path_windows} -> {temp_pbf_path_windows}")
    osmconvert_cmd = ['wsl', 'osmconvert', temp_osm_path_wsl, '-o=' + temp_pbf_path_wsl]
    print(f"  DEBUG (WSL): {' '.join(osmconvert_cmd)}")
    result = sub.run(osmconvert_cmd, capture_output=True, text=True)
    print("  WSL stdout:", result.stdout)
    print("  WSL stderr:", result.stderr)
    if result.returncode != 0:
        sys.exit(f"ERROR: osmconvert terminó con código de error {result.returncode} al convertir a PBF.")
    if not os.path.exists(temp_pbf_path_windows) or os.stat(temp_pbf_path_windows).st_size == 0:
        sys.exit(f"ERROR: PBF convertido '{temp_pbf_path_windows}' está vacío o no se generó.")

# Convertir PBF a OSM XML para extraer las carreteras con osmfilter
print(f"1.3) Convirtiendo PBF a OSM XML para filtrado de carreteras: {temp_pbf_path_windows}")
temp_osm_for_filter_path_windows = os.path.join(output_windows_dir, f"{filename_base}_filter.osm")
temp_osm_for_filter_path_wsl = to_wsl_path(temp_osm_for_filter_path_windows)

osmconvert_xml_cmd = ['wsl', 'osmconvert', temp_pbf_path_wsl, '-o=' + temp_osm_for_filter_path_wsl]
print(f"  DEBUG (WSL): {' '.join(osmconvert_xml_cmd)}")
result = sub.run(osmconvert_xml_cmd, capture_output=True, text=True)
print("  WSL stdout:", result.stdout)
print("  WSL stderr:", result.stderr)
if result.returncode != 0:
    sys.exit(f"ERROR: osmconvert terminó con código de error {result.returncode} al generar OSM XML para filtrado.")
if not os.path.exists(temp_osm_for_filter_path_windows) or os.stat(temp_osm_for_filter_path_windows).st_size == 0:
    sys.exit(f"ERROR: Archivo OSM XML para filtrado '{temp_osm_for_filter_path_windows}' está vacío o no se generó.")


# --- PASO 3: Extraer carreteras del archivo OSM XML usando geopandas ---
print("\n--- Extracción de Carreteras (Roads) ---")
output_roads_geojson_path = os.path.join(output_directory, f"{filename_base}_roads.geojson")

# Comando osmfilter para extraer solo las vías que son carreteras.
# obtiene LineStrings y MultiLineStrings.
output_roads_osm_xml_filtered_path_windows = os.path.join(output_windows_dir, f"{filename_base}_roads_filtered.osm")
output_roads_osm_xml_filtered_path_wsl = to_wsl_path(output_roads_osm_xml_filtered_path_windows)

# Filtra solo las vías que tienen la etiqueta 'highway'
osmfilter_roads_cmd = [
    'wsl', 'osmfilter',
    temp_osm_for_filter_path_wsl,
    '--keep-ways=highway=*',
    '-o=' + output_roads_osm_xml_filtered_path_wsl
]
print(f"  DEBUG (WSL - osmfilter roads): {' '.join(osmfilter_roads_cmd)}")
result = sub.run(osmfilter_roads_cmd, capture_output=True, text=True)
print("  WSL stdout:", result.stdout)
print("  WSL stderr:", result.stderr)
if result.returncode != 0:
    sys.exit(f"ERROR: osmfilter terminó con código de error {result.returncode} al filtrar carreteras.")
if not os.path.exists(output_roads_osm_xml_filtered_path_windows) or os.stat(output_roads_osm_xml_filtered_path_windows).st_size == 0:
    print(f"ADVERTENCIA: Archivo de carreteras filtrado '{output_roads_osm_xml_filtered_path_windows}' está vacío o no se generó. Esto puede ser normal si no hay carreteras en el bounding box, o un error de filtrado.")

# Leer LineStrings y MultiLineStrings con geopandas.
print(f"  Leyendo carreteras y convirtiendo a GeoJSON: {output_roads_geojson_path}")
try:
    # Intenta leer la capa 'lines'. Esto debería incluir tanto LineString como MultiLineString
    gdf_roads = gpd.read_file(output_roads_osm_xml_filtered_path_windows, layer='lines')

    if not gdf_roads.empty:
        gdf_roads_filtered = gdf_roads[
            (gdf_roads.geometry.geom_type == 'LineString') |
            (gdf_roads.geometry.geom_type == 'MultiLineString')
        ]

        if not gdf_roads_filtered.empty:
            gdf_roads_filtered.to_file(output_roads_geojson_path, driver='GeoJSON', encoding='utf-8')
            print(f"  Carreteras guardadas en: {output_roads_geojson_path}")
        else:
            print(f"  ADVERTENCIA: El GeoJSON de carreteras filtrado no contiene LineString o MultiLineString válidos.")
            # Crear un GeoJSON vacío si no hay carreteras válidas
            with open(output_roads_geojson_path, 'w') as f:
                geojson.dump(FeatureCollection([]), f, indent=2)

    else:
        print(f"  ADVERTENCIA: No se encontraron elementos en la capa 'lines' del archivo OSM filtrado. Generando GeoJSON de carreteras vacío.")
        # Crear un GeoJSON vacío si no se encontró nada
        with open(output_roads_geojson_path, 'w') as f:
            geojson.dump(FeatureCollection([]), f, indent=2)

except Exception as e:
    print(f"  ERROR al leer/convertir carreteras con geopandas: {e}")
    sys.exit(f"Fallo crítico: No se pudieron procesar las carreteras. Error: {e}") # Salir si falla


# --- PASO 4: Convertir GeoJSONs finales a Shapefiles ---
print("\n--- Conversión a Shapefiles (.shp) ---")

output_initial_shp_path = os.path.join(output_directory, f"{filename_base}_initial_zones.shp")
output_safe_shp_path = os.path.join(output_directory, f"{filename_base}_safe_zones.shp")
output_roads_shp_path = os.path.join(output_directory, f"{filename_base}_roads.shp")


# Función para convertir GeoJSON a Shapefile usando geopandas
def convert_geojson_to_shp(geojson_path, shp_path, driver_name='ESRI Shapefile'):
    if not os.path.exists(geojson_path) or os.stat(geojson_path).st_size == 0:
        print(f"  ADVERTENCIA: No se puede convertir '{geojson_path}' a SHP, el archivo no existe o está vacío.")
        return
    try:
        gdf = gpd.read_file(geojson_path)
        if not gdf.empty:
            gdf.to_file(shp_path, driver=driver_name)
            print(f"  Convertido: {geojson_path} -> {shp_path}")
        else:
            print(f"  ADVERTENCIA: GeoJSON '{geojson_path}' está vacío, no se generará SHP.")
    except Exception as e:
        print(f"  ERROR al convertir {geojson_path} a SHP: {e}")

convert_geojson_to_shp(output_initial_geojson_path, output_initial_shp_path)
convert_geojson_to_shp(output_safe_geojson_path, output_safe_shp_path)
convert_geojson_to_shp(output_roads_geojson_path, output_roads_shp_path)

# --- PASO 5: Limpieza de archivos temporales ---
print("\n--- Limpieza de Archivos Temporales ---")
temp_files = [
    temp_osm_path_windows,
    temp_pbf_path_windows,
    temp_osm_for_filter_path_windows,
    output_roads_osm_xml_filtered_path_windows
]
for f in temp_files:
    if os.path.exists(f):
        try:
            os.remove(f)
            print(f"  Eliminado: {f}")
        except OSError as e:
            print(f"  ERROR al eliminar archivo temporal {f}: {e}")
    else:
        print(f"  Temporal no encontrado (ya eliminado o no creado): {f}")

print("\n--- Proceso de Preparación Geoespacial Completado ---")
print("Archivos Shapefile listos en la carpeta de salida:")
print(f"  - {filename_base}_initial_zones.shp")
print(f"  - {filename_base}_safe_zones.shp")
print(f"  - {filename_base}_roads.shp")
