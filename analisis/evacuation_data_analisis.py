import pandas as pd
import numpy as np
import argparse
import os

import matplotlib.pyplot as plt
from matplotlib.ticker import MaxNLocator, PercentFormatter

# Configuración global de matplotlib para mejor apariencia
plt.rcParams['font.family'] = 'sans-serif'
plt.rcParams['font.sans-serif'] = ['Arial', 'Helvetica', 'DejaVu Sans']
plt.rcParams['font.size'] = 14
plt.rcParams['axes.linewidth'] = 1.4
plt.rcParams['grid.linewidth'] = 1.1
plt.rcParams['lines.linewidth'] = 2.8

color_map = {
    "evacuation_data_escenario100000000_procesado.csv": "#1f77b4",  # Azul (C0)
    "evacuation_data_escenario000100000_procesado.csv": "#ff7f0e",  # Naranja (C1)
    "evacuation_data_escenario000000100_procesado.csv": "#2ca02c",  # Verde (C2)
    "evacuation_data_escenario050000050_procesado.csv": "#d62728",  # Rojo (C3)
}

linestyle_map = {
    "evacuation_data_escenario100000000_procesado.csv": "-",
    "evacuation_data_escenario000100000_procesado.csv": "--",
    "evacuation_data_escenario000000100_procesado.csv": "-.",
    "evacuation_data_escenario050000050_procesado.csv": ":",
}

nombres_personalizados = {
    "evacuation_data_escenario100000000_procesado.csv": "Escenario 1",
    "evacuation_data_escenario000100000_procesado.csv": "Escenario 2",
    "evacuation_data_escenario000000100_procesado.csv": "Escenario 3",
    "evacuation_data_escenario050000050_procesado.csv": "Escenario 4",
}

TOTAL_AGENTES = 300
TIMESTAMP_INTERVAL = 100


def agents_mean(df, columnas_experimentos):
    return df[columnas_experimentos].mean(axis=1)


def agents_std(df, columnas_experimentos):
    return df[columnas_experimentos].std(axis=1)


def csv_clean(archivo_csv):
    df = pd.read_csv(archivo_csv, sep=';')
    columnas_experimentos = [col for col in df.columns if col.startswith('evacuated_count')]
    return df, columnas_experimentos


def filtrar_timestamps(df, intervalo=TIMESTAMP_INTERVAL):
    df_filtrado = df[df['time_seconds'] % intervalo == 0].copy()
    return df_filtrado


def plot_csv(df, archivo, output_dir='graficos'):
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    df['time_seconds'] = pd.to_numeric(df['time_seconds'], errors='coerce')
    df['average'] = pd.to_numeric(df['average'], errors='coerce')
    df['std'] = pd.to_numeric(df['std'], errors='coerce') if 'std' in df.columns else 0
    df = df.dropna(subset=['time_seconds', 'average'])

    if len(df) == 0:
        print(f"Advertencia: No hay datos válidos para graficar en {archivo}")
        return

    df['average_pct'] = (df['average'] / TOTAL_AGENTES) * 100
    df['std_pct'] = (df['std'] / TOTAL_AGENTES) * 100
    df_filtrado = filtrar_timestamps(df)

    intervalo_x = 1000
    x_max = 12000

    # Figura más compacta
    fig, ax = plt.subplots(figsize=(12, 8))
    base = os.path.basename(archivo)
    color = color_map.get(base, '#1f77b4')
    linestyle = linestyle_map.get(base, '-')
    nombre = nombres_personalizados.get(base, base)

    # Banda de ± std
    if 'std' in df_filtrado.columns:
        ax.fill_between(
            df_filtrado['time_seconds'],
            df_filtrado['average_pct'] - df_filtrado['std_pct'],
            df_filtrado['average_pct'] + df_filtrado['std_pct'],
            color=color,
            alpha=0.2
        )

    # Línea principal más gruesa
    ax.plot(df_filtrado['time_seconds'], df_filtrado['average_pct'],
            linewidth=2.5, color=color, linestyle=linestyle,
            label=nombre, marker='', markersize=0)

    # Etiqueta más compacta
    if len(df_filtrado) > 0:
        ultimo_x = df_filtrado['time_seconds'].iloc[-1]
        ultimo_y = df_filtrado['average_pct'].iloc[-1]
        offset_x = -500
        offset_y = -3

        ax.text(ultimo_x + offset_x, ultimo_y + offset_y, nombre,
                fontsize=9, fontweight='bold',
                color='white',
                verticalalignment='center',
                bbox=dict(boxstyle='round,pad=0.3', facecolor=color,
                          edgecolor='none', alpha=0.9))

    # Configurar ejes con mejor espaciado
    ax.set_xlim(-200, x_max)
    ax.set_ylim(-2, 102)
    ax.set_xlabel('Tiempo (segundos)', fontsize=16, fontweight='bold')
    ax.set_ylabel('Evacuados (%)', fontsize=16, fontweight='bold')
    ax.set_title('Progreso de Evacuación', fontsize=18, fontweight='bold', pad=15)
    ax.set_xticks(np.arange(0, x_max + intervalo_x, intervalo_x))
    ax.set_yticks([0, 25, 50, 75, 100])
    ax.tick_params(labelsize=16)

    ax.yaxis.set_major_formatter(PercentFormatter(decimals=0))
    ax.grid(True, alpha=0.25, linestyle='--')
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)

    # Leyenda mejorada
    legend_label = f'{nombre} (n={TOTAL_AGENTES} agentes)'
    ax.legend([legend_label], loc='lower right', frameon=True, fontsize=12,
              fancybox=True, shadow=True)

    plt.tight_layout()

    nombre_base = os.path.splitext(os.path.basename(archivo))[0]
    output_file = os.path.join(output_dir, f'{nombre_base}_grafico.png')
    plt.savefig(output_file, dpi=300, bbox_inches='tight', facecolor='white')
    plt.close()

    print(f"✓ Gráfico guardado: {output_file}")


def summary_plot(datos_archivos, output_dir='graficos', nombre_archivo='comparativo_evacuacion.png',
                 titulo='Comparación de Evacuación'):
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # Figura más compacta
    fig, ax = plt.subplots(figsize=(12, 7))
    intervalo_x = 1000
    x_max = 12000

    lineas_etiquetas = []

    for archivo, df in datos_archivos.items():
        df['time_seconds'] = pd.to_numeric(df['time_seconds'], errors='coerce')
        df['average'] = pd.to_numeric(df['average'], errors='coerce')
        df['std'] = pd.to_numeric(df['std'], errors='coerce') if 'std' in df.columns else 0
        df = df.dropna(subset=['time_seconds', 'average'])

        df['average_pct'] = (df['average'] / TOTAL_AGENTES) * 100
        df['std_pct'] = (df['std'] / TOTAL_AGENTES) * 100
        df_filtrado = filtrar_timestamps(df)

        base = os.path.basename(archivo)
        nombre = nombres_personalizados.get(base, base)
        color = color_map.get(base, None)
        linestyle = linestyle_map.get(base, '-')

        if 'std' in df_filtrado.columns:
            ax.fill_between(
                df_filtrado['time_seconds'],
                df_filtrado['average_pct'] - df_filtrado['std_pct'],
                df_filtrado['average_pct'] + df_filtrado['std_pct'],
                color=color,
                alpha=0.2
            )

        # Línea más gruesa
        ax.plot(df_filtrado['time_seconds'], df_filtrado['average_pct'],
                linewidth=2.5, color=color, linestyle=linestyle,
                label=nombre, marker='', markersize=0)

        if len(df_filtrado) > 0:
            ultimo_x = df_filtrado['time_seconds'].iloc[-1]
            ultimo_y = df_filtrado['average_pct'].iloc[-1]
            lineas_etiquetas.append((ultimo_x, ultimo_y, nombre, color))

    # Etiquetas más compactas
    offsets_base = [
        (-550, -1),
        (-550, -4),
        (-550, 3),
        (-550, -4),
    ]

    for i, (x, y, nombre, color_linea) in enumerate(lineas_etiquetas):
        offset_x, offset_y = offsets_base[i] if i < len(offsets_base) else (-550, -4)
        ax.text(x + offset_x, y + offset_y, nombre,
                fontsize=9, fontweight='bold',
                color='white',
                verticalalignment='center',
                bbox=dict(boxstyle='round,pad=0.3', facecolor=color_linea,
                          edgecolor='none', alpha=0.9))

    # Configurar ejes
    ax.set_xlim(-200, x_max)
    ax.set_ylim(-2, 102)
    ax.set_xlabel('Tiempo (segundos)', fontsize=16, fontweight='bold')
    ax.set_ylabel('Evacuados (%)', fontsize=16, fontweight='bold')
    ax.set_title(titulo, fontsize=18, fontweight='bold', pad=15)
    ax.set_xticks(np.arange(0, x_max + intervalo_x, intervalo_x))
    ax.set_yticks([0, 25, 50, 75, 100])
    ax.tick_params(labelsize=16)

    ax.yaxis.set_major_formatter(PercentFormatter(decimals=0))
    ax.grid(True, alpha=0.25, linestyle='--')
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)

    # Leyenda mejorada
    handles, labels = ax.get_legend_handles_labels()
    legend = ax.legend(handles, labels, loc='lower right', frameon=True, fontsize=14,
                       title=f'Total agentes: {TOTAL_AGENTES}', title_fontsize=14,
                       fancybox=True, shadow=True)

    plt.tight_layout()

    output_file = os.path.join(output_dir, nombre_archivo)
    plt.savefig(output_file, dpi=300, bbox_inches='tight', facecolor='white')
    plt.close()

    print(f"✓ Gráfico comparativo guardado: {output_file}")


def extraer_tiempos_evacuacion(archivo_csv):
    df = pd.read_csv(archivo_csv, sep=';', decimal=',')
    columnas_experimentos = [col for col in df.columns if col.startswith('evacuated_count')]
    tiempos_evacuacion = []

    for col in columnas_experimentos:
        df[col] = pd.to_numeric(df[col], errors='coerce')
        for agente_num in range(1, TOTAL_AGENTES + 1):
            mask = df[col] >= agente_num
            if mask.any():
                tiempo_evacuacion = df.loc[mask, 'time_seconds'].iloc[0]
                tiempos_evacuacion.append(tiempo_evacuacion)

    return tiempos_evacuacion


def crear_boxplot_escenario(archivo, output_dir='graficos'):
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    base = os.path.basename(archivo)
    nombre = nombres_personalizados.get(base, base)
    color = color_map.get(base, '#1f77b4')

    tiempos = extraer_tiempos_evacuacion(archivo)

    if len(tiempos) == 0:
        print(f"Advertencia: No hay datos de evacuación en {archivo}")
        return

    # Figura más compacta con mejor ratio
    fig, ax = plt.subplots(figsize=(5, 7))

    # Boxplot más angosto
    bp = ax.boxplot([tiempos], vert=True, patch_artist=True, widths=0.4,
                    boxprops=dict(facecolor=color, alpha=0.8, linewidth=1.5),
                    medianprops=dict(color='black', linewidth=3),
                    whiskerprops=dict(linewidth=1.8, color='#333'),
                    capprops=dict(linewidth=1.8, color='#333'),
                    flierprops=dict(marker='o', markerfacecolor=color,
                                    markersize=5, alpha=0.6, markeredgecolor='none'))

    # Estadísticas
    q1 = np.percentile(tiempos, 25)
    mediana = np.percentile(tiempos, 50)
    q3 = np.percentile(tiempos, 75)
    minimo = np.min(tiempos)
    maximo = np.max(tiempos)
    promedio = np.mean(tiempos)

    # Configurar ejes con mejor espaciado
    ax.set_ylabel('Tiempo de evacuación (s)', fontsize=16, fontweight='bold')
    ax.set_xlabel(nombre, fontsize=16, fontweight='bold')
    ax.set_title(f'Distribución de Tiempos\n{nombre}',
                 fontsize=18, fontweight='bold', pad=15)
    ax.set_xticks([1])
    ax.set_xticklabels([''])
    ax.set_ylim(0, max(tiempos) * 1.05)
    ax.tick_params(labelsize=16)
    ax.grid(True, alpha=0.25, axis='y', linestyle='--')
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)

    # Texto de estadísticas más compacto y legible
    stats_text = (
        f'Min: {minimo:.0f}s\n'
        f'Q1: {q1:.0f}s\n'
        f'Mediana: {mediana:.0f}s\n'
        f'Media: {promedio:.0f}s\n'
        f'Q3: {q3:.0f}s\n'
        f'Max: {maximo:.0f}s\n'
        f'N = {len(tiempos)}'
    )

    ax.text(0.97, 0.97, stats_text,
            transform=ax.transAxes,
            fontsize=11,
            verticalalignment='top',
            horizontalalignment='right',
            bbox=dict(boxstyle='round,pad=0.6', facecolor='white',
                      alpha=0.95, edgecolor='gray', linewidth=1.2))

    plt.tight_layout()

    nombre_base = os.path.splitext(os.path.basename(archivo))[0]
    output_file = os.path.join(output_dir, f'{nombre_base}_boxplot.png')
    plt.savefig(output_file, dpi=300, bbox_inches='tight', facecolor='white')
    plt.close()

    print(f"✓ Boxplot guardado: {output_file}")
    print(f"  - Mediana: {mediana:.1f}s | Promedio: {promedio:.1f}s | Rango: [{minimo:.0f}s - {maximo:.0f}s]")


def crear_boxplot_comparativo(datos_archivos, output_dir='graficos',
                              nombre_archivo='boxplot_comparativo.png',
                              titulo='Comparación de Tiempos de Evacuación'):
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # Figura con mejor proporción
    fig, ax = plt.subplots(figsize=(10, 7))

    datos_boxplot = []
    etiquetas = []
    colores = []

    for archivo in datos_archivos:
        base = os.path.basename(archivo)
        nombre = nombres_personalizados.get(base, base)
        color = color_map.get(base, '#2E86AB')

        tiempos = extraer_tiempos_evacuacion(archivo)

        if len(tiempos) > 0:
            datos_boxplot.append(tiempos)
            etiquetas.append(nombre)
            colores.append(color)

    if len(datos_boxplot) == 0:
        print("Advertencia: No hay datos para el boxplot comparativo")
        return

    # Boxplots más angostos
    bp = ax.boxplot(datos_boxplot, vert=True, patch_artist=True, widths=0.5,
                    labels=etiquetas,
                    boxprops=dict(alpha=0.8, linewidth=1.5),
                    medianprops=dict(color='black', linewidth=3),
                    whiskerprops=dict(linewidth=1.8, color='#333'),
                    capprops=dict(linewidth=1.8, color='#333'),
                    flierprops=dict(marker='o', markersize=5,
                                    alpha=0.6, markeredgecolor='none'))

    # Aplicar colores
    for patch, color in zip(bp['boxes'], colores):
        patch.set_facecolor(color)

    for flier, color in zip(bp['fliers'], colores):
        flier.set_markerfacecolor(color)

    # Configurar ejes
    ax.set_ylabel('Tiempo de evacuación (s)', fontsize=16, fontweight='bold')
    ax.set_xlabel('Escenarios', fontsize=16, fontweight='bold')
    ax.set_title(titulo, fontsize=18, fontweight='bold', pad=15)
    ax.tick_params(labelsize=17)
    ax.grid(True, alpha=0.25, axis='y', linestyle='--')
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)

    plt.xticks(rotation=0)
    plt.tight_layout()

    output_file = os.path.join(output_dir, nombre_archivo)
    plt.savefig(output_file, dpi=300, bbox_inches='tight', facecolor='white')
    plt.close()

    print(f"✓ Boxplot comparativo guardado: {output_file}")


def main():
    parser = argparse.ArgumentParser(description='Calcular promedio y desviación estándar de evacuaciones')
    parser.add_argument('archivos', nargs='+', help='Archivos CSV a procesar')
    parser.add_argument('--boxplot', action='store_true', help='Generar también boxplots de tiempos de evacuación')
    args = parser.parse_args()

    output_dir = 'graficos'
    datos_archivos = {}

    print("=== Procesando archivos de evacuación ===\n")

    archivos_procesados = []
    for archivo in args.archivos:
        try:
            ruta_completa = os.path.abspath(archivo)
            print(f"Procesando archivo: {ruta_completa}")

            df, columnas_experimentos = csv_clean(archivo)
            df['average'] = agents_mean(df, columnas_experimentos)
            df['std'] = agents_std(df, columnas_experimentos).round(6)

            nombre_base = os.path.splitext(archivo)[0]
            extension = os.path.splitext(archivo)[1]
            nuevo_archivo = f"{nombre_base}_procesado{extension}"
            df.to_csv(nuevo_archivo, index=False, sep=';', decimal=',')

            archivos_procesados.append(nuevo_archivo)
            print(f"✓ Archivo procesado guardado: {nuevo_archivo}")

        except FileNotFoundError:
            print(f"✗ Error: No se encontró el archivo {archivo}")
        except Exception as e:
            print(f"✗ Error procesando {archivo}: {str(e)}")

    print("\n=== Generando gráficos ===\n")

    for archivo in archivos_procesados:
        try:
            print(f"Graficando: {archivo}")
            df = pd.read_csv(archivo, sep=';', decimal=',')
            df['time_seconds'] = pd.to_numeric(df['time_seconds'], errors='coerce')
            df['average'] = pd.to_numeric(df['average'], errors='coerce')
            df = df.dropna(subset=['time_seconds', 'average'])

            plot_csv(df, archivo, output_dir)
            datos_archivos[archivo] = df

        except FileNotFoundError:
            print(f"✗ Error: No se encontró el archivo {archivo}")
        except Exception as e:
            print(f"✗ Error procesando {archivo}: {str(e)}")

    print("\n=== Creando gráficos comparativos ===")

    escenarios = {}
    for archivo in datos_archivos.keys():
        base = os.path.basename(archivo)
        if "escenario100000000" in base:
            escenarios[1] = archivo
        elif "escenario000100000" in base:
            escenarios[2] = archivo
        elif "escenario000000100" in base:
            escenarios[3] = archivo
        elif "escenario050000050" in base:
            escenarios[4] = archivo

    if all(e in escenarios for e in [1, 2, 3]):
        datos_123 = {
            escenarios[1]: datos_archivos[escenarios[1]],
            escenarios[2]: datos_archivos[escenarios[2]],
            escenarios[3]: datos_archivos[escenarios[3]]
        }
        summary_plot(datos_123, output_dir,
                     nombre_archivo='comparativo_escenarios_1_2_3.png',
                     titulo='Comparación: Escenarios 1, 2 y 3')

    if all(e in escenarios for e in [1, 3, 4]):
        datos_134 = {
            escenarios[1]: datos_archivos[escenarios[1]],
            escenarios[3]: datos_archivos[escenarios[3]],
            escenarios[4]: datos_archivos[escenarios[4]]
        }
        summary_plot(datos_134, output_dir,
                     nombre_archivo='comparativo_escenarios_1_3_4.png',
                     titulo='Comparación: Escenarios 1, 3 y 4')

    if len(datos_archivos) > 1:
        summary_plot(datos_archivos, output_dir,
                     nombre_archivo='comparativo_todos_escenarios.png',
                     titulo='Comparación: Todos los Escenarios')

    if args.boxplot:
        print("\n=== Generando boxplots de evacuación ===\n")

        archivos_validos = []
        for archivo in archivos_procesados:
            try:
                print(f"Procesando boxplot: {archivo}")
                crear_boxplot_escenario(archivo, output_dir)
                archivos_validos.append(archivo)
            except Exception as e:
                print(f"✗ Error creando boxplot para {archivo}: {str(e)}")

        print("\n=== Creando boxplots comparativos ===")

        if len(archivos_validos) > 1:
            if all(e in escenarios for e in [1, 2, 3]):
                crear_boxplot_comparativo([escenarios[1], escenarios[2], escenarios[3]],
                                          output_dir,
                                          nombre_archivo='boxplot_escenarios_1_2_3.png',
                                          titulo='Comparación de Tiempos: Escenarios 1, 2 y 3')

            if all(e in escenarios for e in [1, 3, 4]):
                crear_boxplot_comparativo([escenarios[1], escenarios[3], escenarios[4]],
                                          output_dir,
                                          nombre_archivo='boxplot_escenarios_1_3_4.png',
                                          titulo='Comparación de Tiempos: Escenarios 1, 3 y 4')

            crear_boxplot_comparativo(archivos_validos, output_dir,
                                      nombre_archivo='boxplot_todos_escenarios.png',
                                      titulo='Comparación de Tiempos: Todos los Escenarios')

    print("\n=== Proceso completado ===")


if __name__ == "__main__":
    main()