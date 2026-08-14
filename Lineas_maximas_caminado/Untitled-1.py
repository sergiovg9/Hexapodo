
import pandas as pd

# ==========================
# Configuración
# ==========================
CSV_ENTRADA = "LineasMaximas.csv"
ARCHIVO_SALIDA = "SalidaWorkspace.cpp"

# ==========================
# Leer CSV
# ==========================
df = pd.read_csv(CSV_ENTRADA, header=None)

df.columns = [
    "altura",
    "angulo",
    "xi",
    "yi",
    "xf",
    "yf",
    "longitud"
]

# Ordenar
df = df.sort_values(["altura", "angulo"]).reset_index(drop=True)

# Obtener alturas y ángulos
alturas = sorted(df["altura"].unique())
angulos = sorted(df["angulo"].unique())

# ==========================
# Crear archivo .h
# ==========================
with open(ARCHIVO_SALIDA, "w") as f:

    f.write("#include \"WorkspaceData.h\"\n\n")

    f.write(f"const int NUM_ALTURAS = {len(alturas)};\n")
    f.write(f"const int NUM_ANGULOS = {len(angulos)};\n\n")

    f.write("const float alturas[NUM_ALTURAS] = {\n")
    f.write("    ")
    f.write(", ".join(f"{h:.3f}f" for h in alturas))
    f.write("\n};\n\n")

    f.write("const float angulos[NUM_ANGULOS] = {\n")
    f.write("    ")
    f.write(", ".join(f"{a:.3f}f" for a in angulos))
    f.write("\n};\n\n")

    f.write("const LineaTrabajo workspace[NUM_ALTURAS][NUM_ANGULOS] = {\n")

    for altura in alturas:

        f.write("    {\n")

        datos_altura = (
            df[df["altura"] == altura]
            .sort_values("angulo")
            .set_index("angulo")
        )

        for angulo in angulos:

            if angulo in datos_altura.index:
                r = datos_altura.loc[angulo]

                f.write(
                    "        "
                    "{%.6ff, %.6ff, %.6ff, %.6ff, %.6ff},\n"
                    % (
                        r["xi"],
                        r["yi"],
                        r["xf"],
                        r["yf"],
                        r["longitud"],
                    )
                )
            else:
                f.write(
                    "        {0.0f, 0.0f, 0.0f, 0.0f, 0.0f},\n"
                )

        f.write("    },\n")

    f.write("};\n")

print(f"Archivo generado: {ARCHIVO_SALIDA}")