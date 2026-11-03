"""
Resume los resultados de un entrenamiento de ML-Agents en una tabla de texto corta,
para poder pegarla en un chat en lugar de hacer capturas de TensorBoard.

Uso (desde la carpeta del proyecto, con el venv activado):
    python resumen_tb.py results\\Fase5_Obstaculos
    python resumen_tb.py results\\Fase5_Obstaculos --bin 500000
    python resumen_tb.py results\\Fase5_Obstaculos --csv resumen.csv
    python resumen_tb.py results\\Fase5_Obstaculos --tags Fase5/Choque Fase5/Timeout
"""
import argparse
import csv
from collections import defaultdict
from pathlib import Path

from tensorboard.backend.event_processing.event_accumulator import EventAccumulator

DEFAULT_TAGS = [
    "Environment/Cumulative Reward",
    "Environment/Episode Length",
    "Fase5/LlegadaMeta",
    "Fase5/Choque",
    "Fase5/Timeout",
    "Fase5/NumPalets",
]


def find_event_dirs(root: Path):
    return sorted({p.parent for p in root.rglob("events.out.tfevents*")})


def read_scalars(root: Path, tags):
    data = defaultdict(list)  # tag -> [(step, value)]
    for d in find_event_dirs(root):
        acc = EventAccumulator(str(d), size_guidance={"scalars": 0})
        acc.Reload()
        available = set(acc.Tags().get("scalars", []))
        for tag in tags:
            if tag in available:
                data[tag].extend((e.step, e.value) for e in acc.Scalars(tag))
    for tag in data:
        data[tag].sort()
    return data


def bin_means(points, bin_size):
    bins = defaultdict(list)
    for step, value in points:
        bins[step // bin_size].append(value)
    return {(b + 1) * bin_size: sum(v) / len(v) for b, v in bins.items()}


def main():
    parser = argparse.ArgumentParser(description="Resumen compacto de un run de ML-Agents")
    parser.add_argument("run_dir", help="Carpeta del run, por ejemplo results\\Fase5_Obstaculos")
    parser.add_argument("--bin", type=int, default=250000, help="Pasos por fila (por defecto 250000)")
    parser.add_argument("--tags", nargs="+", default=DEFAULT_TAGS, help="Gráficas a mostrar")
    parser.add_argument("--csv", help="Guardar también la tabla en este archivo CSV")
    args = parser.parse_args()

    root = Path(args.run_dir)
    if not root.exists():
        print(f"No existe la carpeta: {root}")
        return

    data = read_scalars(root, args.tags)
    if not data:
        print("No se encontraron datos de TensorBoard en esa carpeta (¿ruta correcta?).")
        return

    tags = [t for t in args.tags if t in data]
    missing = [t for t in args.tags if t not in data]
    binned = {t: bin_means(data[t], args.bin) for t in tags}
    rows = sorted({step for t in tags for step in binned[t]})

    names = [t.split("/")[-1] for t in tags]
    header = ["pasos"] + names
    table = []
    for step in rows:
        line = [f"{step / 1e6:.2f}M"]
        for t in tags:
            v = binned[t].get(step)
            line.append("-" if v is None else f"{v:.3f}")
        table.append(line)

    widths = [max(len(str(r[i])) for r in [header] + table) for i in range(len(header))]
    print(f"Run: {root}  (media por tramos de {args.bin:,} pasos)")
    print("  ".join(h.rjust(w) for h, w in zip(header, widths)))
    for r in table:
        print("  ".join(c.rjust(w) for c, w in zip(r, widths)))
    if missing:
        print("Sin datos para: " + ", ".join(missing))

    if args.csv:
        with open(args.csv, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(header)
            writer.writerows(table)
        print(f"CSV guardado en {args.csv}")


if __name__ == "__main__":
    main()