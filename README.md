# Przetwarzanie danych TLS (Naziemny Skaning Laserowy)

Narzędzie realizujące cały proces przetwarzania danych pozyskanych techniką Naziemnego Skaningu Laserowego (TLS) — od surowych stanowisk w formacie LAS do scalonej, przefiltrowanej chmury punktów oraz modeli mesh.

## Funkcjonalności

### Ocena 3.0

1. **Rozrzedzenie chmury punktów** — Voxel downsampling do zadanej rozdzielczości (domyślnie 5 cm).
2. **Filtracja** — Radius Outlier Removal (ROR) + Statistical Outlier Removal (SOR).
3. **Estymacja normalnych** — KD-Tree Hybrid Search z orientacją w stronę skanera (punkt 0,0,0).
4. **Rejestracja Target-Based** — Wzajemna orientacja stanowisk na podstawie punktów wiążących wczytanych z plików TXT.
5. **Scalenie** — Połączenie stanowisk w jedną chmurę i zapis do PLY + LAS.

### Ocena 4.0

6. **ICP** — Dodatkowa poprawa orientacji algorytmem ICP (Point-to-Plane) po rejestracji Target-Based.
7. **Triangulacja** — Surface reconstruction per stanowisko metodą Ball Pivoting (BPA) lub Poisson.
8. **Zapis mesh** — Modele mesh zapisywane do tego samego folderu wyjściowego co chmura punktów.

## Wymagania

- Python 3.8+
- Biblioteki:
  ```
  open3d
  laspy
  numpy
  ```

### Instalacja zależności

```bash
pip install open3d laspy numpy
```

### Format plików targets

Pliki TXT z punktami wiążącymi muszą mieć nazwy identyczne z nazwami stanowisk (bez rozszerzenia `.las`). Format:

```
# Komentarze zaczynające się od #
id x y z
```

Przykład (`targets/Zewnatrz-10.txt`):
```
# Punkty wiazace dla stanowiska Zewnatrz-10
# Format: id x y z
1 19.910000 2.450000 -3.110000
2 22.580000 -1.280000 -3.020000
3 25.520000 -2.790000 -3.040000
```

## Uruchomienie

### Ocena 3.0 (podstawowa rejestracja Target-Based)

```bash
python tls_process.py --input . --output ./wynik --targets ./targets --reference Zewnatrz-10 --voxel 0.05
```

### Ocena 4.0 (z ICP + triangulacją BPA)

```bash
python tls_process.py --input . --output ./wynik --targets ./targets --reference Zewnatrz-10 --voxel 0.05 --icp --mesh bpa
```

### Ocena 4.0 (z ICP + triangulacją Poisson)

```bash
python tls_process.py --input . --output ./wynik --targets ./targets --reference Zewnatrz-10 --voxel 0.05 --icp --mesh poisson
```

## Parametry

| Parametr          | Domyślna wartość | Opis                                                                 |
|-------------------|------------------|----------------------------------------------------------------------|
| `--input`         | `.`              | Katalog ze stanowiskami (`.las`, `.laz`, `.txt`)                     |
| `--output`        | *(wymagany)*     | Katalog zapisu wyników                                               |
| `--targets`       | `./targets`      | Katalog z plikami TXT punktów wiążących                              |
| `--reference`     | pierwsze alfabet. | Nazwa stanowiska referencyjnego (bez rozszerzenia)                  |
| `--voxel`         | `0.05`           | Rozmiar voxela w metrach (zakres 0.05–0.10 zalecany)                |
| `--stations`      | automatycznie    | Lista nazw stanowisk do przetworzenia (bez rozszerzeń)              |
| `--icp`           | wyłączony        | Włącza poprawę orientacji algorytmem ICP po Target-Based            |
| `--icp-threshold` | `0.2`            | Maksymalna odległość korespondencji w ICP [m]                       |
| `--mesh`          | `none`           | Triangulacja: `none`, `bpa` (Ball Pivoting) lub `poisson` (Poisson) |

## Pliki wyjściowe

Po uruchomieniu w folderze `--output` pojawią się:

- `merged_cloud.ply` — Scalona chmura punktów (PLY)
- `merged_cloud.las` — Scalona chmura punktów (LAS)
- `mesh_<nazwa_stanowiska>.ply` — Modele mesh per stanowisko (jeśli `--mesh` != `none`)

## Opis algorytmu

1. **Wczytanie** — Każde stanowisko jest wczytywane z pliku LAS (via `laspy`).
2. **Downsampling** — Redukcja gęstości chmury metodą voxel grid.
3. **Filtracja ROR** — Usunięcie punktów izolowanych (< 6 sąsiadów w promieniu 3×voxel).
4. **Filtracja SOR** — Usunięcie outlierów statystycznych (20 sąsiadów, 2σ).
5. **Normalne** — Estymacja i orientacja normalnych w stronę pozycji skanera (0,0,0).
6. **Target-Based** — Transformacja stanowisk do układu referencyjnego na podstawie wspólnych punktów wiążących (min. 3 punkty).
7. **ICP** *(opcjonalnie)* — Drobna korekta orientacji algorytmem Point-to-Plane ICP (max 50 iteracji).
8. **Scalenie** — Połączenie stanowisk z kolorowaniem per stanowisko.
9. **Triangulacja** *(opcjonalnie)* — Rekonstrukcja powierzchni BPA lub Poisson per stanowisko z czyszczeniem mesh.
10. **Wizualizacja** — Automatyczne otwarcie podglądu scalonej chmury w Open3D (jeśli dostępny wyświetlacz).
