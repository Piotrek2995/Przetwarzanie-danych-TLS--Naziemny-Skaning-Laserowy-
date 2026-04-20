# Przetwarzanie TLS - projekt

Skrypt do przetwarzania chmur punktow z naziemnego skaningu laserowego (TLS). Wczytuje stanowiska, robi preprocessing, rejestruje je na targetach i sklada w jedna chmure. Opcjonalnie ICP + triangulacja.

## Co robi

### Ocena 3.0
1. Rozrzedzanie chmury (voxel downsampling, domyslnie 5 cm)
2. Filtracja - najpierw ROR potem SOR
3. Liczenie normalnych (KD-Tree, orientacja w strone skanera czyli 0,0,0)
4. Rejestracja na targetach (wspolne punkty wiazace z plikow txt)
5. Scalenie stanowisk w jedna chmure, zapis do PLY i LAS

### Ocena 4.0
6. ICP (point-to-plane) po rejestracji targetami
7. Triangulacja per stanowisko - BPA albo Poisson
8. Zapis meshy do tego samego folderu

## Co trzeba miec

- Python 3.8+
- biblioteki: `open3d`, `laspy`, `numpy`

```bash
pip install open3d laspy numpy
```

## Format plikow targetow

Pliki txt musza sie nazywac tak samo jak stanowiska (bez rozszerzenia). Format w srodku:

```
id x y z
```

Linie z `#` na poczatku sa pomijane, mozna tam wpisywac opisy.

Przyklad (`targets/Zewnatrz-10.txt`):
```
# Punkty wiazace dla Zewnatrz-10
1 19.910000 2.450000 -3.110000
2 22.580000 -1.280000 -3.020000
3 25.520000 -2.790000 -3.040000
```

## Jak uruchomic

Wersja podstawowa (oc. 3.0):
```bash
python tls_process.py --input . --output ./wynik --targets ./targets --reference Zewnatrz-10 --voxel 0.05
```

Z ICP i BPA (oc. 4.0):
```bash
python tls_process.py --input . --output ./wynik --targets ./targets --reference Zewnatrz-10 --voxel 0.05 --icp --mesh bpa
```

Z ICP i Poissonem:
```bash
python tls_process.py --input . --output ./wynik --targets ./targets --reference Zewnatrz-10 --voxel 0.05 --icp --mesh poisson
```

## Argumenty

- `--input` - katalog ze stanowiskami (.las/.laz/.txt), domyslnie `.`
- `--output` - katalog na wyniki (wymagany)
- `--targets` - katalog z plikami targetow, domyslnie `./targets`
- `--reference` - nazwa stanowiska referencyjnego (bez rozszerzenia). Jak nie podam to bierze pierwsze alfabetycznie.
- `--voxel` - rozmiar voxela w metrach, domyslnie 0.05 (sensownie 0.05-0.10)
- `--stations` - opcjonalnie lista stanowisk do przetworzenia
- `--icp` - wlacza ICP po rejestracji targetami
- `--icp-threshold` - prog odleglosci dla ICP, domyslnie 0.2 m
- `--mesh` - `none`, `bpa` albo `poisson`, domyslnie `none`

## Co wypluwa

W folderze z `--output`:
- `merged_cloud.ply` i `merged_cloud.las` - scalona chmura
- `mesh_<nazwa>.ply` - meshe per stanowisko (tylko jak `--mesh` != none)

## Jak to dziala w skrocie

1. Wczytuje kazde stanowisko (laspy dla las/laz, numpy.loadtxt dla txt)
2. Voxel downsampling do zadanej rozdzielczosci
3. ROR - wywala punkty ktore maja <6 sasiadow w promieniu 3x voxel
4. SOR - wywala outliery statystyczne (20 sasiadow, std_ratio=2.0)
5. Liczy normalne i orientuje je w strone 0,0,0 (tam gdzie byl skaner)
6. Na targetach liczy transformacje do ukladu referencyjnego (trzeba min. 3 wspolnych targetow), do tego RMSE na targetach zeby bylo wiadomo jak wyszlo
7. Opcjonalnie ICP (max 50 iteracji) dla dokladniejszego dopasowania
8. Sklada wszystko razem, kazde stanowisko na inny kolor
9. Opcjonalnie triangulacja i czyszczenie mesha (duplikaty, zdegenerowane trojkaty, non-manifold)
10. Na koncu probuje otworzyc podglad w Open3D (jak jest wyswietlacz)

## Uwagi

- Dla BPA testuje kilka promieni (1.5, 2.0, 3.0, 4.0 x voxel)
- Dla Poissona uzywam depth=9, na koncu wycinam 5% trojkatow o najnizszej gestosci zeby pozbyc sie artefaktow na brzegach
- Jak na ktoryms stanowisku brakuje pliku targetow to skrypt sie wywala
- Kolory sa hardcoded (6 kolorow, potem sie powtarzaja)