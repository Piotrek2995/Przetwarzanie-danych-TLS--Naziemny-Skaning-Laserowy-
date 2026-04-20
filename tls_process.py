
import argparse
import os
import glob
import numpy as np
import laspy
import open3d as o3d


#wczytywanie danych

def wczytaj_chmure(sciezka):
    """Wczytuje chmure punktow z pliku LAS/LAZ albo TXT."""
    rozszerzenie = os.path.splitext(sciezka)[1].lower()

    if rozszerzenie in (".las", ".laz"):
        dane_las = laspy.read(sciezka)
        punkty = np.asarray(dane_las.xyz)
    elif rozszerzenie == ".txt":
        # txt - zakladam ze kolumny to x y z (moze byc wiecej, bierzemy 3 pierwsze)
        punkty = np.loadtxt(sciezka, comments="#")[:, :3]
    else:
        raise ValueError(f"Nie wiem co zrobic z formatem: {sciezka}")

    chmura = o3d.geometry.PointCloud()
    chmura.points = o3d.utility.Vector3dVector(punkty)
    return chmura


def wczytaj_targety(plik_txt):
    """
    Targety z pliku txt, format:
    id  x  y  z
    Linie zaczynajace sie od # sa pomijane, zrobilem tam opisy
    """
    wynik = {}
    with open(plik_txt, "r") as f:
        for linia in f:
            linia = linia.strip()
            if not linia or linia.startswith("#"):
                continue
            czesci = linia.split()
            if len(czesci) < 4:
                continue  # pomijamy dziwne linie
            nr = int(float(czesci[0]))
            wspolrzedne = np.array([float(czesci[1]), float(czesci[2]), float(czesci[3])])
            wynik[nr] = wspolrzedne
    return wynik


#preprocessing

def rozrzedz(chmura, rozmiar_voxela):
    wynik = chmura.voxel_down_sample(voxel_size=rozmiar_voxela)
    print(f"  downsampling ({rozmiar_voxela*100:.0f}cm): {len(chmura.points)} -> {len(wynik.points)} pkt")
    return wynik


def filtruj(chmura, rozmiar_voxela):
    # usuwanie outlierow - najpierw ROR potem SOR
    ile_przed = len(chmura.points)

    # ROR - radius outlier removal
    chmura_ror, idx = chmura.remove_radius_outlier(nb_points=6,
                                                    radius=rozmiar_voxela * 3.0)
    print(f"  ROR: {ile_przed} -> {len(chmura_ror.points)} (usunieto {ile_przed - len(chmura_ror.points)})")

    ile_po_ror = len(chmura_ror.points)

    # SOR - statistical outlier removal
    chmura_czysta, idx2 = chmura_ror.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    print(f"  SOR: {ile_po_ror} -> {len(chmura_czysta.points)} (usunieto {ile_po_ror - len(chmura_czysta.points)})")

    return chmura_czysta


def oblicz_normalne(chmura, rozmiar_voxela):
    # normalne - promien szukania = 4x voxel, max 30 sasiadow
    chmura.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=rozmiar_voxela * 4.0,
            max_nn=30))
    # orientacja w strone skanera (zakladamy ze skaner jest w 0,0,0)
    chmura.orient_normals_towards_camera_location(
        camera_location=np.array([0.0, 0.0, 0.0]))
    return chmura


#rejestracja 

def rejestracja_targetami(tgt_src, tgt_ref):
    """
    Liczy transformacje zrodlowej chmury do referencyjnej
    na podstawie wspolnych targetow.
    Zwraca macierz 4x4.
    """
    wspolne_id = sorted(set(tgt_src.keys()) & set(tgt_ref.keys()))
    if len(wspolne_id) < 3:
        raise ValueError(f"Za malo wspolnych targetow! Mam {len(wspolne_id)}, a potrzeba min 3")

    # tworzymy chmury z samych targetow
    src_pts = o3d.geometry.PointCloud()
    ref_pts = o3d.geometry.PointCloud()
    src_pts.points = o3d.utility.Vector3dVector(
        np.array([tgt_src[i] for i in wspolne_id]))
    ref_pts.points = o3d.utility.Vector3dVector(
        np.array([tgt_ref[i] for i in wspolne_id]))

    # korespondencje 1:1
    koresp = o3d.utility.Vector2iVector([[i, i] for i in range(len(wspolne_id))])
    estymator = o3d.pipelines.registration.TransformationEstimationPointToPoint()
    macierz_T = estymator.compute_transformation(src_pts, ref_pts, koresp)

    # rmse na targetach (zeby sprawdzic jakosc)
    src_np = np.asarray(src_pts.points)
    ref_np = np.asarray(ref_pts.points)
    src_hom = np.hstack([src_np, np.ones((len(src_np), 1))])  # wspolrzedne jednorodne
    src_po_transf = (macierz_T @ src_hom.T).T[:, :3]
    bledy = np.sqrt(np.mean(np.sum((src_po_transf - ref_np)**2, axis=1)))
    print(f"  wspolne targety: {wspolne_id}")
    print(f"  RMSE na targetach: {bledy:.4f} m")

    return macierz_T


## ---------- zapis ----------

def zapisz_chmure(chmura, katalog_wyj, nazwa):
    os.makedirs(katalog_wyj, exist_ok=True)

    # ply
    sciezka_ply = os.path.join(katalog_wyj, f"{nazwa}.ply")
    o3d.io.write_point_cloud(sciezka_ply, chmura)
    print(f"  -> {sciezka_ply}")

    # las
    xyz = np.asarray(chmura.points)
    hdr = laspy.LasHeader(point_format=3, version="1.2")
    hdr.offsets = np.min(xyz, axis=0)
    hdr.scales = np.array([0.001, 0.001, 0.001])
    las_out = laspy.LasData(hdr)
    las_out.x = xyz[:, 0]
    las_out.y = xyz[:, 1]
    las_out.z = xyz[:, 2]
    sciezka_las = os.path.join(katalog_wyj, f"{nazwa}.las")
    las_out.write(sciezka_las)
    print(f"  -> {sciezka_las}")


def zapisz_mesh(mesh, katalog_wyj, nazwa):
    os.makedirs(katalog_wyj, exist_ok=True)
    sciezka = os.path.join(katalog_wyj, f"{nazwa}.ply")
    o3d.io.write_triangle_mesh(sciezka, mesh)
    print(f"  mesh zapisany: {sciezka}")


## ---------- triangulacja (mesh) ----------

def zrob_mesh(chmura, metoda, rozmiar_voxela):
    """Triangulacja chmury - BPA albo Poisson."""

    # upewniamy sie ze sa normalne
    if not chmura.has_normals():
        oblicz_normalne(chmura, rozmiar_voxela)

    if metoda == "bpa":
        # ball pivoting - testuje kilka promieni
        promienie = [rozmiar_voxela * m for m in (1.5, 2.0, 3.0, 4.0)]
        print(f"  BPA, promienie: {promienie}")
        mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
            chmura, o3d.utility.DoubleVector(promienie))

    elif metoda == "poisson":
        print("  Poisson depth=9...")
        mesh, gestosc = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
            chmura, depth=9)
        # usuwamy trojkaty o niskiej gestosci (artefakty na brzegach)
        gestosc_np = np.asarray(gestosc)
        prog = np.quantile(gestosc_np, 0.05)
        mesh.remove_vertices_by_mask(gestosc_np < prog)
    else:
        print(f"  nieznana metoda: {metoda}??")
        return None

    # czyszczenie mesha
    mesh.remove_duplicated_vertices()
    mesh.remove_duplicated_triangles()
    mesh.remove_degenerate_triangles()
    mesh.remove_non_manifold_edges()
    mesh.compute_vertex_normals()

    print(f"  wynik: {len(mesh.vertices)} wierzcholkow, {len(mesh.triangles)} trojkatow")
    return mesh


#szukanie plikow

def znajdz_stanowiska(katalog):
    pliki = []
    for roz in ("*.las", "*.laz", "*.txt"):
        pliki.extend(glob.glob(os.path.join(katalog, roz)))
    return sorted(pliki)


# MAIN

def main():
    parser = argparse.ArgumentParser(
        description="Przetwarzanie chmur punktow TLS - projekt")
    parser.add_argument("--input", default=".",
                        help="katalog z plikami stanowisk (.las/.laz/.txt)")
    parser.add_argument("--output", required=True,
                        help="katalog na wyniki")
    parser.add_argument("--targets", default="./targets",
                        help="katalog z plikami txt targetow")
    parser.add_argument("--reference", default=None,
                        help="nazwa stanowiska referencyjnego (bez rozszerzenia)")
    parser.add_argument("--voxel", type=float, default=0.05,
                        help="rozmiar voxela w metrach (def. 0.05)")
    parser.add_argument("--stations", nargs="+", default=None,
                        help="opcjonalnie: lista nazw stanowisk do przetworzenia")
    # argumenty do oc. 4.0
    parser.add_argument("--icp", action="store_true",
                        help="wlacz ICP po target-based")
    parser.add_argument("--icp-threshold", type=float, default=0.2,
                        help="prog odleglosci ICP (def. 0.2m)")
    parser.add_argument("--mesh", choices=["none", "bpa", "poisson"],
                        default="none",
                        help="metoda triangulacji (none/bpa/poisson)")
    args = parser.parse_args()

    # szukamy plikow stanowisk
    lista_plikow = znajdz_stanowiska(args.input)

    if args.stations:
        # filtruj do wybranych
        wybrane = set(args.stations)
        lista_plikow = [f for f in lista_plikow
                        if os.path.splitext(os.path.basename(f))[0] in wybrane]
    else:
        # bierzemy tylko te ktore maja plik z targetami
        tmp = []
        for p in lista_plikow:
            nazwa = os.path.splitext(os.path.basename(p))[0]
            plik_tgt = os.path.join(args.targets, nazwa + ".txt")
            if os.path.exists(plik_tgt):
                tmp.append(p)
        lista_plikow = tmp

    if len(lista_plikow) < 2:
        print(f"BLAD: za malo stanowisk ({len(lista_plikow)}), potrzeba >= 2")
        return

    print(f"Znalezione stanowiska: {[os.path.basename(f) for f in lista_plikow]}")

    # wczytywanie i preprocessing
    chmury = {}     # nazwa -> chmura po preprocessingu
    targety = {}    # nazwa -> dict targetow

    for sciezka in lista_plikow:
        nazwa = os.path.splitext(os.path.basename(sciezka))[0]
        print(f"\n--- {nazwa} ({sciezka}) ---")

        ch = wczytaj_chmure(sciezka)
        print(f"  wczytano {len(ch.points)} punktow")

        ch = rozrzedz(ch, args.voxel)
        ch = filtruj(ch, args.voxel)
        ch = oblicz_normalne(ch, args.voxel)
        chmury[nazwa] = ch

        # wczytaj targety
        plik_tgt = os.path.join(args.targets, nazwa + ".txt")
        if not os.path.exists(plik_tgt):
            print(f"BLAD: brak pliku targetow {plik_tgt}")
            return
        targety[nazwa] = wczytaj_targety(plik_tgt)
        print(f"  targety: {len(targety[nazwa])} pkt z {plik_tgt}")

    # rejestracja
    nazwa_ref = args.reference if args.reference else sorted(chmury.keys())[0]
    if nazwa_ref not in chmury:
        print(f"BLAD: stanowisko ref '{nazwa_ref}' nie istnieje!")
        return

    print(f"\nStanowisko referencyjne: {nazwa_ref}")

    przetworzone = {}  # po transformacji
    ref_chmura = chmury[nazwa_ref]

    for nazwa, ch in chmury.items():
        if nazwa == nazwa_ref:
            print(f"\n[{nazwa}] to ref, pomijam transformacje")
            przetworzone[nazwa] = ch
            continue

        print(f"\n[{nazwa}] -> rejestracja do {nazwa_ref}")
        T = rejestracja_targetami(targety[nazwa], targety[nazwa_ref])
        print(f"  macierz T:\n{T}")
        ch.transform(T)

        # opcjonalnie ICP
        if args.icp:
            print(f"  ICP (threshold={args.icp_threshold}m)...")
            wynik_icp = o3d.pipelines.registration.registration_icp(
                ch, ref_chmura,
                args.icp_threshold,
                np.eye(4),
                o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50))
            print(f"  ICP fitness={wynik_icp.fitness:.4f}, rmse={wynik_icp.inlier_rmse:.4f}m")
            print(f"  macierz ICP:\n{wynik_icp.transformation}")
            ch.transform(wynik_icp.transformation)

        przetworzone[nazwa] = ch

    # scalanie i kolorowanie
    kolory = [
        [1, 0.5, 0], [0, 0.5, 1], [0.2, 0.8, 0.2],
        [0.9, 0.2, 0.7], [0.9, 0.9, 0.1], [0.3, 0.3, 0.9]]

    cala_chmura = o3d.geometry.PointCloud()
    for i, (nazwa, ch) in enumerate(przetworzone.items()):
        kopia = o3d.geometry.PointCloud(ch)
        kopia.paint_uniform_color(kolory[i % len(kolory)])
        cala_chmura += kopia

    print(f"\nScalona chmura: {len(cala_chmura.points)} pkt")
    zapisz_chmure(cala_chmura, args.output, "merged_cloud")

    # triangulacja
    if args.mesh != "none":
        print(f"\n=== triangulacja: {args.mesh} ===")
        for nazwa, ch in przetworzone.items():
            print(f"\n[{nazwa}]")
            m = zrob_mesh(ch, args.mesh, args.voxel)
            if m is None or len(m.triangles) == 0:
                print("  brak trojkatow, pomijam")
                continue
            zapisz_mesh(m, args.output, f"mesh_{nazwa}")

    # wizualizacja
    try:
        o3d.visualization.draw_geometries(
            [cala_chmura], window_name="TLS - scalona chmura")
    except Exception:
        print("(nie udalo sie otworzyc wizualizacji, pewnie brak wyswietlacza)")


if __name__ == "__main__":
    main()
