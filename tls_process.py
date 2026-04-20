
import argparse
import os
import glob
import numpy as np
import laspy
import open3d as o3d


def load_station(path):
    # wczytanie stanowiska z LAS/TXT

    ext = os.path.splitext(path)[1].lower()
    if ext in (".las", ".laz"):
        las = laspy.read(path)
        xyz = np.asarray(las.xyz)
    elif ext == ".txt":
        xyz = np.loadtxt(path, comments="#")[:, :3]
    else:
        raise ValueError(f"Nieobslugiwany format: {path}")
    pc = o3d.geometry.PointCloud()
    pc.points = o3d.utility.Vector3dVector(xyz)
    return pc


def downsample(pc, voxel_size):
    #Rozrzedzenie chmury punktow metoda voxel downsampling.
    out = pc.voxel_down_sample(voxel_size=voxel_size)
    print(f"  Downsampling ({voxel_size*100:.0f} cm): "
          f"{len(pc.points)} -> {len(out.points)} punktow")
    return out


def filter_cloud(pc, voxel_size):
    #Filtracja geometryczna (ROR) + statystyczna (SOR).
    n0 = len(pc.points)
    pc_ror, _ = pc.remove_radius_outlier(nb_points=6, radius=voxel_size * 3.0)
    print(f"  ROR: {n0} -> {len(pc_ror.points)} punktow "
          f"(usunieto {n0 - len(pc_ror.points)})")
    n1 = len(pc_ror.points)
    pc_sor, _ = pc_ror.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    print(f"  SOR: {n1} -> {len(pc_sor.points)} punktow "
          f"(usunieto {n1 - len(pc_sor.points)})")
    return pc_sor


def estimate_normals(pc, voxel_size):
    # estymacja normalnych, orientacja w strone skanera
    pc.estimate_normals(
        search_param=o3d.geometry.KDTreeSearchParamHybrid(
            radius=voxel_size * 4.0, max_nn=30
        )
    )
    pc.orient_normals_towards_camera_location(camera_location=np.array([0.0, 0.0, 0.0]))
    return pc


def load_targets(path):
    # wczytanie targetow z TXT (id x y z)
    targets = {}
    with open(path, "r") as f:
        for line in f:
            s = line.strip()
            if not s or s.startswith("#"):
                continue
            parts = s.split()
            if len(parts) < 4:
                continue
            tid = int(float(parts[0]))
            xyz = np.array([float(parts[1]), float(parts[2]), float(parts[3])])
            targets[tid] = xyz
    return targets


def target_based_registration(src_targets, ref_targets):
    # transformacja src -> ref na podstawie wspolnych targetow
    common = sorted(set(src_targets.keys()) & set(ref_targets.keys()))
    if len(common) < 3:
        raise ValueError(f"Za malo wspolnych punktow wiazacych: {len(common)} (min. 3)")
    src_pc = o3d.geometry.PointCloud()
    ref_pc = o3d.geometry.PointCloud()
    src_pc.points = o3d.utility.Vector3dVector(np.array([src_targets[i] for i in common]))
    ref_pc.points = o3d.utility.Vector3dVector(np.array([ref_targets[i] for i in common]))
    corrs = o3d.utility.Vector2iVector([[i, i] for i in range(len(common))])
    est = o3d.pipelines.registration.TransformationEstimationPointToPoint()
    T = est.compute_transformation(src_pc, ref_pc, corrs)

    src_np = np.asarray(src_pc.points)
    ref_np = np.asarray(ref_pc.points)
    src_h = np.hstack([src_np, np.ones((len(src_np), 1))])
    src_t = (T @ src_h.T).T[:, :3]
    rmse = float(np.sqrt(np.mean(np.sum((src_t - ref_np) ** 2, axis=1))))
    print(f"  Wspolne ID: {common}, RMSE na targets: {rmse:.4f} m")
    return T


def save_cloud(pc, out_dir, name):
    # zapis chmury do PLY + LAS
    os.makedirs(out_dir, exist_ok=True)
    ply = os.path.join(out_dir, f"{name}.ply")
    o3d.io.write_point_cloud(ply, pc)
    print(f"  Zapisano: {ply}")

    xyz = np.asarray(pc.points)
    header = laspy.LasHeader(point_format=3, version="1.2")
    header.offsets = np.min(xyz, axis=0)
    header.scales = np.array([0.001, 0.001, 0.001])
    las = laspy.LasData(header)
    las.x = xyz[:, 0]
    las.y = xyz[:, 1]
    las.z = xyz[:, 2]
    las_path = os.path.join(out_dir, f"{name}.las")
    las.write(las_path)
    print(f"  Zapisano: {las_path}")


def reconstruct_mesh(pc, method, voxel_size):
    # triangulacja BPA lub Poisson
    if not pc.has_normals():
        pc.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(
                radius=voxel_size * 4.0, max_nn=30
            )
        )
        pc.orient_normals_towards_camera_location(np.array([0.0, 0.0, 0.0]))

    if method == "bpa":
        radii = [voxel_size * r for r in (1.5, 2.0, 3.0, 4.0)]
        print(f"  Ball Pivoting, radii={radii}")
        mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(
            pc, o3d.utility.DoubleVector(radii)
        )
    elif method == "poisson":
        print("  Poisson (depth=9)")
        mesh, densities = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(
            pc, depth=9
        )
        densities = np.asarray(densities)
        thr = np.quantile(densities, 0.05)
        mesh.remove_vertices_by_mask(densities < thr)
    else:
        return None

    mesh.remove_duplicated_vertices()
    mesh.remove_duplicated_triangles()
    mesh.remove_degenerate_triangles()
    mesh.remove_non_manifold_edges()
    mesh.compute_vertex_normals()
    print(f"  Mesh: {len(mesh.vertices)} wierzcholkow, {len(mesh.triangles)} trojkatow")
    return mesh


def save_mesh(mesh, out_dir, name):
    # zapis mesh do PLY
    os.makedirs(out_dir, exist_ok=True)
    path = os.path.join(out_dir, f"{name}.ply")
    o3d.io.write_triangle_mesh(path, mesh)
    print(f"  Zapisano mesh: {path}")


def find_stations(input_dir):
    # szukanie plikow stanowisk w katalogu
    files = []
    for ext in ("*.las", "*.laz", "*.txt"):
        files.extend(glob.glob(os.path.join(input_dir, ext)))
    return sorted(files)


def main():
    ap = argparse.ArgumentParser(description="Przetwarzanie danych TLS")
    ap.add_argument("--input", default=".", help="Katalog ze stanowiskami (.las/.laz/.txt)")
    ap.add_argument("--output", required=True, help="Katalog zapisu wynikow")
    ap.add_argument("--targets", default="./targets",
                    help="Katalog z plikami TXT punktow wiazacych (nazwy == nazwy stanowisk)")
    ap.add_argument("--reference", default=None,
                    help="Nazwa stanowiska referencyjnego (bez rozszerzenia). Domyslnie pierwsze alfabetycznie.")
    ap.add_argument("--voxel", type=float, default=0.05, help="Rozmiar voxela [m] (domyslnie 0.05)")
    ap.add_argument("--stations", nargs="+", default=None,
                    help="Opcjonalna lista nazw stanowisk do przetworzenia (bez rozszerzen)")
    ap.add_argument("--icp", action="store_true",
                    help="Dodatkowa poprawa orientacji algorytmem ICP po Target-Based (ocena 4.0)")
    ap.add_argument("--icp-threshold", type=float, default=0.2,
                    help="Max odleglosc korespondencji w ICP [m] (domyslnie 0.2)")
    ap.add_argument("--mesh", choices=["none", "bpa", "poisson"], default="none",
                    help="Triangulacja per stanowisko: bpa=Ball Pivoting, poisson=Poisson (ocena 4.0)")
    args = ap.parse_args()

    all_files = find_stations(args.input)
    if args.stations:
        wanted = set(args.stations)
        all_files = [f for f in all_files
                     if os.path.splitext(os.path.basename(f))[0] in wanted]
    else:
        def has_targets(p):
            name = os.path.splitext(os.path.basename(p))[0]
            return os.path.exists(os.path.join(args.targets, name + ".txt"))
        all_files = [f for f in all_files if has_targets(f)]

    if len(all_files) < 2:
        raise SystemExit(f"Znaleziono {len(all_files)} stanowisk - potrzeba >= 2.")

    print(f"Stanowiska do przetworzenia: {[os.path.basename(f) for f in all_files]}")

    stations = {}
    targets_per_station = {}
    for path in all_files:
        name = os.path.splitext(os.path.basename(path))[0]
        print(f"\n[Stanowisko: {name}]  plik: {path}")
        pc = load_station(path)
        print(f"  Wczytano: {len(pc.points)} punktow")
        pc = downsample(pc, args.voxel)
        pc = filter_cloud(pc, args.voxel)
        pc = estimate_normals(pc, args.voxel)
        stations[name] = pc

        tpath = os.path.join(args.targets, name + ".txt")
        if not os.path.exists(tpath):
            raise SystemExit(f"Brak pliku punktow wiazacych: {tpath}")
        targets_per_station[name] = load_targets(tpath)
        print(f"  Wczytano {len(targets_per_station[name])} punktow wiazacych z {tpath}")

    ref_name = args.reference or sorted(stations.keys())[0]
    if ref_name not in stations:
        raise SystemExit(f"Stanowisko referencyjne '{ref_name}' nie zostalo wczytane.")
    print(f"\nStanowisko referencyjne: {ref_name}")

    transformed = {}
    ref_pc = stations[ref_name]
    for name, pc in stations.items():
        if name == ref_name:
            print(f"\n[{name}] referencyjne - transformacja = I")
            transformed[name] = pc
            continue

        print(f"\n[{name}] -> rejestracja Target-Based do {ref_name}")
        T = target_based_registration(targets_per_station[name],
                                      targets_per_station[ref_name])
        print(f"  Macierz Target-Based:\n{T}")
        pc.transform(T)

        if args.icp:
            print(f"  [ICP] poprawa orientacji (threshold={args.icp_threshold} m)")
            icp = o3d.pipelines.registration.registration_icp(
                pc, ref_pc, args.icp_threshold, np.eye(4),
                o3d.pipelines.registration.TransformationEstimationPointToPlane(),
                o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=50),
            )
            print(f"  [ICP] fitness={icp.fitness:.4f}  inlier_rmse={icp.inlier_rmse:.4f} m")
            print(f"  Macierz ICP (korekcja):\n{icp.transformation}")
            pc.transform(icp.transformation)

        transformed[name] = pc

    colors = [
        [1.0, 0.5, 0.0], [0.0, 0.5, 1.0], [0.2, 0.8, 0.2],
        [0.9, 0.2, 0.7], [0.9, 0.9, 0.1], [0.3, 0.3, 0.9],
    ]
    merged = o3d.geometry.PointCloud()
    for i, (name, pc) in enumerate(transformed.items()):
        colored = o3d.geometry.PointCloud(pc)
        colored.paint_uniform_color(colors[i % len(colors)])
        merged += colored

    print(f"\nScalona chmura: {len(merged.points)} punktow")
    save_cloud(merged, args.output, "merged_cloud")

    if args.mesh != "none":
        print(f"\n=== Triangulacja ({args.mesh}) per stanowisko ===")
        for name, pc in transformed.items():
            print(f"\n[{name}] surface reconstruction")
            mesh = reconstruct_mesh(pc, args.mesh, args.voxel)
            if mesh is None or len(mesh.triangles) == 0:
                print(f"  Brak trojkatow - pomijam zapis.")
                continue
            save_mesh(mesh, args.output, f"mesh_{name}")

    try:
        o3d.visualization.draw_geometries(
            [merged], window_name="TLS - scalona chmura"
        )
    except Exception as e:
        print(f"(pominieto wizualizacje: {e})")


if __name__ == "__main__":
    main()
