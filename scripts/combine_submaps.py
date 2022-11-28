import open3d as o3d
import os
import sys

def load_point_clouds(fdir, n_submaps, voxel_size=0.0):
    pcds = []
    if n_submaps is None:
        n_submaps = len(os.listdir(fdir))-1
    for i in range(1,n_submaps+1):
        pcd = o3d.io.read_point_cloud(os.path.join(fdir,"%d.pcd" %i))
        if voxel_size>0:
            pcd_down = pcd.voxel_down_sample(voxel_size)
        else: 
            pcd_down = pcd

        pcds.append(pcd_down)
    return pcds

def main(args):

    fdir = args[0] 
    voxel_size = float(args[1])
    if len(args) > 2:
        n_submaps = int(args[2])
    else:
        n_submaps = None

    pcds = load_point_clouds(fdir,n_submaps)
    pcd_combined = o3d.geometry.PointCloud()
    for point_id in range(len(pcds)):
        pcd_combined += pcds[point_id]
        # pcd_combined += pcds[point_id].voxel_down_sample(voxel_size)
    pcd_combined_down = pcd_combined.voxel_down_sample(voxel_size)

    o3d.visualization.draw_geometries([pcd_combined_down])    
    o3d.io.write_point_cloud(os.path.join(fdir,"output.pcd"), pcd_combined_down)

if __name__ == '__main__':
    main(sys.argv[1:])                                    