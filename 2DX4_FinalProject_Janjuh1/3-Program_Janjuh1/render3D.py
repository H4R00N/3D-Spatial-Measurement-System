import numpy as np
import open3d as o3d

def renderFile(totalSlices):#function that takes total slices
    pcd = o3d.io.read_point_cloud("points2dx4proj.xyz", format='xyz')#gets a pointcloud of all points from xyz file
    print(pcd)#prints point cloud (will show number of points)
    print(np.asarray(pcd.points))#turnspoint cloud in numpy array where you can see each point when printed out

    slicePoints = (np.asarray(pcd.points).size/3)/totalSlices#gets the amount of points in one slice by counting 1 colm (x-colm) and dividing by total slice (almost always 5)
    
    lines = []#lines array that store which points in pcd I want to be connected and visualised as
    # For loop that connects all points within the plane to its adjacent point
    for x in range(totalSlices):#for all slices
        for yz in range(int(slicePoints)):#for every point in slice
            currP = yz#set as current point
            nextP = currP+1 if (currP+1 != slicePoints) else 0#set next point to either yz+1 or 0
            lines.append([currP+x*slicePoints,nextP+x*slicePoints])#append in line

    # For loop that connects all points within the plane to its adjacent point on next plane
    for x in range(totalSlices-1):#since opperates on 2 slices at a time only need to gor to total slices -1
        for yz in range(int(slicePoints)):#for each point
            lines.append([yz+slicePoints*x,yz+slicePoints*(x+1)])#add current point and next point on next slice

    #makes a line_set structur which combine the line list that has posistions with actual values from pcd
    lines_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(np.asarray(pcd.points)),lines=o3d.utility.Vector2iVector(lines))
    colors = [[1, 0, 0] for i in range(len(lines))]#colors = red
    lines_set.colors = o3d.utility.Vector3dVector(colors)#set lines as the red color
    o3d.visualization.draw_geometries([lines_set])#3d render lines_set, visualize the lines_set struct

if __name__ == '__main__':#main method that calls method with 5 slices
    renderFile(5)
