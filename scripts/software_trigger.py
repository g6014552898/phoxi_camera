#!/usr/bin/env python
import numpy as np
import open3d as o3d
import cv2
import os
import sys
from harvesters.core import Harvester

# PhotoneoTL_DEV_<ID>
device_id = "PhotoneoTL_DEV_InstalledExamples-basic-example"
if len(sys.argv) == 2:
    device_id = "PhotoneoTL_DEV_" + sys.argv[1]
print("--> device_id: ", device_id)

cti_file_path = os.getenv('PHOXI_CONTROL_PATH') + "/API/lib/photoneo.cti"
print("--> cti_file_path: ", cti_file_path)

with Harvester() as h:
    h.add_file(cti_file_path, True, True)
    h.update()

    # Print out available devices
    print()
    print("Name : ID")
    print("---------")
    for item in h.device_info_list:
        print(item.property_dict['serial_number'], ' : ', item.property_dict['id_'])
    print()

    with h.create({'id_': device_id}) as ia:
        features = ia.remote_device.node_map

        #print(dir(features))
        print("TriggerMode BEFORE: ", features.PhotoneoTriggerMode.value)
        features.PhotoneoTriggerMode.value = "Software"
        print("TriggerMode AFTER: ", features.PhotoneoTriggerMode.value)

        # Send every output structure
        features.SendTexture.value = True
        features.SendPointCloud.value = True
        features.SendNormalMap.value = True
        features.SendDepthMap.value = False
        features.SendConfidenceMap.value = False
        #features.SendEventMap.value = True         # MotionCam-3D exclusive
        #features.SendColorCameraImage.value = True # MotionCam-3D Color exclusive

        ia.start()

        # Trigger frame by calling property's setter.
        # Must call TriggerFrame before every fetch.
        features.TriggerFrame.execute() # trigger first frame
        buffer = ia.fetch()             # grab first frame
        print(buffer)

        # features.TriggerFrame.execute() # trigger second frame
        # buffer = ia.fetch()             # grab second frame
        # print(buffer)

        payload = buffer.payload

        # Order is dependent on the selected output structure
        # Check SendTexture, SendPointCloud, SendNormalMap, SendDepthMap, SendConfidenceMap, SendEventMap (MotionCam-3D only), SendColorCameraImage (MotionCam-3D Color only)
        # payload.components[#]
        # [0] Texture
        # [1] PointCloud [X,Y,Z,...]
        # [2] NormalMap [X,Y,Z,...]
        # [3] DepthMap
        # [4] ConfidenceMap
        # [5] EventMap
        # [6] ColorCameraImage

        # Texture Image
        texture_component = payload.components[0]
        texture = None
        if texture_component.width > 0 and texture_component.height > 0:
            # Reshape 1D array to 2D array with image size
            texture = texture_component.data.reshape(texture_component.height, texture_component.width, 1).copy()
            # Show image
            cv2.imshow("Texture", texture)

        # Point Cloud
        point_cloud_component = payload.components[1]
        # Normal Map
        norm_component = payload.components[2]
        # Visualize point cloud
        if point_cloud_component.width > 0 and point_cloud_component.height > 0:
            # Reshape for Open3D visualization to N x 3 arrays
            point_cloud = point_cloud_component.data.reshape(point_cloud_component.height * point_cloud_component.width, 3).copy()
            norm_map = norm_component.data.reshape(norm_component.height * norm_component.width, 3).copy()

            pcd = o3d.geometry.PointCloud()
            pcd.points = o3d.utility.Vector3dVector(point_cloud)
            pcd.normals = o3d.utility.Vector3dVector(norm_map)

            # If texture is present shade points with texture
            if texture is not None:
                color_xyz = np.zeros((point_cloud_component.height * point_cloud_component.width, 3))
                color_xyz[:, 0] = np.reshape(1/65536 * texture, -1)
                color_xyz[:, 1] = np.reshape(1/65536 * texture, -1)
                color_xyz[:, 2] = np.reshape(1/65536 * texture, -1)
                pcd.colors = o3d.utility.Vector3dVector(color_xyz)
            
            print(pcd)
            o3d.visualization.draw_geometries([pcd], width=800,height=600)

        # Color Camera Image
        # color_image_component = payload.components[6]
        # if color_image_component.width > 0 and color_image_component.height > 0:
            # # Reshape 1D array to 2D RGB image
            # color_image = color_image_component.data.reshape(color_image_component.height, color_image_component.width, 3).copy()
            # # Normalize array to range 0 - 65535
            # color_image = cv2.normalize(color_image, dst=None, alpha=0, beta=65535, norm_type=cv2.NORM_MINMAX)
            # # Show image
            # cv2.imshow("Color Camera Image", color_image)

        # The ia object will automatically call the destroy method
        # once it goes out of the block.

    # The h object will automatically call the reset method
    # once it goes out of the block.
