import cv2
import math
import numpy as np

class ExtractObject:
    camera_matrix = np.array([[1000, 0, 640], [0, 1000, 360], [0, 0, 1]], dtype=np.float32)
    dist_coeffs = np.zeros(5, dtype=np.float32)

    camera_x = 0.0
    camera_y = 0.0
    camera_z = 0.0
    camera_pitch = 0.0
    camera_yaw = 0.0

    target_x = 3.0
    target_y = 0.0
    target_z = 0.38
    target_width = 1.2
    target_height = 0.5 # 0.26
    
    def __init__(self, camera_matrix, dist_coeffs, camera_x = 0.08, camera_y = -0.01, camera_z = -0.06, target_z = 0.02, target_width = 1.2, target_height = 0.6):
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs
        self.camera_x = camera_x
        self.camera_y = camera_y
        self.camera_z = camera_z
        self.target_z = target_z
        self.target_width = target_width
        self.target_height = target_height

    def crop_3d_quad(self, image, points_3d, camera_matrix, dist_coeffs, output_size=(200, 200)):
        """
        3D座標の4点を用いて画像を切り出す
        :param image: 入力画像
        :param points_3d: 3D座標の4点 (numpy array shape: (4, 3))
        :param camera_matrix: カメラ行列 (numpy array shape: (3, 3))
        :param dist_coeffs: 歪み係数 (numpy array shape: (5,))
        :param output_size: 出力画像サイズ (w, h)
        :return: 変換後の画像
        """
        # 3D座標を2D画像座標に変換
        points_2d, _ = cv2.projectPoints(points_3d, np.zeros((3, 1)), np.zeros((3, 1)), camera_matrix, dist_coeffs)
        points_2d = points_2d.reshape(-1, 2)
    
        # 変換後の座標を定義（出力画像の4隅）
        width, height = output_size
        dst_points = np.array([[0, 0], [width-1, 0], [width-1, height-1], [0, height-1]], dtype=np.float32)
    
        # 射影変換行列を求める
        M = cv2.getPerspectiveTransform(points_2d.astype(np.float32), dst_points)
    
        # 透視変換を適用
        cropped = cv2.warpPerspective(image, M, (width, height))

        # 画像に四角形を描画
        image_with_quad = image.copy()
        cv2.polylines(image_with_quad, [points_2d.astype(np.int32)], True, (0, 255, 0), thickness=2)
    
        return cropped, image_with_quad

    def extract(self, image, target_x, target_y, camera_pitch, camera_yaw, output_size=(200, 200)):
        self.target_x = target_x
        self.target_y = target_y
        self.camera_pitch = camera_pitch
        self.camera_yaw = camera_yaw
        print(self.target_x, self.target_y, self.target_z)

        points_3d = np.array([
            [-self.target_y-self.target_width/2.0*math.cos(math.atan2(self.target_y,self.target_x)), -self.target_z-self.target_height/2.0, self.target_x-self.target_width/2.0*math.sin(math.atan2(self.target_y,self.target_x))],
            [-self.target_y+self.target_width/2.0*math.cos(math.atan2(self.target_y,self.target_x)), -self.target_z-self.target_height/2.0, self.target_x+self.target_width/2.0*math.sin(math.atan2(self.target_y,self.target_x))],
            [-self.target_y+self.target_width/2.0*math.cos(math.atan2(self.target_y,self.target_x)), -self.target_z+self.target_height/2.0, self.target_x+self.target_width/2.0*math.sin(math.atan2(self.target_y,self.target_x))],
            [-self.target_y-self.target_width/2.0*math.cos(math.atan2(self.target_y,self.target_x)), -self.target_z+self.target_height/2.0, self.target_x-self.target_width/2.0*math.sin(math.atan2(self.target_y,self.target_x))]
        ], dtype=np.float32)
        print(points_3d)

        for point_3d in points_3d:
            point_x = point_3d[0]
            point_z = point_3d[2]
            point_3d[2] = point_z * math.cos(-self.camera_yaw) - (-point_x) * math.sin(-self.camera_yaw)
            point_3d[0] = -(point_z * math.sin(-self.camera_yaw) + (-point_x) * math.cos(-self.camera_yaw))

            point_y = point_3d[1]
            point_z = point_3d[2]
            point_3d[2] = point_z * math.cos(-self.camera_pitch) - (-point_y) * math.sin(-self.camera_pitch)
            point_3d[1] = -(point_z * math.sin(-self.camera_pitch) + (-point_y) * math.cos(-self.camera_pitch))
    
            point_3d[0] -= -self.camera_y
            point_3d[1] -= -self.camera_z
            point_3d[2] += -self.camera_x
        print(points_3d)
            
        return self.crop_3d_quad(image, points_3d, self.camera_matrix, self.dist_coeffs, output_size)

'''
# 入力画像を読み込む（適当なカメラ画像を使用）
image = cv2.imread('input.jpg')
image = cv2.resize(image, (1280,720))

# 四角形領域を切り出し
#cropped_image = crop_3d_quad(image, points_3d, camera_matrix, dist_coeffs)
extractor = ExtractObject(np.array([[1000, 0, 640], [0, 1000, 360], [0, 0, 1]], dtype=np.float32), np.zeros(5, dtype=np.float32))
cropped_image, result_image = extractor.extract(image, 3.0, 0.0, 0.0, 0.0)

# 結果を表示
cv2.imshow('input.jpg', image)
cv2.imshow('Cropped Image', result_image)
cv2.waitKey(0)
cv2.destroyAllWindows()
'''