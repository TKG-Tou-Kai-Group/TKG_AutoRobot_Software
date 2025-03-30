import cv2
import numpy as np
from numba import jit

@jit('float64[:,:](uint8[:,:],float64[:,:],float64[:,:])', nopython=True, cache=True)
def matching_template(img, positive_template, negative_template):
    img_h, img_w = img.shape
    template_h, template_w = positive_template.shape
    result = np.zeros((img_h - template_h, img_w - template_w), np.float64)
    for y in range(img_h - template_h):
        for x in range(img_w - template_w):
            value_1 = 0.0
            value_2 = 0.0
            for v in range(template_h//2):
                for u in range(template_w):
                    value_1 += img[y+v, x+u] * (positive_template[v, u] - negative_template[v, u])
                    value_2 += img[y+v+template_h//2, x+u] * (positive_template[v+template_h//2, u] - negative_template[v+template_h//2, u])
            if value_1 < 0:
                value_1 = 0
            if value_2 < 0:
                value_2 = 0
            value = value_1 * value_2
            result[y, x] = value
    return result

class DamagePanelDetection:
    DEBUG = False

    def __init__(self, target_h, target_s, target_v, sigma_h, sigma_s, sigma_v):
        self.target_h = target_h
        self.target_s = target_s
        self.target_v = target_v
        self.sigma_h = sigma_h
        self.sigma_s = sigma_s
        self.sigma_v = sigma_v

        self.detect_result = None

        self.make_template()

    def detect(self, img):
        # OpenCVはBGRがデフォルトなのでHSVに変換
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # 各画素のHSV値との差分を計算
        # データ型をfloatに変換しておくと計算がしやすいです
        h = img_hsv[:, :, 0].astype(np.float32)
        s = img_hsv[:, :, 1].astype(np.float32)
        v = img_hsv[:, :, 2].astype(np.float32)

        # Hueは周期性（0～179の循環）を持つので、最小の角度差を考慮
        diff_h = np.minimum(np.abs(h - self.target_h), 180 - np.abs(h - self.target_h))
        # 彩度と明度は通常の差分を計算
        diff_s = np.abs(s - self.target_s)
        diff_v = np.abs(v - self.target_v)

        # ガウス関数による重みづけ
        # ガウス関数: exp( -((x - μ)^2) / (2σ^2) )
        weight_h = np.exp(-0.5 * (diff_h / self.sigma_h) ** 2)
        weight_s = np.exp(-0.5 * (diff_s / self.sigma_s) ** 2)
        weight_v = np.exp(-0.5 * (diff_v / self.sigma_v) ** 2)

        # 各チャネルの重みを組み合わせる方法は複数考えられますが、
        # ここでは単純に各重みの積をとります
        weight = weight_h * weight_s * weight_v

        # マスク作成
        mask = (weight * 255).astype(np.uint8)

        # 7. マスクを元画像に適用して抽出結果を作成
        if self.DEBUG:
            self.detect_result = cv2.bitwise_and(img, img, mask=mask)

        matching_result = matching_template(mask, self.positive_template, self.negative_template)
        _, max_val, _, max_loc = cv2.minMaxLoc(matching_result)
        # TODO: しきい値の調整は必須
        #threashold = 1800000000000.0
        threashold = 630000000000.0 / 200.0
        if self.DEBUG:
            print(f"最大値: {max_val}, しきい値：{threashold}, 位置: {max_loc}, 判定率: {max_val / threashold * 100}")
        if max_val < threashold:
            return (-1, -1)
        else:
            return (int(max_loc[0] + self.SIZE_X//2), int(max_loc[1] + self.SIZE_Y//2))

    def make_template(self):

        # パラメータ設定
        SCALE_X = 0.2
        SCALE_Y = 0.4
        self.SIZE_Y = 86
        self.SIZE_X = int(self.SIZE_Y / SCALE_Y * SCALE_X)

        positive_mu_1 = -0.235 / 2
        positive_mu_2 = 0.235 / 2
        positive_sigma = 0.03  # 標準偏差

        # Y方向の数列を生成
        x = np.linspace(-SCALE_Y/2, SCALE_Y/2, self.SIZE_Y)
        positive_y_1 = np.exp(-((x - positive_mu_1) ** 2) / (2 * positive_sigma ** 2))  # ガウス関数
        positive_y_2 = np.exp(-((x - positive_mu_2) ** 2) / (2 * positive_sigma ** 2))  # ガウス関数
        positive_y = (positive_y_1 + positive_y_2) * 255

        # 縦ベクトル化
        positive_y = positive_y.reshape((self.SIZE_Y, 1))  # (SIZE_Y, 1) に変形

        # 横方向に繰り返して (SIZE_Y, SIZE_X) の画像を生成
        self.positive_template = np.tile(positive_y, (1, self.SIZE_X // 2))
        self.positive_template = np.concatenate([np.zeros((self.SIZE_Y, self.SIZE_X // 4), float), self.positive_template, np.zeros((self.SIZE_Y, self.SIZE_X // 4), float)], 1)


        negative_mu = 0
        negative_sigma = 0.03  # 標準偏差
        negative_mu_1 = -SCALE_Y/2
        negative_mu_2 =  SCALE_Y/2
        negative_mu_3 = -SCALE_X/2
        negative_mu_4 =  SCALE_X/2
        negative_sigma_2 = 0.02  # 標準偏差

        # Y方向の数列を生成
        negative_y = np.exp(-((x - negative_mu) ** 2) / (2 * negative_sigma ** 2))  # ガウス関数
        negative_y_1 = np.exp(-((x - negative_mu_1) ** 2) / (2 * negative_sigma_2 ** 2))  # ガウス関数
        negative_y_2 = np.exp(-((x - negative_mu_2) ** 2) / (2 * negative_sigma_2 ** 2))  # ガウス関数
        negative_y = (negative_y + negative_y_1 + negative_y_2) * 255

        # X方向の数列を生成
        y = np.linspace(-SCALE_X/2, SCALE_X/2, self.SIZE_X)
        negative_x_1 = np.exp(-((y - negative_mu_3) ** 2) / (2 * negative_sigma_2 ** 2))  # ガウス関数
        negative_x_2 = np.exp(-((y - negative_mu_4) ** 2) / (2 * negative_sigma_2 ** 2))  # ガウス関数
        negative_x = (negative_x_1 + negative_x_2) * 255
        negative_x = negative_x.reshape((1, self.SIZE_X))

        # 縦ベクトル化
        negative_y = negative_y.reshape((self.SIZE_Y, 1))  # (SIZE_Y, 1) に変形

        # 横方向に繰り返して (SIZE_Y, SIZE_X) の画像を生成
        self.negative_template = np.tile(negative_y, (1, self.SIZE_X)) + np.tile(negative_x, (self.SIZE_Y, 1))

    def get_detect_result(self):
        return self.detect_result
