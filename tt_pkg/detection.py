import cv2
import numpy as np
from tt_pkg.config import settings_BL, settings_PU


# 创建卡尔曼滤波器
kf = cv2.KalmanFilter(4, 2)  # 状态向量为4维，测量向量为2维

# 设置状态转移矩阵
kf.transitionMatrix = np.array([[1, 0, 1, 0],
                                [0, 1, 0, 1],
                                [0, 0, 1, 0],
                                [0, 0, 0, 1]], np.float32)

# 设置测量矩阵
kf.measurementMatrix = np.array([[1, 0, 0, 0],
                                 [0, 1, 0, 0]], np.float32)

# 设置过程噪声协方差矩阵
kf.processNoiseCov = np.array([[1, 0, 0, 0],
                               [0, 1, 0, 0],
                               [0, 0, 1, 0],
                               [0, 0, 0, 1]], np.float32) * 0.03

# 设置测量噪声协方差矩阵
kf.measurementNoiseCov = np.array([[1, 0],
                                   [0, 1]], np.float32) * 0.1
def cul_dist(a, b):  # 计算两点间距
    return (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2


def open(img):  # 开操作
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (settings_BL["k_size"], settings_BL["k_size"]))  # 定义卷积核
    open_img = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel)
    open_img = cv2.morphologyEx(open_img, cv2.MORPH_DILATE, kernel)  # 略膨胀
    return open_img


def Approx(con):  # con为预先得到的最大轮廓
    num = 0.01
    # 初始化时不需要太小，因为四边形所需的值并不很小
    ep = num * cv2.arcLength(con, True)
    con = cv2.approxPolyDP(con, ep, True)
    while 1:
        if len(con) <= 4:  # 防止程序崩溃设置的<=4
            break
        else:
            num = num * 1.5
            ep = num * cv2.arcLength(con, True)
            con = cv2.approxPolyDP(con, ep, True)
            continue
    return con


def cul_pos(points):  # 计算坐标点的平均值
    x_sum = 0
    y_sum = 0

    for point in points:
        x_sum += point[0]
        y_sum += point[1]
    x_avg = int(x_sum / len(points))
    y_avg = int(y_sum / len(points))
    return x_avg, y_avg

def detect_PU(img):
    # 将图片转换为灰度图
    gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(4, 4))  # 自适应直方图均衡
    gray_img = clahe.apply(gray_img)
    # 2. 对灰度图像进行模糊处理。
    blur_img = cv2.bilateralFilter(gray_img, 9, 5, 75)  # 使用双边滤波
    blur_img = cv2.addWeighted(gray_img, 1.5, blur_img, -0.5, 0)  # 叠加
    # 3. 使用 Canny 边缘检测算法检测边缘。
    edges = cv2.Canny(blur_img, settings_PU['range'][0], settings_PU['range'][1])
    # 适当膨胀
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (9, 9))  # 定义卷积核
    dila_img = cv2.morphologyEx(edges, cv2.MORPH_DILATE, kernel)  # 膨胀

    contours, _ = cv2.findContours(dila_img, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    contour = []
    centers = []
    for single in contours:
        if (settings_PU['area_min'] <= cv2.contourArea(single) <= settings_PU['area_max'] and cv2.arcLength(single, True)
                >= settings_PU['leng'] and len(single) >= 5):  # 画出近似椭圆至少需要5个点
            contour.append(single)
            ellipse = cv2.fitEllipse(single)  # ( 椭圆拟合
            # 椭圆面积与角度筛选
            if 3.14 * ellipse[1][0] * ellipse[1][1] <= settings_PU['ellipse_max'] and (
                    30 <= ellipse[2] <= 150 or 210 <= ellipse[2] <= 330):
                center = ellipse[0]
                center = tuple(map(int, center))
                centers.append(center)
                # cv.ellipse(ori_img, ellipse, (0, 255, 0), 2)
                # cv.circle(ori_img, center, 2, (0, 255, 0), 2)
    # if contour:
    #     cv.drawContours(ori_img, contour, -1, (0, 0, 255), 3)

    selected = []
    for current in centers:
        count = 0
        temp = [current]

        for center in centers:
            if cul_dist(current, center) <= settings_PU['distance']:
                count += 1
                temp.append(center)
        if count >= settings_PU['max_count']:
            selected.append(temp)
            centers = [center for center in centers if center not in temp]  # 将已经入组的点排除
    if len(selected) <= 3:
        final = []
        for points in selected:
            result = cul_pos(points)
            analysis_result = calculate_rgb(img, result)  # 判断颜色
            if analysis_result:
                final.append(analysis_result)
        if check_distance(final, 200) and check_rgb(final) and final:  # 防止一个标靶中错判出多个点，或者误判出同种颜色
            for point in final:
                cv2.circle(img, point[1], 2, (0, 255, 0), 2)
            return final
        return None
    return None

def calculate_rgb(img, center):
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  # 转换BDR色域为HSV
    x = center[0]
    y = center[1]
    # 获取指定坐标点周围的矩形区域
    hsv = hsv_img[y, x]
    if 0 <= hsv[0] <= 5 or 175 <= hsv[0] <= 180:
        # cv2.putText(img, "red", center, cv2.FONT_HERSHEY_SIMPLEX,
        #            0.75,
        #            (0, 0, 255), 2)
        return 1, center
    if 35 <= hsv[0] <= 77:
        # cv2.putText(img, "green", center, cv2.FONT_HERSHEY_SIMPLEX,
        #            0.75,
        #            (0, 0, 255), 2)
        return 2, center
    if 100 <= hsv[0] <= 124:
        # cv2.putText(img, "blue", center, cv2.FONT_HERSHEY_SIMPLEX,
        #            0.75,
        #            (0, 0, 255), 2)
        return 3, center
    return []


def check_distance(points, limit):
    for i in range(len(points)):
        for j in range(i + 1, len(points)):
            distance = cul_dist(points[i][1], points[j][1])
            if distance < limit:
                return 0
    return 1


def check_rgb(points):
    for i in range(len(points)):
        for j in range(i + 1, len(points)):
            if points[i][0] == points[j][0]:
                return 0
    return 1

def detect_BL(img):
    hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)  # 转换BDR色域为HSV
    # 定义截取的颜色范围
    # 截取
    mask_img_r_part1 = cv2.inRange(hsv_img, np.array(settings_BL["lower_red_1"]), np.array(settings_BL["upper_red_1"]))  # 红色的颜色范围要单独处理
    mask_img_r_part2 = cv2.inRange(hsv_img, np.array(settings_BL["lower_red_2"]), np.array(settings_BL["upper_red_2"]))
    mask_img_r = cv2.bitwise_or(mask_img_r_part1, mask_img_r_part2)

    mask_img_g = cv2.inRange(hsv_img, np.array(settings_BL["lower_green"]), np.array(settings_BL["upper_green"]))

    mask_img_b = cv2.inRange(hsv_img, np.array(settings_BL["lower_blue"]), np.array(settings_BL["upper_blue"]))

    # 进行开操作，先腐蚀后膨胀
    open_img_r = open(mask_img_r)
    open_img_g = open(mask_img_g)
    open_img_b = open(mask_img_b)

    result_r = [1, find_contours(open_img_r)]

    result_g = [2, find_contours(open_img_g)]

    result_b = [3, find_contours(open_img_b)]

    return result_r, result_g, result_b


def find_contours(img):
    
    corrected_pos = []
    # 寻找轮廓
    contours, _ = cv2.findContours(img, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    contour = []
    pos = []
    for single in contours:
        if cv2.contourArea(single) >= settings_BL["area"] and cv2.arcLength(single, True) >= settings_BL["leng"]:
            # hull = cv.convexHull(single)  # 凸包
            contour.append(single)
    if contour:
        max_contour = max(contour, key=cv2.contourArea)
        # approx = Approx(max_contour)  # 使用四边形进行拟合
        # center = cul_pos(approx)
        center = cul_pos(max_contour[0])  # 计算中心点

        # (x, y), radius = cv.minEnclosingCircle(max_contour)  # 拟合圆
        # center1 = (int(x), int(y))
        # radius = int(radius)

        if len(pos) < settings_BL["window"]:  # 一开始暂时禁用卡尔曼滤波，防止第一个位置错位
            final_pos = center
            pos.append(center)
        else:
            pos.append(center)
            # 使用卡尔曼滤波器
            for single in pos:
                kf.predict()  # 预测
                corrected = kf.correct(np.array([[single[0]], [single[1]]], np.float32))  # 校正
                corrected_pos = (int(corrected[0][0]), int(corrected[1][0]))

            if cul_dist(pos[-2], corrected_pos) > settings_BL["limit"] ** 2:  # 坐标变动足够大才更新坐标
                pos[-1] = corrected_pos
                # 控制pos数组的大小
                pos = pos[1:]
            else:
                pos = pos[:-1]
            final_pos = pos[-1]

        # cv.circle(ori_img, center1, radius, (0, 255, 0), 2)
        # cv.drawContours(ori_img, [approx], -1, (0, 0, 255), 3)
        # cv2.circle(ori_img, final_pos, 5, (0, 255, 0), 2)
        # output = np.array(pos, np.int32)
        # output = output.reshape((-1, 1, 2))
        # cv.polylines(ori_img, [output], False, color=(0, 0, 255), thickness=3)
        return final_pos

    return []


def cul_text_pos(points):
    x_sum = 0
    y_sum = 0
    for point in points[0]:
        x_sum += point[0]
        y_sum += point[1]
    x_avg = int(x_sum / len(points[0]))
    y_avg = int(y_sum / len(points[0]))
    return x_avg, y_avg


def cul_k(x1, x2, x3, x4):  # 计算斜率差
    k1 = (x2[1] - x1[1]) / (float(x2[0] - x1[0]))
    k2 = (x4[1] - x3[1]) / (float(x4[0] - x3[0]))
    return (k1 - k2) ** 2


def cul_diff(points):
    p1 = points[0][0]
    p2 = points[0][1]
    p3 = points[0][2]
    p4 = points[0][3]

    return cul_k(p1, p2, p4, p3), cul_k(p1, p4, p2, p3)

def detect_QR(img):
    # 灰度化
    gray_img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # 设置检测器
    qrcoder = cv2.QRCodeDetector()
    # 检测识别二维码
    codeinfo, points, straight_qrcode = qrcoder.detectAndDecode(gray_img)

    return codeinfo, points, straight_qrcode


if __name__ == "__main__":

    print(cv2.__version__)
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FPS, 30)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
    cap.set(cv2.CAP_PROP_HUE, 0)  # 固定色调
    cap.set(cv2.CAP_PROP_SATURATION, 75)  # 设置饱和度
    cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0)  # 禁用自动曝光
    cap.set(cv2.CAP_PROP_EXPOSURE, settings_BL["exposure"])
    cap.set(6, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))

    while 1:
        # Take each frame
        _, ori_img = cap.read()
        if ori_img is None:
            continue

        codeinfo, points, straight_qrcode = detect_QR(ori_img)
        if codeinfo:
            print(codeinfo)
            a, b = cul_diff(points)
            if a <= 0.1 or b <= 0.1:  # 斜率差足够小（平行）
                # 绘制识别框
                cv2.drawContours(ori_img, [np.int32(points)], 0, (0, 0, 255), 2)
                # 绘制解码信息
                # text_pos = cul_text_pos(points)
                # cv2.putText(ori_img, codeinfo, text_pos, cv2.FONT_HERSHEY_SIMPLEX,
                #            0.5,
                #            (255, 0, 0), 1)

        # result_r, result_g, result_b = detect_BL(ori_img)      
        # if result_r[1]:
        #     cv2.circle(ori_img, result_r[1], 5, (0, 255, 0), 2)

        # if result_g[1]:
        #     cv2.circle(ori_img, result_g[1], 5, (0, 255, 0), 2)

        # if result_b[1]:
        #     cv2.circle(ori_img, result_b[1], 5, (0, 255, 0), 2)
        # print(result_r, result_g, result_b)

        final = detect_PU(ori_img)
        text = 'Canny_L:{},Canny_H:{}'.format(settings_PU['range'][0], settings_PU['range'][1])
        cv2.putText(ori_img, text, (5, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (255, 255, 255), 2)
        print(final)
        cv2.imshow("result", ori_img)

        key = cv2.waitKey(1)
        if key == 27:
            break
    cv2.destroyAllWindows()
