import cv2
import numpy as np
import os
import json
import time
from config import COORDINATES_FILE

# ====================== 可调参数（“置信度”相关） ======================
# 最小面积阈值：用于过滤噪声/小块，数值越大检测越严格
MIN_AREA = 700
# 最大面积阈值：过滤巨大多边形背景干扰
MAX_AREA = 50000
# 颜色饱和度下限：越大越严格（在光照较弱或颜色不纯时可适当降低）
MIN_SAT = 100
# 亮度下限：越大越严格（画面偏暗时可适当降低）
MIN_VAL = 80
# 形状近似阈值：轮廓近似的精度（占周长比例）
APPROX_EPS_RATIO = 0.05
ASPECT_TOL = 0.3
MIN_VERTICES = 4
MAX_VERTICES = 8
MIN_EXTENT = 0.5
# 中心像素稳定阈值：波动小于该值则不更新中心坐标
STABLE_TOL = 2
# 角度时域平滑系数（0~1）：越小越平滑但延迟越高
ALPHA = 0.2
# 连续稳定帧阈值：达到该帧数后写一次数据
STABLE_FRAMES = 10

def open_camera():
    cap = cv2.VideoCapture(1)
    if not cap.isOpened():
        cap = cv2.VideoCapture(1, cv2.CAP_DSHOW)
    if not cap.isOpened():
        return None
    return cap

def color_masks(hsv):
    # red
    lower_red1 = np.array([0, 134, 93])
    upper_red1 = np.array([14, 255, 255])
    lower_red2 = np.array([170, 100, 70])
    upper_red2 = np.array([180, 255, 255])
    red = cv2.bitwise_or(cv2.inRange(hsv, lower_red1, upper_red1), cv2.inRange(hsv, lower_red2, upper_red2))
    # green
    lower_green = np.array([63, 99, 70])
    upper_green = np.array([85, 255, 255])
    green = cv2.inRange(hsv, lower_green, upper_green)
    # blue
    lower_blue = np.array([100, 144, 151])
    upper_blue = np.array([130, 255, 255])
    blue = cv2.inRange(hsv, lower_blue, upper_blue)

    return {"red": red, "green": green, "blue": blue}

def refine(mask):
    k = np.ones((5,5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k)
    return mask

def smooth_angle(name, current_angle, last_angles):
    if name in last_angles:
        filtered = ALPHA * current_angle + (1.0 - ALPHA) * last_angles[name]
        last_angles[name] = filtered
        return filtered
    else:
        last_angles[name] = current_angle
        return current_angle

def draw_blocks(frame, hsv, prev_centers, last_angles):
    masks = color_masks(hsv)
    colors_bgr = {"red": (0,0,255), "green": (0,255,0), "blue": (255,0,0)}
    blocks = []
    for name, m in masks.items():
        m = refine(m)
        cnts = cv2.findContours(m, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
        for c in cnts:
            area = cv2.contourArea(c)
            # 面积过滤：太小或太大都认为是干扰
            if area < MIN_AREA or area > MAX_AREA:
                continue
            # 勾勒形状（轮廓近似）
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, APPROX_EPS_RATIO * peri, True)
            # 顶点数量过滤：近似四边形（容忍轻微噪声）
            vtx = len(approx)
            if vtx < MIN_VERTICES or vtx > MAX_VERTICES:
                continue
            # 宽高比过滤：正方形宽高比应接近 1
            x,y,w,h = cv2.boundingRect(approx)
            if h <= 0 or w <= 0:
                continue
            ar = w / float(h)
            if not (1.0 - ASPECT_TOL <= ar <= 1.0 + ASPECT_TOL):
                continue
            # 填充度过滤：轮廓面积 / 外接矩形面积，过小说明不规则或空洞
            extent = area / float(w*h)
            if extent < MIN_EXTENT:
                continue
            cv2.drawContours(frame, [approx], -1, colors_bgr[name], 2)
            # 计算并标注中心
            M = cv2.moments(c)
            if M["m00"] > 0:
                cx = int(M["m10"]/M["m00"])
                cy = int(M["m01"]/M["m00"])
            else:
                r = cv2.minAreaRect(c)
                cx, cy = int(r[0][0]), int(r[0][1])
            # 计算方向角（以 y 轴为正方向，逆时针为正）：使用最小外接矩形，并归一化到 [-45,45]
            rect = cv2.minAreaRect(c)
            box = cv2.boxPoints(rect).astype(np.float32)
            max_len = -1.0
            angle_deg = 0.0
            for i in range(4):
                p1 = box[i]
                p2 = box[(i+1) % 4]
                dx = float(p2[0] - p1[0])
                dy = float(p2[1] - p1[1])
                L = dx*dx + dy*dy
                if L > max_len:
                    max_len = L
                    # 以 y 轴为参考：atan2(vx, vy)
                    angle_deg = float(np.degrees(np.arctan2(dx, dy)))
            # 角度归一化到 [-45, 45]
            if angle_deg < -45.0:
                angle_deg += 90.0
            elif angle_deg > 45.0:
                angle_deg -= 90.0
            # 角度时域滤波（移动平均）
            angle_deg = float(smooth_angle(name, angle_deg, last_angles))
            # 中心像素稳定：若变化小于阈值则保持上次中心
            if name in prev_centers:
                lx, ly = prev_centers[name]
                if abs(cx - lx) < STABLE_TOL and abs(cy - ly) < STABLE_TOL:
                    cx, cy = lx, ly
                else:
                    prev_centers[name] = (cx, cy)
            else:
                prev_centers[name] = (cx, cy)
            cv2.circle(frame, (cx, cy), 6, colors_bgr[name], -1)
            cv2.putText(frame, f"{name} ({cx},{cy}) ang={angle_deg:.1f}°", (cx+8, cy-8), cv2.FONT_HERSHEY_SIMPLEX, 0.6, colors_bgr[name], 2)
            blocks.append({"color": name, "x": float(cx), "y": float(cy), "angle": float(angle_deg)})
    return frame, blocks

def main():
    cap = open_camera()
    if cap is None:
        print("摄像头打开失败")
        return
    print("摄像头预览启动")
    saved = False
    stable_counter = 0
    blocks_prev = []
    while True:
        ret, frame = cap.read()
        if not ret:
            cv2.waitKey(10)
            continue
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        if 'prev_centers' not in globals():
            prev_centers = {}
        if 'last_angles' not in globals():
            last_angles = {}
        out, blocks = draw_blocks(frame, hsv, prev_centers, last_angles)
        if not saved:
            # 当前帧与上一帧按颜色进行近邻匹配，所有当前块在 STABLE_TOL 内则记为稳定
            stable = True
            by_color_prev = {}
            for b in blocks_prev:
                by_color_prev.setdefault(b["color"], []).append((b["x"], b["y"]))
            for b in blocks:
                color = b["color"]
                cx, cy = b["x"], b["y"]
                if color not in by_color_prev or len(by_color_prev[color]) == 0:
                    stable = False
                    break
                dists = [abs(cx - px) + abs(cy - py) for (px, py) in by_color_prev[color]]
                if min(dists) >= STABLE_TOL:
                    stable = False
                    break
            blocks_prev = [{"color": b["color"], "x": b["x"], "y": b["y"]} for b in blocks]
            stable_counter = stable_counter + 1 if stable else 0
            if stable and stable_counter >= STABLE_FRAMES and len(blocks) > 0:
                try:
                    os.makedirs(os.path.dirname(COORDINATES_FILE), exist_ok=True)
                    data = {"timestamp": time.time(), "blocks": blocks}
                    tmp = COORDINATES_FILE + ".tmp"
                    with open(tmp, "w", encoding="utf-8") as f:
                        json.dump(data, f, ensure_ascii=False, indent=2)
                    os.replace(tmp, COORDINATES_FILE)
                    saved = True
                    print(f"检测稳定，已写入 {len(blocks)} 个小物块")
                except Exception:
                    pass
        cv2.imshow("Detect", out)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
