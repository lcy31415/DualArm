import cv2
import numpy as np

def draw_obb_on_image(image, label_data):
    """
    在图片上绘制OBB框、重心和方向箭头
    :param image: OpenCV图像数组
    :param label_data: 包含标签数据的列表，每一行是字符串或列表
    """
    h_img, w_img = image.shape[:2]
    
    # 定义颜色 (B, G, R)
    COLOR_BOX = (0, 255, 0)      # 绿色：边框
    COLOR_CENTER = (0, 0, 255)   # 红色：重心
    COLOR_ARROW = (255, 0, 0)    # 蓝色：方向箭头
    COLOR_TEXT = (255, 255, 255) # 白色：文字

    for line in label_data:
        # 1. 解析数据
        if isinstance(line, str):
            parts = list(map(float, line.strip().split()))
        else:
            parts = line
            
        class_id = int(parts[0])
        coords = parts[1:] # 后8个是坐标
        
        # 2. 坐标反归一化 (Normalized -> Pixel)
        # 格式: x1, y1, x2, y2, x3, y3, x4, y4
        points = []
        for i in range(0, 8, 2):
            x = int(coords[i] * w_img)
            y = int(coords[i+1] * h_img)
            points.append([x, y])
        
        pts = np.array(points, np.int32)
        
        # 3. 绘制多边形 (OBB框)
        # reshape成 (-1, 1, 2) 是cv2.polylines的要求
        pts_reshaped = pts.reshape((-1, 1, 2))
        cv2.polylines(image, [pts_reshaped], isClosed=True, color=COLOR_BOX, thickness=2)
        
        # 4. 计算重心 (Centroid)
        # 方法：直接求4个顶点的平均值
        center_x = int(np.mean(pts[:, 0]))
        center_y = int(np.mean(pts[:, 1]))
        cv2.circle(image, (center_x, center_y), 5, COLOR_CENTER, -1) # 画实心圆
        
        # 5. 计算并绘制方向 (Orientation)
        # 逻辑：对于二指夹爪，通常沿着物体的“长边”抓取，或者垂直于长边
        # 这里我们需要找出哪一条边是长边，以此作为主方向
        
        p0, p1, p2 = pts[0], pts[1], pts[2]
        
        # 计算前两条边的长度
        dist01 = np.linalg.norm(p0 - p1)
        dist12 = np.linalg.norm(p1 - p2)
        
        # 确定长边向量
        if dist01 > dist12:
            # 0-1 是长边
            vec = p1 - p0
            length = dist01
        else:
            # 1-2 是长边
            vec = p2 - p1
            length = dist12
            
        # 归一化向量并放大，以便绘制
        if length > 0:
            vec = vec / length
            arrow_len = 50 # 箭头的像素长度
            end_x = int(center_x + vec[0] * arrow_len)
            end_y = int(center_y + vec[1] * arrow_len)
            
            # 画箭头：从重心指向长边方向
            cv2.arrowedLine(image, (center_x, center_y), (end_x, end_y), 
                            COLOR_ARROW, thickness=2, tipLength=0.3)
        
        # 6. (可选) 标注类别ID
        cv2.putText(image, f"ID: {class_id}", (pts[0][0], pts[0][1] - 10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLOR_TEXT, 1)

    return image

# ==========================================
# 模拟运行部分
# ==========================================

if __name__ == "__main__":
    # ---------------------------------------------------------
    # 1. 读取真实图片 (使用了 r'' 来解决路径报错)
    # ---------------------------------------------------------
    img_path = r'C:\Users\LCY\Desktop\eye\dataset\test\images\0a5eba0d5ded7adb_jpg.rf.809a4519e34846068216c3d9a1995c36.jpg'
    real_image = cv2.imread(img_path)

    # 安全检查：确保图片真的读到了
    if real_image is None:
        print(f"❌ 错误：无法找到图片，请检查路径是否正确：\n{img_path}")
    else:
        print(f"✅ 成功读取图片，尺寸：{real_image.shape}")

        # ---------------------------------------------------------
        # 2. 准备标签数据 (把你之前的标签数据放在这里)
        # ---------------------------------------------------------
        label_txt = """
0 0.9984375 0.6732937500000001 0.22766093096200982 0.373879338903796 0.19843750000000004 0.5076203125000001 0.9692140690379902 0.8070347235962042
0 0.9984375 0.7905171875000001 0.1617615573271852 0.5072238919628544 0.13906250000000006 0.6264046875000001 0.975738442672815 0.9096979830371457
        """
        lines = label_txt.strip().split('\n')

        # ---------------------------------------------------------
        # 3. 处理图片 (这里改动了：把 dummy_image 换成了 real_image)
        # ---------------------------------------------------------
        result_img = draw_obb_on_image(real_image, lines)

        # 4. 显示结果
        # 注意：如果图片太大，屏幕放不下，可以先 resize 一下
        # result_img = cv2.resize(result_img, (800, 600)) 
        
        cv2.imshow("YOLOv8 OBB Visualization", result_img)
        print("按任意键退出窗口...")
        cv2.waitKey(0)
        cv2.destroyAllWindows()