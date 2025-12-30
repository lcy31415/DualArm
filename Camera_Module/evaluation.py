from ultralytics import YOLO  # <--- 必须加上这一行导入

def evaluate():
    # 1. 加载你训练好的最佳模型
    # 注意：确保路径正确，如果 runs/obb/train3/weights/best.pt 不存在，请检查文件夹名
    model = YOLO(r'runs\obb\train3\weights\best.pt') 

    # 2. 在验证集上评估模型
    metrics = model.val()

    # 3. 打印一些关键指标
    print(f"mAP 50-95: {metrics.obb.map}")    # OBB 的平均精度
    print(f"mAP 50: {metrics.obb.map50}")      # 阈值为 0.5 时的精度
    print(f"mAP 75: {metrics.obb.map75}")      # 阈值为 0.75 时的精度

if __name__ == '__main__':
    evaluate()