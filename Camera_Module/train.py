from ultralytics import YOLO

def train_model():
    # 1. 加载模型
    model = YOLO('yolov8n-obb.pt') 

    # 2. 开始训练
    results = model.train(
        data=r'C:\Users\LCY\Desktop\eye\dataset\data.yaml', 
        epochs=100,
        imgsz=640,
        device=0,
        batch=16,
        workers=4,  # 如果还是报错，可以尝试把这个数字改小，比如 4 或者 2
        task='obb'
    )

if __name__ == '__main__':
    # 这一行是必须的，用来保护多进程启动
    train_model()