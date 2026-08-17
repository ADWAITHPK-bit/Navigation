#!/usr/bin/env python3
from pathlib import Path
from ultralytics import YOLO

def main():
    dataset_dir = Path.home() / 'igvc_dataset'
    data_yaml = dataset_dir / 'data.yaml'
    model = YOLO('yolo11n.pt')
    model.train(
        data=str(data_yaml),
        epochs=100,
        imgsz=640,
        batch=16,
        device=0,
        workers=4,
        project=str(Path.home() / 'igvc_yolo_runs'),
        name='obstacle_detector',
        pretrained=True,
        patience=20,
        save=True,
        plots=True
    )

    metrics = model.val(
        data=str(data_yaml),
        imgsz=640,
        device=0
    )

    print("\nTraining complete.")
    print(
        f"mAP50: {metrics.box.map50:.4f}"
    )
    print(
        f"mAP50-95: {metrics.box.map:.4f}"
    )

if __name__ == '__main__':
    main()