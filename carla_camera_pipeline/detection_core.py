"""ROS-independent KITTI-style projection, YOLO and surface-depth estimation."""
import colorsys
import numpy as np
from PIL import ImageDraw


def project_lidar(points, calibration, image_shape):
    height, width = image_shape
    xyz = np.asarray(points)[:, :3].astype(np.float64)
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    camera = np.column_stack((xyz, np.ones(len(xyz)))) @ calibration['Tr_velo_to_cam'].T
    rectified = camera @ calibration['R0_rect'].T
    projected = np.column_stack((rectified, np.ones(len(rectified)))) @ calibration['P2'].T
    valid = (rectified[:, 2] > 0) & (projected[:, 2] > 1e-8)
    valid &= np.isfinite(projected).all(axis=1)
    projected, rectified = projected[valid], rectified[valid]
    uv = projected[:, :2] / projected[:, 2:3]
    inside = (uv[:, 0] >= 0) & (uv[:, 0] < width) & (uv[:, 1] >= 0) & (uv[:, 1] < height)
    return np.column_stack((uv[inside], rectified[inside, 2]))


def detect_objects(image, model, conf=0.25, device='cpu', imgsz=640):
    result = model.predict(source=image, conf=conf, device=device, imgsz=imgsz, verbose=False)[0]
    if result.boxes is None:
        return []
    boxes = result.boxes
    return [dict(class_id=int(cls), class_name=str(result.names[int(cls)]),
                 confidence=float(score), bbox_xyxy=[float(x) for x in box])
            for box, cls, score in zip(boxes.xyxy.cpu().numpy(), boxes.cls.cpu().numpy(),
                                       boxes.conf.cpu().numpy())]


def estimate_depth(boxes, projected_points, min_points=3, box_margin=0.0):
    results = []
    u, v, z = projected_points.T
    for detection in boxes:
        x1, y1, x2, y2 = detection['bbox_xyxy']
        dx, dy = (x2-x1)*box_margin, (y2-y1)*box_margin
        roi = [x1+dx, y1+dy, x2-dx, y2-dy]
        depths = z[(u >= roi[0]) & (u <= roi[2]) & (v >= roi[1]) & (v <= roi[3])]
        enough = len(depths) >= min_points
        results.append({**detection, 'depth_roi_xyxy': roi, 'num_points': int(len(depths)),
                        'depth_m': float(np.median(depths)) if enough else None,
                        'depth_iqr_m': float(np.percentile(depths, 75)-np.percentile(depths, 25)) if enough else None,
                        'status': 'ok' if enough else 'insufficient_points'})
    return results


def draw_projection(image, projected, color_max_depth):
    canvas = image.copy()
    draw = ImageDraw.Draw(canvas)
    for u, v, depth in projected[np.argsort(projected[:, 2])[::-1]]:
        hue = min(depth / color_max_depth, 1.0) * 2 / 3
        color = tuple(round(c*255) for c in colorsys.hsv_to_rgb(hue, 1, 1))
        draw.ellipse((u-1, v-1, u+1, v+1), fill=color)
    draw.rectangle((0, 0, min(canvas.width, 560), 20), fill='black')
    draw.text((5, 4), f'LiDAR camera Z: red=near, blue>={color_max_depth:g}m', fill='white')
    return canvas


def draw_detections(canvas, detections):
    canvas = canvas.copy()
    draw = ImageDraw.Draw(canvas)
    for obj in detections:
        box = obj['bbox_xyxy']
        draw.rectangle(box, outline='lime', width=2)
        depth = 'N/A' if obj['depth_m'] is None else f'{obj["depth_m"]:.2f}m'
        text = f'{obj["class_name"]} {obj["confidence"]:.2f} Z={depth} n={obj["num_points"]}'
        x, y = max(0, box[0]), max(22, box[1]-15)
        draw.rectangle(draw.textbbox((x, y), text), fill='black')
        draw.text((x, y), text, fill='lime')
    return canvas


def transform_matrix(translation, quaternion):
    """ROS xyzw quaternion and translation -> target-from-source 3x4 matrix."""
    q = np.asarray(quaternion, dtype=float)
    norm = np.linalg.norm(q)
    if not np.isfinite(q).all() or norm < 1e-12 or not np.isfinite(translation).all():
        raise ValueError('Invalid TF translation/quaternion')
    x, y, z, w = q / norm
    rotation = np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    return np.column_stack((rotation, translation))
