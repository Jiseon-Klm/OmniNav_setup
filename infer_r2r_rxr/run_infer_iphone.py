#!/usr/bin/env python3
"""
iPhone 데이터를 사용한 OmniNav 추론
infer_r2r_rxr의 evaluate_agent를 베이스로 하되,
simulator로부터 이미지를 받는 부분만 iPhone 데이터 읽기로 변경
"""
import numpy as np
import argparse
import torch
import os
import csv
import json
import sys
from pathlib import Path
from tqdm import trange
import cv2
from PIL import Image
from datetime import datetime

from agent.waypoint_agent import Waypoint_Agent

# 맵 크기 및 설정
MAP_SIZE = 1024  # 맵 해상도
MAP_METERS_PER_PIXEL = 0.05  # 1픽셀 = 0.05m
MAP_CENTER = (MAP_SIZE // 2, MAP_SIZE // 2)  # 맵 중심


def load_instruction(instruction_path):
    """instruction.txt 읽기"""
    with open(instruction_path, 'r', encoding='utf-8') as f:
        return f.read().strip()


def load_odometry(odom_path):
    """odometry.csv 읽기 및 pose 변환
    
    카메라 좌표계 (iPhone): X=오른쪽, Y=아래, Z=앞방향
    Habitat/로봇 좌표계: X=앞방향, Y=왼쪽, Z=위
    
    변환: 카메라 Z축이 전진이므로 position을 그대로 사용
    """
    poses = []
    with open(odom_path, 'r') as f:
        reader = csv.reader(f)
        header = next(reader)  # timestamp, frame, x, y, z, qx, qy, qz, qw
        
        for row in reader:
            try:
                frame = int(row[1])
                x_cam = float(row[2])
                y_cam = float(row[3])
                z_cam = float(row[4])
                qx = float(row[5])
                qy = float(row[6])
                qz = float(row[7])
                qw = float(row[8])
                
                # 카메라 좌표계를 Habitat 좌표계로 변환
                # 카메라: (x, y, z) = (오른쪽, 아래, 앞방향)
                # Habitat: (x, y, z) = (앞방향, 왼쪽, 위)
                # 카메라 Z가 전진이므로 -> Habitat X
                # 카메라 X가 오른쪽이므로 -> Habitat -Y
                # 카메라 Y가 아래이므로 -> Habitat Z
                
                # 일단 position은 그대로 사용 (나중에 좌표계 변환 필요하면 수정)
                position = [x_cam, y_cam, z_cam]
                rotation = [qw, qx, qy, qz]  # Habitat은 (w, x, y, z) 순서
                
                poses.append({
                    'frame': frame,
                    'position': position,
                    'rotation': rotation,
                    'raw': (x_cam, y_cam, z_cam, qx, qy, qz, qw)
                })
            except (ValueError, IndexError) as e:
                print(f"[WARN] Skipping invalid row: {row}, error: {e}")
                continue
    
    return poses


def get_rgb_images(rgb_dir):
    """RGB 이미지 목록 가져오기 (정렬)"""
    rgb_path = Path(rgb_dir)
    image_files = sorted(rgb_path.glob("*.png"))
    return [str(p) for p in image_files]


def load_rgb_image(image_path):
    """RGB 이미지 로드 (BGR -> RGB 변환)"""
    img_bgr = cv2.imread(image_path)
    if img_bgr is None:
        raise ValueError(f"Failed to load image: {image_path}")
    img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
    return img_rgb


def create_fake_observations(frame_idx, image_path, instruction, pose):
    """Simulator observations를 모방한 딕셔너리 생성
    
    iPhone 데이터는 front 이미지만 있으므로, left/right는 front로 복제
    """
    rgb = load_rgb_image(image_path)
    
    # iPhone 데이터는 front만 있음, left/right는 front로 복제
    # (실제로는 모델이 'special_token' 모드일 때만 left/right 사용)
    observations = {
        'front': rgb,
        'left': rgb.copy(),  # 일단 front로 복제
        'right': rgb.copy(),  # 일단 front로 복제
        'rgb': rgb,
        'instruction': {'text': instruction},
        'pose': {
            'position': pose['position'],
            'rotation': pose['rotation']
        }
    }
    
    return observations


def world_to_map_coords(world_pos, map_center, meters_per_pixel):
    """월드 좌표를 맵 픽셀 좌표로 변환
    
    Args:
        world_pos: (x, y) 월드 좌표 (로봇 좌표계: x=좌우, y=전진)
        map_center: (cx, cy) 맵 중심 픽셀 좌표
        meters_per_pixel: 미터당 픽셀 수
    """
    x_world, y_world = world_pos
    # 로봇 좌표계: x=좌우(왼쪽=-, 오른쪽=+), y=전진(+)
    # 맵 좌표계: x=열(왼쪽=0), y=행(위=0)
    # 월드 x가 증가하면 맵 x도 증가, 월드 y가 증가하면 맵 y는 감소 (맵이 위에서 본 뷰)
    map_x = map_center[0] + int(x_world / meters_per_pixel)
    map_y = map_center[1] - int(y_world / meters_per_pixel)  # y축 뒤집기
    
    return (map_x, map_y)


def create_topdown_map_gt_only(odom_poses, current_idx, map_center, meters_per_pixel):
    """흰 바탕 맵에 GT 경로만 그린 이미지 생성
    
    Args:
        odom_poses: Odometry poses 리스트
        current_idx: 현재 프레임 인덱스
        map_center: 맵 중심 픽셀 좌표 (cx, cy)
        meters_per_pixel: 미터당 픽셀 수
    """
    map_img = np.zeros((MAP_SIZE, MAP_SIZE), dtype=np.uint8)
    
    # GT 경로 그리기 (odometry에서 현재까지의 경로)
    if current_idx >= 0 and len(odom_poses) > 0:
        gt_path_map_coords = []
        for i in range(min(current_idx + 1, len(odom_poses))):
            pose = odom_poses[i]
            world_x = pose['position'][0]
            world_y = pose['position'][2]
            
            if i == 0:
                origin = (world_x, world_y)
            
            rel_x = world_x - origin[0]
            rel_y = world_y - origin[1]
            
            map_coord = world_to_map_coords((rel_x, rel_y), map_center, meters_per_pixel)
            gt_path_map_coords.append(map_coord)
        
        # GT 경로 선 그리기 (검은색)
        for i, coord in enumerate(gt_path_map_coords):
            x, y = coord
            if 0 <= x < MAP_SIZE and 0 <= y < MAP_SIZE:
                map_img[y, x] = 1  # MAP_VALID_POINT
                cv2.circle(map_img, (x, y), 3, 10, -1)  # 10 = MAP_REFERENCE_POINT (검은색)
                
                if i > 0:
                    prev_coord = gt_path_map_coords[i-1]
                    cv2.line(map_img, prev_coord, coord, 10, 2)  # 검은색
    
    return map_img


def create_topdown_map_pred_only(pred_path, map_center, meters_per_pixel):
    """흰 바탕 맵에 예측 경로만 그린 이미지 생성
    
    Args:
        pred_path: 예측 경로 좌표 리스트 [(x, y), ...]
        map_center: 맵 중심 픽셀 좌표 (cx, cy)
        meters_per_pixel: 미터당 픽셀 수
    """
    map_img = np.zeros((MAP_SIZE, MAP_SIZE), dtype=np.uint8)
    
    # 예측 경로 그리기
    if len(pred_path) > 0:
        pred_path_map_coords = []
        for pos in pred_path:
            map_coord = world_to_map_coords(pos, map_center, meters_per_pixel)
            pred_path_map_coords.append(map_coord)
        
        # 예측 경로 선 그리기 (노란색)
        for i, coord in enumerate(pred_path_map_coords):
            x, y = coord
            if 0 <= x < MAP_SIZE and 0 <= y < MAP_SIZE:
                map_img[y, x] = 1  # MAP_VALID_POINT
                cv2.circle(map_img, (x, y), 3, 12, -1)  # 12 = MAP_WAYPOINT_PREDICTION (노란색)
                
                if i > 0:
                    prev_coord = pred_path_map_coords[i-1]
                    cv2.line(map_img, prev_coord, coord, 12, 2)  # 노란색
    
    return map_img


def create_topdown_map_with_path(odom_poses, current_idx, pred_path=[]):
    """흰 바탕 맵에 GT 경로와 예측 경로를 그린 이미지 생성 (레거시, 이제는 사용 안 함)
    
    새로운 방식: GT와 예측을 별도로 그려서 세로로 합침
    """
    # 이 함수는 더 이상 사용하지 않지만, 호환성을 위해 유지
    # create_fake_info를 사용하세요
    info = create_fake_info(odom_poses, current_idx, pred_path)
    return info['gt_map']


def create_fake_info(odom_poses, current_idx, pred_path=[]):
    """Simulator info를 모방한 딕셔너리 생성
    
    iPhone 데이터에는 topdown map이 없으므로 흰 바탕에 경로를 그린 맵 생성
    GT와 예측 경로를 별도로 생성하되, 두 경로의 스케일을 맞춤
    """
    # 1. GT 경로와 예측 경로의 모든 좌표 수집
    all_coords_x = []
    all_coords_y = []
    
    # GT 경로 좌표 수집
    if current_idx >= 0 and len(odom_poses) > 0:
        origin = None
        for i in range(min(current_idx + 1, len(odom_poses))):
            pose = odom_poses[i]
            world_x = pose['position'][0]
            world_y = pose['position'][2]
            
            if i == 0:
                origin = (world_x, world_y)
            
            rel_x = world_x - origin[0]
            rel_y = world_y - origin[1]
            all_coords_x.append(rel_x)
            all_coords_y.append(rel_y)
    
    # 예측 경로 좌표 수집
    if len(pred_path) > 0:
        for pos in pred_path:
            all_coords_x.append(pos[0])
            all_coords_y.append(pos[1])
    
    # 2. 좌표 범위 계산 (마진 포함)
    if len(all_coords_x) > 0 and len(all_coords_y) > 0:
        min_x, max_x = min(all_coords_x), max(all_coords_x)
        min_y, max_y = min(all_coords_y), max(all_coords_y)
        
        # 마진 추가 (경로가 맵 가장자리에 닿지 않도록)
        margin = 2.0  # 미터 단위
        range_x = max(max_x - min_x, 0.1) + 2 * margin
        range_y = max(max_y - min_y, 0.1) + 2 * margin
        
        # 맵 범위에 맞는 스케일 계산
        map_range_meters = min(range_x, range_y)
        if map_range_meters > 0:
            # 맵 크기의 80%를 사용하도록 스케일 조정
            available_pixels = MAP_SIZE * 0.8
            meters_per_pixel = map_range_meters / available_pixels
        else:
            meters_per_pixel = MAP_METERS_PER_PIXEL
        
        # 맵 중심 계산 (GT 경로 시작점이 맵 중심이 되도록)
        if current_idx >= 0 and len(odom_poses) > 0:
            center_world_x = 0.0  # GT 원점 기준
            center_world_y = 0.0
        else:
            # 예측 경로만 있는 경우
            center_world_x = (min_x + max_x) / 2
            center_world_y = (min_y + max_y) / 2
        
        map_center = (MAP_SIZE // 2, MAP_SIZE // 2)
    else:
        # 경로가 없는 경우 기본값 사용
        map_center = MAP_CENTER
        meters_per_pixel = MAP_METERS_PER_PIXEL
    
    # 3. 같은 스케일로 GT와 예측 맵 생성
    gt_map = create_topdown_map_gt_only(odom_poses, current_idx, map_center, meters_per_pixel)
    pred_map = create_topdown_map_pred_only(pred_path, map_center, meters_per_pixel)
    
    # 두 맵을 세로로 합치기 (GT 위, 예측 아래)
    if gt_map.shape == pred_map.shape:
        combined_map = np.concatenate([gt_map, pred_map], axis=0)
    else:
        # 크기가 다르면 높이 맞춤
        target_w = max(gt_map.shape[1], pred_map.shape[1])
        gt_map_resized = cv2.resize(gt_map, (target_w, gt_map.shape[0]), interpolation=cv2.INTER_NEAREST)
        pred_map_resized = cv2.resize(pred_map, (target_w, pred_map.shape[0]), interpolation=cv2.INTER_NEAREST)
        combined_map = np.concatenate([gt_map_resized, pred_map_resized], axis=0)
    
    return {
        'top_down_map_vlnce': combined_map,
        'gt_map': gt_map,  # GT 맵 따로 보관
        'pred_map': pred_map,  # 예측 맵 따로 보관
    }


def evaluate_iphone_data(data_dir, model_path, result_path, max_frames=0):
    """iPhone 데이터로 추론 실행
    
    Args:
        data_dir: iPhone 데이터 디렉토리 (instruction.txt, odometry.csv, rgb/ 포함)
        model_path: OmniNav 모델 경로
        result_path: 결과 저장 경로
        max_frames: 최대 프레임 수 (0=전체)
    """
    data_dir = Path(data_dir)
    
    # 1. Instruction 로드
    instruction_path = data_dir / 'instruction.txt'
    instruction = load_instruction(instruction_path)
    print(f"[INFO] Instruction: {instruction}")
    
    # 2. Odometry 로드
    odom_path = data_dir / 'odometry.csv'
    odom_poses = load_odometry(odom_path)
    print(f"[INFO] Loaded {len(odom_poses)} odometry poses")
    
    # 3. RGB 이미지 목록 가져오기
    rgb_dir = data_dir / 'rgb'
    image_files = get_rgb_images(rgb_dir)
    print(f"[INFO] Found {len(image_files)} RGB images")
    
    if max_frames > 0:
        image_files = image_files[:max_frames]
        odom_poses = odom_poses[:max_frames]
        print(f"[INFO] Limited to {len(image_files)} frames")
    
    # 4. Agent 초기화
    model_name = '/'.join(model_path.split('/')[-3:])
    result_path_full = os.path.join(result_path, model_name)
    
    # 로그 파일 설정 (result 폴더 내)
    log_dir = os.path.join(result_path_full, "log")
    os.makedirs(log_dir, exist_ok=True)
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    log_file = os.path.join(log_dir, f"inference_{timestamp}.log")
    
    # stdout/stderr를 로그 파일과 콘솔에 동시에 출력하도록 설정
    class Tee:
        def __init__(self, *files):
            self.files = files
        def write(self, obj):
            for f in self.files:
                f.write(obj)
                f.flush()
        def flush(self):
            for f in self.files:
                f.flush()
    
    original_stdout = sys.stdout
    original_stderr = sys.stderr
    log_f = open(log_file, 'w', encoding='utf-8')
    sys.stdout = Tee(sys.stdout, log_f)
    sys.stderr = Tee(sys.stderr, log_f)
    
    print(f"[INFO] Log file: {log_file}")
    print(f"[INFO] All output will be saved to log file")
    
    # require_map=True로 설정하여 map_vis 생성
    agent = Waypoint_Agent(model_path, result_path_full, require_map=True)
    
    # 5. 에피소드 ID (현재는 하나의 에피소드로 간주)
    episode_id = data_dir.name  # 예: 'a1af0cece0'
    
    agent.reset()
    agent.episode_id = episode_id
    
    # 6. 프레임별 추론 실행
    num_frames = len(image_files)
    
    results = []
    pred_path = []  # 예측 경로 누적
    curr_pos = np.array([0.0, 0.0])  # 현재 위치 (로봇 좌표계)
    
    for i in trange(num_frames, desc=f"Processing {episode_id}"):
        image_path = image_files[i]
        frame_idx = int(Path(image_path).stem)
        
        # Odometry에서 해당 프레임의 pose 찾기
        pose = None
        for p in odom_poses:
            if p['frame'] == frame_idx:
                pose = p
                break
        
        if pose is None:
            print(f"[WARN] No pose found for frame {frame_idx}, using default")
            pose = {
                'position': [0.0, 0.0, 0.0],
                'rotation': [1.0, 0.0, 0.0, 0.0]
            }
        
        # Observations 생성
        obs = create_fake_observations(frame_idx, image_path, instruction, pose)
        
        # Info 생성 (topdown map에 경로 그리기, 이전 프레임까지의 예측 경로 사용)
        # 현재 프레임의 예측은 아직 모르므로 이전까지의 경로만 표시
        info = create_fake_info(odom_poses, i, pred_path)
        
        # Agent act (추론) - info를 전달하여 맵 시각화
        with torch.no_grad():
            action = agent.act(obs, info, episode_id)
        
        # 예측 경로 누적 (action에서 waypoint 추출하여 다음 프레임에서 사용)
        if 'action' in action and len(action['action']) > 0:
            waypoint = action['action'][0]  # 첫 번째 waypoint
            # waypoint는 로컬 좌표계 (x, y)
            curr_pos = curr_pos + np.array([waypoint[0], waypoint[1]]) * 0.3  # PREDICT_SCALE = 0.3
            pred_path.append(curr_pos.copy())
        
        # 결과 저장
        result = {
            'frame': frame_idx,
            'action': action,
            'pose': pose
        }
        results.append(result)
        
        # Action 출력 (참고용)
        if 'arrive_pred' in action:
            log_parts = [f"arrive={action['arrive_pred']}"]
            
            # Heading (recover_angle) - Log all 5 values
            if 'recover_angle' in action:
                recover_angle = action['recover_angle']
                if isinstance(recover_angle, np.ndarray):
                    # Convert to degrees for better readability
                    recover_angle_deg = np.degrees(recover_angle)
                    # Flatten if needed
                    if recover_angle_deg.ndim > 1:
                        recover_angle_deg = recover_angle_deg.flatten()
                    
                    # Format as list string
                    angle_str = "[" + ", ".join([f"{x:.2f}°" for x in recover_angle_deg]) + "]"
                    log_parts.append(f"heading={angle_str}")
                else:
                    log_parts.append(f"heading={np.degrees(recover_angle):.2f}°")
            
            # Sin/Cos - Log all values
            if 'sin_angle' in action:
                sin_vals = action['sin_angle']
                if isinstance(sin_vals, np.ndarray):
                    if sin_vals.ndim > 1: sin_vals = sin_vals.flatten()
                    sin_str = "[" + ", ".join([f"{x:.4f}" for x in sin_vals]) + "]"
                    log_parts.append(f"sin={sin_str}")
                else:
                    log_parts.append(f"sin={sin_vals:.4f}")
                    
            if 'cos_angle' in action:
                cos_vals = action['cos_angle']
                if isinstance(cos_vals, np.ndarray):
                    if cos_vals.ndim > 1: cos_vals = cos_vals.flatten()
                    cos_str = "[" + ", ".join([f"{x:.4f}" for x in cos_vals]) + "]"
                    log_parts.append(f"cos={cos_str}")
                else:
                    log_parts.append(f"cos={cos_vals:.4f}")

            # Waypoint - Log first one as before
            if 'action' in action and len(action['action']) > 0:
                wp = action['action'][0]
                log_parts.append(f"wp[0]=({wp[0]:.4f}, {wp[1]:.4f})")
            
            print(f"Frame {frame_idx}: " + ", ".join(log_parts))
    
    # 7. Agent reset (GIF 저장)
    agent.reset()
    
    # 8. 결과 저장 (JSON) - log_dir는 이미 정의됨
    results_file = os.path.join(log_dir, f"stats_{episode_id}.json")
    
    # 간단한 통계만 저장
    stats = {
        'id': episode_id,
        'num_frames': num_frames,
        'instruction': instruction
    }
    
    with open(results_file, 'w') as f:
        json.dump(stats, f, indent=4)
    
    print(f"\n[INFO] Results saved to: {results_file}")
    print(f"[INFO] Visualization saved to: {result_path_full}")
    print(f"[INFO] Log file: {log_file}")
    
    # stdout/stderr 복원
    sys.stdout = original_stdout
    sys.stderr = original_stderr
    log_f.close()
    
    return results


def main():
    parser = argparse.ArgumentParser(description='iPhone 데이터 OmniNav 추론')
    
    parser.add_argument(
        "--data-dir",
        type=str,
        required=True,
        help="iPhone 데이터 디렉토리 (instruction.txt, odometry.csv, rgb/ 포함)"
    )
    
    parser.add_argument(
        "--model-path",
        type=str,
        required=True,
        help="OmniNav 모델 경로"
    )
    
    parser.add_argument(
        "--result-path",
        type=str,
        required=False,
        default="./data/result_iphone",
        help="결과 저장 경로"
    )
    
    parser.add_argument(
        "--max-frames",
        type=int,
        required=False,
        default=0,
        help="최대 프레임 수 (0=전체)"
    )
    
    args = parser.parse_args()
    
    evaluate_iphone_data(
        args.data_dir,
        args.model_path,
        args.result_path,
        args.max_frames
    )


if __name__ == "__main__":
    main()

