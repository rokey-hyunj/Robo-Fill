# markers.py
import cv2
import numpy as np
from typing import Dict, Optional, Tuple, List

# ArUco 마커 딕셔너리 타입
ARUCO_DICT = cv2.aruco.DICT_4X4_50

# 고객 데이터베이스
CUSTOMER_DATABASE = {
    # ID: {연료 타입, 주유 위치(좌/우)}
    0: {"fuel_type": "gasoline", "oil_pose": "left", "customer_name": "홍길동", "vehicle_type": "sedan"},
    1: {"fuel_type": "diesel", "oil_pose": "right", "customer_name": "김철수", "vehicle_type": "suv"},
    2: {"fuel_type": "lpg", "oil_pose": "left", "customer_name": "이영희", "vehicle_type": "taxi"},
    3: {"fuel_type": "gasoline", "oil_pose": "right", "customer_name": "박지성", "vehicle_type": "sedan"},
    4: {"fuel_type": "diesel", "oil_pose": "left", "customer_name": "최민수", "vehicle_type": "truck"},
    5: {"fuel_type": "lpg", "oil_pose": "right", "customer_name": "정수아", "vehicle_type": "taxi"},
    
    # Benz 차량용
    10: {"fuel_type": "gasoline", "oil_pose": "left", "customer_name": "강대리", "vehicle_type": "benz"},
    11: {"fuel_type": "diesel", "oil_pose": "right", "customer_name": "윤사장", "vehicle_type": "benz"},
    
    # Taxi 차량용
    20: {"fuel_type": "lpg", "oil_pose": "left", "customer_name": "김기사", "vehicle_type": "taxi"},
    21: {"fuel_type": "lpg", "oil_pose": "right", "customer_name": "이기사", "vehicle_type": "taxi"},
    
    # Car 차량용
    30: {"fuel_type": "gasoline", "oil_pose": "left", "customer_name": "박과장", "vehicle_type": "car"},
    31: {"fuel_type": "diesel", "oil_pose": "right", "customer_name": "최부장", "vehicle_type": "car"},
}


def detect_aruco(image: np.ndarray, 
                 aruco_dict_type: int = ARUCO_DICT,
                 visualize: bool = False) -> Tuple[Optional[np.ndarray], Optional[np.ndarray]]:
    
    # ArUco 딕셔너리 및 파라미터 설정
    aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_type)
    parameters = cv2.aruco.DetectorParameters_create()
    
    # 감지 성능 향상을 위한 파라미터 튜닝
    parameters.adaptiveThreshWinSizeMin = 3
    parameters.adaptiveThreshWinSizeMax = 23
    parameters.adaptiveThreshWinSizeStep = 10
    parameters.minMarkerPerimeterRate = 0.03
    parameters.maxMarkerPerimeterRate = 4.0
    
    # 이미지가 컬러인 경우 그레이스케일로 변환
    if len(image.shape) == 3:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    else:
        gray = image.copy()
    
    # ArUco 마커 감지
    corners, ids, rejected_img_points = cv2.aruco.detectMarkers(
        gray, 
        aruco_dict, 
        parameters=parameters
    )
    
    # 시각화
    if visualize and ids is not None:
        output_image = image.copy()
        cv2.aruco.drawDetectedMarkers(output_image, corners, ids)
        cv2.imshow("ArUco Detection", output_image)
        cv2.waitKey(1)
    
    return ids, corners


def get_customer_by_aruco(aruco_id: int) -> Optional[Dict]:
    if aruco_id in CUSTOMER_DATABASE:
        return CUSTOMER_DATABASE[aruco_id]
    else:
        print(f"[WARNING] ArUco ID {aruco_id}에 해당하는 고객 정보가 없습니다.")
        return None


def estimate_marker_pose(corners: np.ndarray, 
                        marker_size: float,
                        camera_matrix: np.ndarray,
                        dist_coeffs: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:

    rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(
        corners,
        marker_size,
        camera_matrix,
        dist_coeffs
    )
    return rvec, tvec


def get_marker_center(corners: np.ndarray) -> Tuple[float, float]:
    if corners is None or len(corners) == 0:
        return None, None
    
    # 4개 코너의 평균을 계산하여 중심점 구하기
    corner_points = corners[0][0]  # shape: (4, 2)
    center_x = np.mean(corner_points[:, 0])
    center_y = np.mean(corner_points[:, 1])
    
    return center_x, center_y


def calculate_distance_to_marker(tvec: np.ndarray) -> float:
    if tvec is None:
        return None
    
    # 유클리드 거리 계산
    distance = np.linalg.norm(tvec)
    return distance


def draw_marker_info(image: np.ndarray, 
                     corners: np.ndarray, 
                     aruco_id: int,
                     customer_info: Optional[Dict] = None) -> np.ndarray:

    output = image.copy()
    
    if corners is None or len(corners) == 0:
        return output
    
    # 마커 테두리 그리기
    cv2.aruco.drawDetectedMarkers(output, corners, np.array([[aruco_id]]))
    
    # 중심점 계산
    center_x, center_y = get_marker_center(corners)
    
    if center_x is not None and center_y is not None:
        # 중심점 표시
        cv2.circle(output, (int(center_x), int(center_y)), 5, (0, 255, 0), -1)
        
        # 고객 정보 표시
        if customer_info:
            text = f"ID:{aruco_id} {customer_info['fuel_type']} {customer_info['oil_pose']}"
            cv2.putText(output, text, 
                       (int(center_x) - 50, int(center_y) - 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    return output


def process_frame(image: np.ndarray, 
                  visualize: bool = True,
                  save_path: Optional[str] = None) -> List[Dict]:

    detected_markers = []
    
    # ArUco 마커 감지
    ids, corners = detect_aruco(image, visualize=False)
    
    if ids is None:
        print("[INFO] 감지된 ArUco 마커가 없습니다.")
        return detected_markers
    
    output_image = image.copy() if visualize else None
    
    # 각 마커에 대해 정보 수집
    for i, marker_id in enumerate(ids.flatten()):
        marker_corners = corners[i]
        
        # 고객 정보 조회
        customer_info = get_customer_by_aruco(int(marker_id))
        
        # 중심점 계산
        center = get_marker_center(marker_corners)
        
        marker_data = {
            "id": int(marker_id),
            "corners": marker_corners,
            "center": center,
            "customer_info": customer_info
        }
        
        detected_markers.append(marker_data)
        
        # 시각화
        if visualize and output_image is not None:
            output_image = draw_marker_info(
                output_image, 
                marker_corners, 
                int(marker_id), 
                customer_info
            )
        
        # 정보 출력
        if customer_info:
            print(f"[INFO] ArUco ID {marker_id} 감지됨:")
            print(f"  - 고객명: {customer_info['customer_name']}")
            print(f"  - 차량: {customer_info['vehicle_type']}")
            print(f"  - 연료: {customer_info['fuel_type']}")
            print(f"  - 주유 위치: {customer_info['oil_pose']}")
            print(f"  - 중심 좌표: {center}")
    
    # 결과 이미지 저장
    if save_path and output_image is not None:
        cv2.imwrite(save_path, output_image)
        print(f"[INFO] 결과 이미지 저장: {save_path}")
    
    # 시각화
    if visualize and output_image is not None:
        cv2.imshow("ArUco Markers Detection", output_image)
        cv2.waitKey(1)
    
    return detected_markers


def add_customer(aruco_id: int, 
                fuel_type: str, 
                oil_pose: str,
                customer_name: str = "Unknown",
                vehicle_type: str = "sedan") -> bool:

    # 유효성 검사
    valid_fuel_types = ["gasoline", "diesel", "lpg"]
    valid_oil_poses = ["left", "right"]
    
    if fuel_type not in valid_fuel_types:
        print(f"[ERROR] 잘못된 연료 타입: {fuel_type}. 가능한 값: {valid_fuel_types}")
        return False
    
    if oil_pose not in valid_oil_poses:
        print(f"[ERROR] 잘못된 주유 위치: {oil_pose}. 가능한 값: {valid_oil_poses}")
        return False
    
    # 데이터베이스에 추가
    CUSTOMER_DATABASE[aruco_id] = {
        "fuel_type": fuel_type,
        "oil_pose": oil_pose,
        "customer_name": customer_name,
        "vehicle_type": vehicle_type
    }
    
    print(f"[INFO] 고객 정보 추가 완료: ID={aruco_id}, {customer_name}, {fuel_type}, {oil_pose}")
    return True


def generate_aruco_marker(marker_id: int, 
                          marker_size: int = 200,
                          save_path: Optional[str] = None) -> np.ndarray:

    aruco_dict = cv2.aruco.Dictionary_get(ARUCO_DICT)
    
    # 마커 생성
    marker_image = cv2.aruco.drawMarker(aruco_dict, marker_id, marker_size)
    
    # 저장
    if save_path:
        cv2.imwrite(save_path, marker_image)
        print(f"[INFO] ArUco 마커 생성 완료: ID={marker_id}, Path={save_path}")
    
    return marker_image


if __name__ == "__main__":
    print("=" * 60)
    print("ArUco Marker Detection System")
    print("=" * 60)
    
    # 1. 마커 생성 예제
    print("\n[1] ArUco 마커 생성 테스트")
    for marker_id in [0, 1, 2, 10, 20, 30]:
        marker_img = generate_aruco_marker(
            marker_id, 
            marker_size=200, 
            save_path=f"aruco_marker_{marker_id}.png"
        )
    
    # 2. 고객 정보 조회 테스트
    print("\n[2] 고객 정보 조회 테스트")
    for test_id in [0, 1, 2, 10, 999]:
        info = get_customer_by_aruco(test_id)
        if info:
            print(f"ID {test_id}: {info}")
    
    # 3. 새 고객 추가 테스트
    print("\n[3] 새 고객 추가 테스트")
    add_customer(
        aruco_id=100,
        fuel_type="gasoline",
        oil_pose="left",
        customer_name="테스트고객",
        vehicle_type="sedan"
    )
    
    print("\n[4] 데이터베이스 전체 출력")
    print(f"총 {len(CUSTOMER_DATABASE)}명의 고객 정보:")
    for marker_id, info in sorted(CUSTOMER_DATABASE.items()):
        print(f"  ID {marker_id:3d}: {info['customer_name']:10s} | "
              f"{info['vehicle_type']:8s} | {info['fuel_type']:8s} | {info['oil_pose']}")
    
    print("\n" + "=" * 60)
    print("테스트 완료!")
    print("=" * 60)