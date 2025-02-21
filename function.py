import cv2
import threading
from pymodbus.client import ModbusSerialClient
from pymodbus.exceptions import ModbusException
import numpy as np

def connect_to_plc(port, baudrate, timeout):
    client = ModbusSerialClient(
        port=port,
        baudrate=baudrate,
        parity='N',
        stopbits=1,
        bytesize=8,
        timeout=timeout
    )
    if client.connect():
        return client
    else:
        print("Kết nối thất bại!")
        return None

# Hàm tính inverse kinematics
def inverse_kinematics(px, py, pz):
    l0 = 0
    l1 = 127.7
    l2 = 140
    l3 = 140
    
    a = px - l0
    if a == 0:
        the1_1 = the1_2 = the1_3 = the1_4 = 90
    else:
        the1_1 = np.degrees(np.arctan2(py, px - l0))
        the1_2 = the1_1
        the1_3 = np.degrees(np.arctan2(-py, -(px - l0)))
        the1_4 = the1_3
    
    def solve_angles(t1):
        A = px * np.cos(np.radians(t1)) + py * np.sin(np.radians(t1)) - l0 * np.cos(np.radians(t1))
        B = pz - l1
        G = A**2 + B**2 - l2**2 - l3**2
        c3 = G / (2 * l2 * l3)
        
        if c3 >= 1:
            s3_1 = s3_2 = 0
        else:
            s3_1 = np.sqrt(1 - c3**2)
            s3_2 = -np.sqrt(1 - c3**2)
        
        the3_1 = np.degrees(np.arctan2(s3_1, c3))
        the3_2 = np.degrees(np.arctan2(s3_2, c3))
        
        c2_1 = (A * (l2 + l3 * np.cos(np.radians(the3_1))) + l3 * B * np.sin(np.radians(the3_1))) / \
               ((l2 + l3 * np.cos(np.radians(the3_1)))**2 + l3**2 * (np.sin(np.radians(the3_1)))**2)
        s2_1 = (B * (l2 + l3 * np.cos(np.radians(the3_1))) - l3 * np.sin(np.radians(the3_1)) * A) / \
               ((l2 + l3 * np.cos(np.radians(the3_1)))**2 + l3**2 * (np.sin(np.radians(the3_1)))**2)
        
        the2_1 = np.degrees(np.arctan2(s2_1, c2_1))
        
        c2_2 = (A * (l2 + l3 * np.cos(np.radians(the3_2))) + l3 * B * np.sin(np.radians(the3_2))) / \
               ((l2 + l3 * np.cos(np.radians(the3_2)))**2 + l3**2 * (np.sin(np.radians(the3_2)))**2)
        s2_2 = (B * (l2 + l3 * np.cos(np.radians(the3_2))) - l3 * np.sin(np.radians(the3_2)) * A) / \
               ((l2 + l3 * np.cos(np.radians(the3_2)))**2 + l3**2 * (np.sin(np.radians(the3_2)))**2)
        
        the2_2 = np.degrees(np.arctan2(s2_2, c2_2))
        
        return (the2_1, the3_1), (the2_2, the3_2)
    
    sol1, sol2 = solve_angles(the1_1)
    sol3, sol4 = solve_angles(the1_3)
    
    solutions = [
        (the1_1, *sol1),
        (the1_2, *sol2),
        (the1_3, *sol3),
        (the1_4, *sol4)
    ]
    
    theta1_range = (0, 180)
    theta2_range = (0, 140)
    theta3_range = (-150, -30)
    home_position = (0, 145, -150)
    
    for q1, q2, q3 in solutions:
        if (theta1_range[0] <= q1 <= theta1_range[1] and
            theta2_range[0] <= q2 <= theta2_range[1] and
            theta3_range[0] <= q3 <= theta3_range[1]):
            delta_q1 = q1 - home_position[0]
            delta_q2 = home_position[1] - q2
            delta_q3 =  (home_position[2]+delta_q2) - q3
            pul_q1 = delta_q1/(9/728)
            pul_q2 = delta_q2/(9/728)
            pul_q3 = delta_q3/(9/728)
            return pul_q2, pul_q3
    
    return None
def read_from_plc(client, address, count, slave_id=1):
    try:
        result = client.read_holding_registers(address=address, count=count, slave=slave_id)
        if not result.isError():
            return result.registers
        else:
            print("Lỗi đọc thanh ghi:", result)
            return None
    except ModbusException as e:
        print(f"Lỗi Modbus: {e}")
        return None
    except Exception as e:
        print(f"Lỗi không xác định: {e}")
        return None
# Gửi giá trị đến PLC
def send_to_plc(client, address, values, slave_id=1):
    try:
        for i, value in enumerate(values):
            client.write_register(address=address + i, value=int(value), slave=slave_id)
        print("Gửi dữ liệu thành công!")
    except ModbusException as e:
        print(f"Lỗi Modbus: {e}")
    except Exception as e:
        print(f"Lỗi không xác định: {e}")

# Xác định màu sắc trong vùng quan tâm (ROI)
def detect_color(frame, roi_coords):
    x, y, w, h = roi_coords
    roi = frame[y:y+h, x:x+w]
    
    hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

    # Định nghĩa khoảng màu đỏ trong không gian HSV
    lower_red1 = np.array([0, 120, 70])
    upper_red1 = np.array([10, 255, 255])
    lower_red2 = np.array([170, 120, 70])
    upper_red2 = np.array([180, 255, 255])

    # Định nghĩa ngưỡng màu xanh dương
    lower_blue = np.array([100, 150, 0])
    upper_blue = np.array([140, 255, 255])

    mask_red = cv2.inRange(hsv_roi, lower_red1, upper_red1) + cv2.inRange(hsv_roi, lower_red2, upper_red2)
    mask_blue = cv2.inRange(hsv_roi, lower_blue, upper_blue)

    red_count = cv2.countNonZero(mask_red)
    blue_count = cv2.countNonZero(mask_blue)

    if red_count > blue_count and red_count > 50:  # Ngưỡng tối thiểu để phát hiện màu
        return "red"
    elif blue_count > red_count and blue_count > 50:
        return "blue"
    else:
        return "none"

# Gửi giá trị đến bit M10
def write_bit_to_plc(client, address, value, slave_id=1):
    try:
        client.write_coil(address=address, value=bool(value), slave=slave_id)
        print(f"Ghi giá trị {value} xuống bit {address}")
    except ModbusException as e:
        print(f"Lỗi Modbus: {e}")
    except Exception as e:
        print(f"Lỗi không xác định: {e}")

# Hàm xử lý động học ngược và gửi tín hiệu PLC
def kinematics_thread(client):
    while True:
        registers = read_from_plc(client, address=90, count=4, slave_id=1)
        if registers:
            SLA, SLB, SL_Red, SL_Blue = registers
            print(f"Đọc từ PLC: {registers}")

            values1 = inverse_kinematics(120, 120, (SLA) * 21+28)
            values2 = inverse_kinematics(120, 120, (SLB+1) * 22+34)
            
            if values1 and values2:
                print(f"Đọc từ PLC: {registers}")
                send_to_plc(client, address=103, values=values1, slave_id=1)
                send_to_plc(client, address=111, values=values2, slave_id=1)
                print(f"Gửi dữ liệu động học: {values1},{values2}")

# Hàm xử lý video
def camera_thread(client):
    cap = cv2.VideoCapture(0)
    roi_coords = (150, 150, 155, 155)  # Tọa độ vùng quan tâm
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Không thể đọc khung hình từ camera!")
            break

        color = detect_color(frame, roi_coords)
        if color == "red":
            write_bit_to_plc(client, address=400, value=1, slave_id=1)
        elif color == "blue":
            write_bit_to_plc(client, address=400, value=0, slave_id=1)

        x, y, w, h = roi_coords
        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
        cv2.putText(frame, f"Color: {color}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        #cv2.imshow("Camera", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

# # Thực thi chương trình
# def function():
#     client = connect_to_plc(port='COM6', baudrate=9600, timeout=0.5)
#     # if client:
#     # Tạo và chạy luồng xử lý động học
#     kinematics_thread_instance = threading.Thread(target=kinematics_thread, args=(client,))
#     kinematics_thread_instance.daemon = True
#     kinematics_thread_instance.start()

#     # Tạo và chạy luồng xử lý camera
#     camera_thread_instance = threading.Thread(target=camera_thread, args=(client,))
#     camera_thread_instance.daemon = True
#     camera_thread_instance.start()

#     # Chờ các luồng chạy
#     try:
#         while True:
#             pass  # Giữ chương trình chính hoạt động
#     except KeyboardInterrupt:
#         print("\nKết thúc chương trình!")
#         client.close()