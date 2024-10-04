

---


# Fun 4
clone folder ลงไปหน้า home ใช้คำสั่งดังนี้:

```bash
cd
git clone https://github.com/Phetzxc/example_description.git
cd example_description
colcon build
source install/setup.bash
```
## Part 1: Setup Environment (2 คะแนน)

### 1. หา workspace ของแขนกลพร้อมวิธีการตรวจสอบคําตอบ (1 คะแนน)
แสดง workspace ของแขนกล ใช้คำสั่งดังนี้:

```bash
ros2 run example_description workspace.py 
```




### 2. Node สำหรับสุ่มเป้าหมาย (0.5 คะแนน)
สร้าง node ที่สุ่มตำแหน่งเป้าหมายของแขนกลภายใน workspace และ pub topic `/target`:
เพื่อแสดง node โดยตรง (ถ้าไม่อยู่ในโหมด Auto ตำแหน่งจะไม่สุ่ม):
```bash
ros2 run example_description random_target.py 
```

เพื่อดูตำแหน่งของเป้าหมายใน **RVIZ2** ให้ใช้คำสั่ง:

```bash
ros2 topic echo /target
```

### 3. Node สำหรับส่งค่าตําแหน่งปลายมือ (0.5 คะแนน)
ส่งตำแหน่งปลายมือของแขนกลไปยัง topic `/end_effector`:

```bash
ros2 topic echo /end_effector
```

ทั้งตำแหน่งของ `/target` และ `/end_effector` จะถูกแสดงใน **RVIZ2** 

---

## ส่วนที่ 2: Controller (7 คะแนน)

แขนกลสามารถทำงานได้ใน 3 โหมด:

- **Inverse Pose Kinematics (IPK)** 
- **Teleoperation (Teleop)** 
- **Autonomous (Auto)**

### 1. เปิดใช้งาน Node

```bash
ros2 launch example_description Fun4.launch.py 
```







### 2. Control mode

#### mode IPK : Inverse Pose Kinematics – (2 คะแนน)
ในโหมดนี้ระบบจะคำนวณตำแหน่งของข้อต่อ (joint) เพื่อให้ไปถึงตำแหน่งที่กำหนดใน task-space

- ตัวอย่างคำสั่ง:
  ```bash
  ros2 service call /change_mode fun4_interfaces/srv/Controlmode "{mode: 'IPK', x: 0.0, y: 0.4, z: 0.1}"
  ```

- หากพบคําตอบ ระบบจะ return response ของ service ออกแบบเป็น success=True และระบบจะส่งคำสั่งให้แขนกลเคลื่อนที่ไปยังเป้าหมาย พร้อมส่งค่า solution กลับมา
- หากไม่พบคําตอบ ระบบ return response ของ service ออกแบบเป็น success=False กลับมา และแขนกลจะไม่เคลื่อนที่ไปไหน



#### mode Auto : Auto – (2 คะแนน)
ในโหมดนี้แขนกลจะเคลื่อนที่ไปยังตำแหน่งที่สุ่มได้ภายใน workspace โดยมีเวลา 10 วินาทีเพื่อไปถึงเป้าหมาย

- คำสั่ง:
  ```bash
  ros2 service call /change_mode fun4_interfaces/srv/Controlmode "{mode: 'Auto'}"
  ```

หากไปถึงเป้าหมายสำเร็จระบบ response กลับไปที่ node สําหรับการสุ่ม และเริ่มการสุ่มเป้าหมายใหม่ต่อไป

#### mode Teleop : Teleoperation – (2 คะแนน) (mode นี้ไม่ได้ทำงานรวมกับอีกสองโหมด ต้องรันแยกออกมา)
ในโหมดนี้ผู้ใช้สามารถควบคุมแขนกลด้วยมือผ่าน topic `/cmd_vel` (Twist message) โดยมีการควบคุม 2 แบบ:

- **Reference จากปลายมือ**
- **Reference จากฐานของแขนกล**

### เปิดใช้งาน Node
```bash
ros2 launch example_description teleop_mode.launch.py  
  ```
  หลัง  launch จะมีหน้าต่างเด้งขึ้นมาเพื่อให้ใช้ควบคุมทิศทางด้วยปุ่ม:

- `a`: เคลื่อนที่ตามแกน +x
-  `s`:เคลื่อนที่ตามแกน +y
- `d`: เคลื่อนที่ตามแกน +z 
- `f`: เคลื่อนที่ตามแกน -x
- `g`: เคลื่อนที่ตามแกน -y
- `h`: เคลื่อนที่ตามแกน -z

- คำสั่ง mode Reference จากปลายมือ :
  ```bash
  ros2 service call /change_mode fun4_interfaces/srv/Controlmode "{mode: 'Teleop', x: 0}"
  ```
- คำสั่ง mode Reference จากฐานของแขนกล:
  ```bash
  ros2 service call /change_mode fun4_interfaces/srv/Controlmode "{mode: 'Teleop', x: 1}"
  ```



หุ่นยนต์จะหยุดเคลื่อนที่ทันทีหากตรวจพบว่าสภาพการเคลื่อนที่เข้าสู่ singularity และจะมีการแจ้งเตือนผ่านข้อความ











