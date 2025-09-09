# 🌀 Smart Fan (Prototype)
 
> 임베디드시스템 Term Project (2025년 1학기)

---

## 📌 프로젝트 개요

여름철 선풍기 사용 중 타이머 미설정으로 인해 새벽에 깨어나는 불편함을 떠올려,  
**버튼/센서 기반 자동 제어 선풍기 시스템**을 구현했습니다.  
RTOS 기반 태스크 구조를 통해 사용자 최소 조작으로 선풍기 On/Off, 풍속 조절, 자동 꺼짐까지 수행됩니다.

---

## ⚙️ 주요 기능 요약

| 기능 | 설명 |
|------|------|
| 전원 버튼 | On/Off |
| 풍속 조절 | 1단 → 2단 → 3단 (회전 속도: 60% → 80% → 100%) |
| 타이머 설정 | BTN3 입력마다 3초 증가 (최대 12초) |
| 초음파 거리 센서 | 사용자와의 거리 30cm 이상 → 자동 종료 |
| LED 상태 표시 | 풍속별 색상 / 타이머 동작 중 점멸 |

※ 타이머는 시연을 위해 짧게 설정하였습니다

---

## 🔧 사용한 디바이스

- **STM32F429ZI** 보드
- **3색 RGB LED 모듈**
- **디지털 버튼 x3**
- **초음파 거리 센서 (HC-SR04)**
- **5V DC 모터 + L298N 드라이버**
- **uC/OS-III RTOS**

---

## 📄 Documentation

For a detailed explanation of the system architecture, task design, and implementation,  
please refer to the following documents included in this repository:

This project is based on the **NUCLEO-F429ZI example code** and was implemented by modifying only the `app.c` file.
- 📄 [Report.pdf](./Report.pdf)
- 📄 [app.c](./app.c)
