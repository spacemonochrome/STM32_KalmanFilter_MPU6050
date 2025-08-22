
# STM32 MPU6050 6DOF Kalman Filter

Bu proje, STM32F103C8T6 (Blue Pill) mikrodenetleyicisi ile MPU6050 ivmeölçer/jyroskop sensöründen alınan verilerle 6 serbestlik dereceli (6DOF) hareketin Kalman filtresi ile hesaplanmasını sağlar. Kodlar CubeIDE ile geliştirilmiştir.

## Özellikler
- STM32F103C8T6 (Blue Pill) ile MPU6050 sensöründen I2C üzerinden veri okuma
- Kalman filtresi ile açısal verilerin (pitch, roll, yaw) hesaplanması
- C dili ile yazılmış, CubeIDE/CubeMX uyumlu
- Kolayca başka STM32 projelerine entegre edilebilir

## Donanım Bağlantısı
- **MPU6050 VCC** → 3.3V (veya 5V)
- **MPU6050 GND** → GND
- **MPU6050 SCL** → PB6 (I2C1_SCL)
- **MPU6050 SDA** → PB7 (I2C1_SDA)

## Derleme ve Kullanım
1. Projeyi STM32CubeIDE ile açın.
2. Gerekli bağlantıları yapın ve kartınızı bilgisayara bağlayın.
3. Derleyip karta yükleyin.
4. Seri port üzerinden veya debugger ile açısal verileri gözlemleyebilirsiniz.

### Temel Fonksiyonlar
- `MPU6050_Init(&hi2c1);`  // MPU6050 başlatma
- `MPU6050_Calibration_offset(&hi2c1, &MPU6500);`  // Ofset kalibrasyonu
- `MPU6050_Read_WithKalman(&hi2c1, &MPU6500, time_start);`  // Kalman filtreli okuma

## Lisans
MIT Lisansı. Ayrıntılar için LICENSE dosyasına bakınız.

## Video
[Demo ve Açıklama Videosu](https://youtu.be/EMkmC6QPMf4)
