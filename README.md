# Micromouse STM32F411CEU6

Firmware Hardware + Motion cho BlackPill và DRV8833. Build bằng `pio run -e micromouse` và nạp một lần bằng ST-Link.

## Chạy thử bằng NRST và LED

BlackPill V2.0 dùng LED **PC13**, sáng ở mức thấp. Nút **KEY** dùng PA0, cùng chân với `nSLEEP` của DRV8833 trên robot này. Dùng **NRST**, tránh dùng KEY và BOOT0. NRST reset MCU, vì vậy thao tác chọn và chạy dùng chuỗi nhấn thay cho đo thời gian giữ nút. [Tài liệu BlackPill của WeAct](https://github.com/WeActStudio/WeActStudio.MiniSTM32F4x1)

Trước khi thử trên sàn, kê bánh và đo **nSLEEP ở mức thấp khi giữ NRST**, kể cả khi nguồn motor đang bật. DRV8833 có điện trở kéo xuống bên trong IC. Kiểm tra module thực tế vì mạch ngoài có thể thay đổi mức chân trong lúc reset. [Datasheet DRV8833](https://www.ti.com/product/DRV8833)

1. Đặt xe đứng yên và bật nguồn. LED nháy từng cụm 1–7 lần để báo bài đang chọn. Nhấn NRST trong cụm nháy ngắn để chuyển sang bài tiếp theo.
2. Khi LED chuyển sang **nháy chậm**, nhấn NRST để mở cửa sổ xác nhận. Khi thấy hai nháy dài lặp lại, nhấn NRST thêm lần nữa trong 6 giây. Nếu hết thời gian, LED quay về bộ chọn.
3. Giữ xe đứng yên trong lúc LED chớp ngắn mỗi giây. Firmware sẽ bắt đầu sau khi gyro hiệu chuẩn xong. LED sáng liên tục khi chạy. Riêng bài 7, LED chỉ sáng liên tục sau khi cả hai encoder đã ghi nhận bánh chạy tới.
4. Nhấn **NRST trong lúc chạy** để dừng khẩn cấp. Hai nháy ngắn báo kết thúc thành công. Nháy rất nhanh báo lỗi. Bài 7 chỉ báo thành công khi nhận NRST trong lúc cả hai bánh đang chạy. Nhấn NRST để thoát trạng thái lỗi hoặc kết quả bài 7 sau khi đã kiểm tra xe. Sau các bài 1–6 thành công, bộ chọn hiện lại và chờ xác nhận mới.

| Bài | Thao tác |
| --- | --- |
| 1 | Tiến 60 count quy đổi theo tâm xe, giữ hướng bằng gyro và cân bằng bánh bằng encoder |
| 2 | Quay trái 90° |
| 3 | Quay phải 90° |
| 4 | Quay trái 45° |
| 5 | Quay phải 45° |
| 6 | Tiến 40 count, quay trái 90°, tiến 40 count |
| 7 | Tiến tối đa theo mục tiêu 120 count. Nhấn NRST khi LED sáng liên tục để thử dừng khẩn cấp |

Thông số bài thử nằm trong `kPhysicalTest` ở `include/physical_test.h`. Quy đổi dùng 69 cạnh lên A/vòng bánh trái và 64/vòng bánh phải. Đường kính bánh 34 mm là giá trị kế thừa. Heading PID giữ `2.0 / 0.06 / 0.15`; turn PD giữ `4.0 / 0 / 0.65`. Hai bài thẳng dùng count danh nghĩa theo tâm do hai encoder có độ phân giải khác nhau. Cần đo độ thẳng, góc, quãng dừng và thời gian ngắt motor trên robot thật.

## Motion và cảm biến

Gọi `motionInit()` một lần, gọi `motionUpdate()` thường xuyên trong vòng lặp và đợi `Idle` sau khi gyro hiệu chuẩn. `motionForward()` giữ hướng tới khi dừng hoặc hết giới hạn an toàn 10 giây. `motionMoveCounts()` nhận count có dấu từ 12 đến 220 theo trị tuyệt đối. `motionTurn()` nhận góc tương đối có dấu từ 10° đến 360° theo trị tuyệt đối, với chiều trái dương. Lệnh trả `true` khi được nhận. `motionStop()` giảm PWM rồi phanh về `Idle`. `motionEmergencyStop()` kéo nSLEEP thấp ngay và khóa lệnh đến khi reset.

`motionGetStatus()`, `motionGetEncoderData()`, `motionIsBusy()` và `motionHasError()` cung cấp trạng thái, lỗi và phản hồi. `motionSetHeadingOffset()` cho phép tầng điều khiển tương lai gửi góc bù ngắn hạn vào cùng PID giữ hướng. API ToF gồm `tof::begin()`, `tof::updateInit()`, `tof::poll()` và `tof::shutdown()`. Firmware thử khởi tạo ToF lúc đứng yên. Ba cảm biến phải, trái, trước dùng địa chỉ 0x31, 0x32, 0x33. XSHUT cần pull-up phía cảm biến. Các bài thử chuyển động sử dụng encoder và IMU. [Tài liệu ICM-20602](https://product.tdk.com/system/files/dam/doc/product/sensor/mortion-inertial/imu/data_sheet/ds-000176-icm-20602-v1.1.pdf) · [Tài liệu carrier VL53L0X](https://www.pololu.com/product/2490)

## Kiểm thử native

```powershell
New-Item -ItemType Directory -Force .pio | Out-Null
g++ -std=c++17 -Wall -Wextra -Werror -pedantic -Iinclude src/control.cpp src/motion.cpp test/motion_tests.cpp -o .pio/motion_tests.exe
if ($LASTEXITCODE) { throw "Motion test build failed" }
foreach ($case in (& ./.pio/motion_tests.exe --list)) {
    & ./.pio/motion_tests.exe $case
    if ($LASTEXITCODE) { throw "Motion test failed ($case)" }
}
g++ -std=c++17 -Wall -Wextra -Werror -pedantic -Itest/support -Iinclude src/imu.cpp src/tof.cpp test/sensor_tests.cpp -o .pio/sensor_tests.exe
if ($LASTEXITCODE) { throw "Sensor test build failed" }
& ./.pio/sensor_tests.exe
if ($LASTEXITCODE) { throw "Sensor test failed" }
```
