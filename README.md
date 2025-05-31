# MiROS - Mini Real-Time Operating System for STM32F103

## Giới thiệu

**MiROS** là một hệ điều hành thời gian thực (RTOS) tối giản, được phát triển cho vi điều khiển STM32F103. Dự án này nhằm mục đích minh họa các khái niệm cơ bản của RTOS như quản lý luồng (thread), semaphore, mutex, queue, event group, software timer và stream buffer. MiROS phù hợp cho mục đích học tập, nghiên cứu hoặc làm nền tảng cho các dự án nhúng nhỏ.

## Tính năng

- **Thread/Task Management:** Hỗ trợ đa luồng với ưu tiên, chuyển ngữ cảnh bằng PendSV.
- **Semaphore:** Hỗ trợ binary và counting semaphore, dùng cho đồng bộ hóa giữa các task hoặc giữa ISR và task.
- **Mutex:** Hỗ trợ mutex với ưu tiên ceiling, tránh priority inversion.
- **Queue:** Hàng đợi FIFO cho truyền dữ liệu giữa các task.
- **Event Group:** Cho phép task chờ nhiều sự kiện (bit) cùng lúc.
- **Software Timer:** Bộ đếm thời gian mềm với callback.
- **Stream Buffer:** Truyền dữ liệu dạng luồng giữa các task.
- **Tick-based Scheduling:** Sử dụng SysTick để quản lý timeout và delay.

## Cấu trúc dự án

```
.
├── Core/
│   ├── Inc/
│   │   └── miros.h         # Định nghĩa API và các struct của RTOS
│   └── Src/
│       ├── miros.c         # Toàn bộ mã nguồn RTOS
│       └── main.c          # Ứng dụng mẫu sử dụng RTOS
├── mrtos/
│   ├── miros.c            
│   └── miros.h
├── Drivers/
│   └── STM32F1xx_HAL_Driver/
├── ...
```

## Sử dụng

### 1. Khởi tạo RTOS

Trong `main.c`, gọi hàm khởi tạo RTOS và các thread:
```c
OS_init(stack_idleThread, sizeof(stack_idleThread));
OSThread_start(&blinky1, 2U, &main_blinky1, stack_blinky1, sizeof(stack_blinky1));
OSThread_start(&blinky2, 5U, &main_blinky2, stack_blinky2, sizeof(stack_blinky2));
OS_run();
```

### 2. Sử dụng các primitive

- **Semaphore:**  
  ```c
  Semaphore_t sema;
  OS_Binary_Semaphore_Init(&sema);
  OS_Semaphore_Get(&sema, timeout);
  OS_Semaphore_Release(&sema);
  ```

- **Mutex:**  
  ```c
  Mutex_t mutex;
  OS_Mutex_Init(&mutex);
  OS_Mutex_Get(&mutex, priority, timeout);
  OS_Mutex_Release(&mutex);
  ```

- **Queue:**  
  ```c
  OS_Queue_t queue;
  uint32_t buffer[10];
  OS_Queue_Init(&queue, buffer, 10);
  OS_Queue_Push(&queue, value, timeout);
  OS_Queue_Pop(&queue, &value, timeout);
  ```

- **Event Group:**  
  ```c
  Event_Group_t event_group;
  OS_EventGroup_Init(&event_group);
  OS_EventGroup_SetBits(&event_group, BIT_1);
  OS_EventGroup_WaitBits(&event_group, BIT_1, true, true, timeout);
  ```

- **Software Timer:**  
  ```c
  soft_timer_t timer;
  OS_SoftwareTimer_Init(&timer, 1000, callback);
  OS_SoftwareTimer_Start(&timer);
  ```

- **Stream Buffer:**  
  ```c
  Stream_Buffer_t stream;
  OS_Stream_Buffer_Init(&stream, 32, 8);
  OS_Stream_Buffer_Send(&stream, data, len, timeout);
  OS_Stream_Buffer_Receive(&stream, data, len, true, timeout);
  ```

### 3. Tích hợp với STM32CubeIDE

- Thêm `miros.c` và `miros.h` vào project.
- Bao gồm `"miros.h"` trong các file cần sử dụng RTOS.
- Đảm bảo hàm `SysTick_Handler` trong `stm32f1xx_it.c` gọi `OS_tick()` và `OS_SoftwareTimer_Tick()`.

## Lưu ý

- MiROS chỉ phù hợp cho mục đích học tập, không khuyến nghị dùng cho sản phẩm thương mại.
- Không xử lý đầy đủ các trường hợp lỗi hoặc bảo vệ tài nguyên phức tạp.
- Ưu tiên thread càng cao thì số càng lớn.

## Tài liệu tham khảo

**Tác giả:**  
- Dựa trên mã nguồn và ý tưởng của Miro Samek,sau đó được bổ sung và phát triển.
