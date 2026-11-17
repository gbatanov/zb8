# Проект координатора на ESP32H2

Для сети на 60–100 устройств оптимально не усложнять систему и оставить сетевой стек на ESP32-H2, а всю бизнес-логику перенести на Linux ПК:
• На ESP32-H2: Соберите прошивку на базе ESP-Zigbee-SDK v2.x (взяв за основу all_device_types_app в роли координатора с увеличенными таблицами network_size и NVS). Напишите на C простую прослойку, которая принимает ZCL-отчеты от датчиков, упаковывает их в бинарные фреймы (или JSON) и отправляет в USB Serial/JTAG, а в обратную сторону принимает команды на управление реле и разрешением сопряжения (permit_join).
• На Linux (на Go или C): Напишите демон, который читает /dev/ttyACM0, хранит в базе данных (например, SQLite или PostgreSQL) человекочитаемые имена устройств, их состояния, привязки (Bindings) и правила автоматизации.
Такой подход избавит вас от необходимости разбирать сложный проприетарный протокол ZNSP из старой ветки SDK, даст доступ к актуальному стеку Zigbee от Espressif и обеспечит легкое масштабирование вашей сети до 100–120 устройств в будущем.



## Правила добавления кластеров для координатора

• Роль Client (Клиент). Для всех кластеров, от которых координатор принимает данные или которыми управляет (температура, влажность, реле, диммеры, энергомониторинг, кнопки), указывается роль EZB_ZCL_CLUSTER_CLIENT.
• Роль Server (Сервер). Сам координатор обязан иметь базовые серверные кластеры (Basic и Identify с ролью EZB_ZCL_CLUSTER_SERVER), чтобы корректно отвечать на запросы идентификации от других устройств в сети.


## Важные нюансы при работе с датчиками в этой схеме

1. Проприетарные кластеры Tuya и Xiaomi.
Многие устройства Tuya используют нестандартный кластер 0xEF00, а старые датчики Xiaomi/Aqara шлют данные в кластер 0x0000 (атрибут 0xFF01) или 0xFCC0. Для Tuya вы можете добавить в массив client_clusters значение 0xEF00, а для Xiaomi — 0xFCC0: функция ezb_zcl_create_cluster_desc() позволяет регистрировать любые произвольные 16-битные ID кластеров.
2. Обработка сырых команд и специфичных отчетов.
В zb_action_handler помимо EZB_ZCL_CORE_REPORT_ATTR_CB_ID (стандартные отчеты атрибутов) обязательно подпишитесь на колбэк EZB_ZCL_CORE_CMD_DEFAULT_RESP_CB_ID и сырые кластерные команды (например, для IAS Zone — EZB_ZCL_CORE_IAS_ZONE_STATUS_CHANGE_NOTIFICATION_CB_ID, так как датчики открытия и протечки шлют не отчет атрибута, а команду смены статуса зоны).
3. Автоматический биндинг при подключении.
Некоторые батарейные датчики (например, IKEA или Sonoff) после входа в сеть начинают слать отчеты только после того, как координатор отправит им ZDO-запрос на привязку (ezb_zdo_bind_req) к своему COORDINATOR_ENDPOINT (1) по нужному кластеру. Эту логику удобно запускать прямо из вашей программы на Go при получении события о новом устройстве (Device Announce).

## partitions.csv
```
# Name,      Type, SubType, Offset,  Size, Flags
nvs,         data, nvs,     0x9000,  0x6000,
phy_init,    data, phy,     0xf000,  0x1000,
factory,     app,  factory, 0x10000, 1M,
zb_storage,  data, nvs,     0x110000, 0x10000,
```


## Минимальный каркас обмена с Linux по USB (COBS + Бинарные фреймы)

Поскольку встроенный USB Serial/JTAG на ESP32-H2 работает как виртуальный COM-порт (/dev/ttyACM0 в Linux), проще всего обмениваться компактными бинарными пакетами, разделенными нулевым байтом 0x00 (кодирование COBS), или текстовыми строками JSON (NDJSON) для быстрого прототипирования.
Важно: в idf.py menuconfig отключите вывод системных логов ESP-IDF в USB (Component config -> ESP System Settings -> Channel for console output -> None или выведите логи на физический UART0), чтобы отладочные сообщения не перемешивались с вашим протоколом.

### Сторона ESP32-H2 (C, ESP-IDF)

Настраиваем драйвер usb_serial_jtag и пересылаем входящие ZCL-отчеты (EZB_ZCL_CORE_REPORT_ATTR_CB_ID) в USB:  
(to_comp.c)


### Сторона Go
```
package main

import (
	"encoding/binary"
	"fmt"
	"io"
	"log"

	"go.bug.st/serial"
)

type HostFrame struct {
	SOF       uint8
	Cmd       uint8
	ShortAddr uint16
	Endpoint  uint8
	ClusterID uint16
	AttrID    uint16
	DataType  uint8
	Len       uint8
	Payload   [32]byte
}

func main() {
	mode := &serial.Mode{BaudRate: 115200}
	port, err := serial.Open("/dev/ttyACM0", mode)
	if err != nil {
		log.Fatalf("Ошибка открытия порта: %v", err)
	}
	defer port.Close()

	// Отправляем команду Permit Join на 60 секунд
	_, err = port.Write([]byte{0xAA, 0x10, 60})
	if err != nil {
		log.Printf("Ошибка записи: %v", err)
	}

	// Читаем входящие события от ESP32-H2
	for {
		var frame HostFrame
		err := binary.Read(port, binary.LittleEndian, &frame)
		if err != nil {
			if err == io.EOF {
				continue
			}
			log.Fatalf("Ошибка чтения фрейма: %v", err)
		}

		if frame.SOF != 0xAA {
			// В боевом коде здесь нужен поиск байта 0xAA или декодер COBS
			continue
		}

		fmt.Printf("Устройство: 0x%04X (EP %d) | Кластер: 0x%04X | Атрибут: 0x%04X | Данные: %x\n",
			frame.ShortAddr, frame.Endpoint, frame.ClusterID, frame.AttrID, frame.Payload[:frame.Len])
	}
}
```


На отладочных платах с ESP32-H2 (например, ESP32-H2-DevKitM-1) эти два разъема подключены к разным аппаратным интерфейсам чипа и имеют четкое разделение:

## Назначение двух USB-портов на плате


Разъем на плате	Куда подключен аппаратно	Как определяется в Linux	Назначение
USB
	Напрямую к встроенному в ESP32-H2 контроллеру USB Serial/JTAG (пины GPIO26 и GPIO27).	/dev/ttyACM0	Встроенный аппаратный JTAG (для отладки GDB) + виртуальный COM-порт (CDC) для данных или прошивки.
UART
	К внешнему мосту USB-UART на плате (например, CP2102 или CH343), который идет на пины UART0 (TX/RX).	/dev/ttyUSB0	Классический последовательный порт для вывода системных логов, загрузчика и прошивки.


## Как разнести логи и поток данных по двум портам

Для вашей задачи идеальная конфигурация — оставить системные логи ESP-IDF на порту UART (/dev/ttyUSB0), а высокоскоростной бинарный протокол координатора пустить через порт USB (/dev/ttyACM0). Тогда отладочные сообщения стека Zigbee никогда не попадут в ваш парсер на Go/C, а вы сможете в реальном времени смотреть логи в соседнем терминале.

### Настройка в idf.py menuconfig

Откройте конфигуратор проекта (idf.py menuconfig) и задайте следующие параметры:
1. Направляем системные логи в порт UART (/dev/ttyUSB0):
	• Перейдите в Component config -> ESP System Settings -> Channel for console output.
	• Выберите Default: UART0 (если стоял USB Serial/JTAG, поменяйте на UART0).
2. Отключаем вторичную консоль (чтобы логи не дублировались в USB):
	• В том же разделе найдите пункт Channel for console secondary output.
	• Выберите No secondary console.

### 2. Инициализация порта USB для данных в коде (C)

Поскольку системная консоль теперь привязана к UART0, драйвер USB Serial/JTAG (/dev/ttyACM0) полностью свободен для вашей программы. Вы инициализируете его в коде и используете исключительно для обмена фреймами с Linux:
```
#include "driver/usb_serial_jtag.h"
#include "esp_log.h"

static const char *TAG = "ZB_GW";

void init_interfaces(void) {
    // 1. Этот лог уйдет строго в порт UART (/dev/ttyUSB0)
    ESP_LOGI(TAG, "Инициализация транспорта USB Serial/JTAG...");

    // 2. Настраиваем порт USB (/dev/ttyACM0) под бинарный протокол
    usb_serial_jtag_driver_config_t usb_cfg = {
        .rx_buffer_size = 2048,
        .tx_buffer_size = 2048,
    };
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usb_cfg));

    ESP_LOGI(TAG, "USB порт готов к обмену данными с хостом");
}
```

### Важная особенность работы usb_serial_jtag при отключенном кабеле

У встроенного контроллера USB Serial/JTAG на ESP32-H2 есть нюанс: если кабель из разъема USB выдернут или программа на Linux закрыла порт /dev/ttyACM0, аппаратный буфер передачи (TX FIFO) перестает опустошаться.
• Чтобы задача Zigbee не зависла при вызове usb_serial_jtag_write_bytes(), всегда передавайте небольшой таймаут (например, pdMS_TO_TICKS(10) или pdMS_TO_TICKS(20)), а не portMAX_DELAY. Если хост не подключен, функция просто вернет 0 записанных байт через 10 мс, и координатор продолжит штатно работать.

### Как этим пользоваться на Linux

1. Подключите оба кабеля к ПК (или в продакшене оставляйте только USB).
2. Для просмотра системных логов и паник ESP-IDF запустите монитор на порту UART:
```
idf.py -p /dev/tty.usbmodem58970173491 monitor
```
3. Ваш координатор на Go или C открывает порт /dev/ttyACM0 (настройка скорости BaudRate для /dev/ttyACM0 формальна, так как USB CDC работает на полной скорости шины USB 2.0 Full-Speed — до 12 Мбит/с).

## Прописать путь к настройкам в терминале

```
. $HOME/work/esp-idf/v6.0/esp-idf/export.sh
```

## Остановить вывод в монитор
```
CTRL+]
```