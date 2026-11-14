# Исходные зависимости

Нативные проверки используют фиксированные версии:

| Компонент | Commit |
|---|---|
| [Axis](https://github.com/rs-a-demyanenko/Axis/tree/55e855dbc42f560b08a481450cb2e694871f0b11) | `55e855dbc42f560b08a481450cb2e694871f0b11` |
| [GyverStepper](https://github.com/GyverLibs/GyverStepper/tree/e9d0a68833605e87a082a9367877434d6bb8f271) | `e9d0a68833605e87a082a9367877434d6bb8f271` |
| [GyverMotor2](https://github.com/GyverLibs/GyverMotor2/tree/29972794183db1d92df00ef0e71a38dcac48545d) | `29972794183db1d92df00ef0e71a38dcac48545d` |

GyverPlanner2 послужил источником архитектуры очереди и подхода к скоростям в стыках. Перенесённая логика переработана для физических координат и внешних опорных заданий Axis. Целочисленная генерация шагов GyverPlanner2 не используется, заголовок `GyverPlanner2.h` в рабочем проекте не подключается.

Исполняемые зависимости: Axis; GyverMotor2 и GyverStepper через Axis. Новых сторонних библиотек не добавлено. Исходники зависимостей скачиваются тестовым скриптом в `.build/deps/`, в установочный ZIP они не входят.
