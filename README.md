# foc-based-servo
**Рассмотрение различных способов управления [BLDC](https://www.robotdigg.com/product/1000/5008-KV335-or-5010-KV340-brushless-motor) с помощью библиотеки [SimpleFOC](https://docs.simplefoc.com/) и драйвера [SimpleFOC PowerShield](https://github.com/simplefoc/Arduino-SimpleFOC-PowerShield) на Arduino UNO**
## Содержание

- [Содержание](#содержание)
- [Управление по скорости с разомкнутым контуром](#управление-по-скорости-с-разомкнутым-контуром)
- [Управление по скорости с обратной связью](#управление-по-скорости-с-обратной-связью)
- [Управление по положению с обратной связью](#управление-по-положению-с-обратной-связью)
- [Inline Current Sense с посмощью драйвера SimpleFOC PowerShield](#inline-current-sense-с-помощью-драйвера-simplefoc-powershield)
- [Управление моментом в режиме dc_current](#управление-моментом-в-режиме-dc-current)
- [Управление моментом в режиме foc_current](#управление-моментом-в-режиме-foc-current)

## Управление по скорости с разомкнутым контуром
В библиотекет SimpleFOC реализовано управление BLDC двигателем с помощью Векторного управления. Для реализации этих алгоритмов необходимо использавать датчик положения, мной был использован датчик [AS5047P](https://www.digikey.com/en/htmldatasheets/production/1819265/0/0/1/as5047p-ts-ek-ab-manual) с обменой данных через SPI интерфейс.
### Соединение с Arduino UNO
| AS5047P Pin | Arduino Uno Pin | Comment |
|:-----------:|:-----------:|:--------|
| GND | GND | - |
| VDD | 5V | - |
| VDD3V| NC | - |
| MOSI | MOSI (Pin: 11) | - |
| MISO | MISO (Pin: 12) | - |
| CLK | SCK (Pin: 13) | - |
| CSn | 9 | Can be any unused digital pin on the arduino as long as it's configured here `AS5047P as5047p(<ChipSelectPort>);` |

Структурная схема системы и теория управления приведены [здесь](https://docs.simplefoc.com/velocity_openloop). Фазное сопротивление обмотки двигателя неизвестно, поэтому ограничение устанавливалось по потребляемому напряжению, а не току, что влияет на точность управления.

## Управление по скорости с обратной связью

Для управления скоростью с обратной связью я использовал ПИД-регулятор, настройка которого была произведена методом Циглера-Никольса. СТруктурная схема системы управления приведена [здесь](https://docs.simplefoc.com/velocity_loop). Контур управления моментом имеет структуру, которая описана [здесь](https://docs.simplefoc.com/torque_control). Контур управления моментом работает в режиме Voltage mode. В результате дальнейше работы был реализован алгоритм с обратной связью по скорости с управлением моентом режимах [dc_current](https://docs.simplefoc.com/dc_current_torque_mode) и [foc_current](https://docs.simplefoc.com/foc_current_torque_mode). При данном методе управления BLDC двигатедь с внешней нагрузкой в виде редуктора и ременной передачм стабильно работает в диапазоне скоростей от 5 до 30 рад/с, при большем значении скорости вращения происходит сильный нагрев платы управления.

## Управление по положению с обратной связью

Структурная схема системы управления двигателем приведена [здесь](https://docs.simplefoc.com/angle_loop). Для оптимадьного управления помимо ПИД-регулятора в контуре тока используется ПИД-регулятор в контуре положения, настройка которого также производилась с помощью эмпирического метода Циглера-Никольса. В результате использования магнитного датчика положения удается добиться крайне высокой точности позиционирования. Для точного управления моменом привода далее будут расмотрены способы управления моментом привода с обратной связью по току вместо связи по напряжению.

## Inline Current Sense с посмощью драйвера SimpleFOC PowerShield

Для осуществления точного управления моментом двигателя с учетом динамики привода необходимо добавить обратную связь по току. В библиотеке SimpleFOC реализовано несколько различных способов отслеживать протекающий по фазам ток в зависимости от аппаратной части принятого решения, что в свою очередь напрямую влияет на алгоритм управления моментом. В рамках проекта был выбран метод отслеживания значения тока с помощью Inline Current Sense, описание метода приведено [здесь](https://docs.simplefoc.com/inline_current_sense). В качестве входных параметров для класса необходимо указать сопротивление шунтирующего резистора, коэффициент операционного усилителя и пины, которые связаны с выводами фаз электродвигателя. Все необходимые параметры указаны в скрипте.

## Управление моментом в режиме dc_current

В рамках данного режима происходить управление током Iq. который получается путем преоьразования Парк из управляющего сигнала, при этом составляющая Id не контролируется. Поэтому данный режим управления не дает полного понимания о генерируемом приводом моменте. Описание данного метода управления приведено [здесь](https://docs.simplefoc.com/dc_current_torque_mode). Для управления использовался ПИ-регулятор с LPF-фильтром, параметры которых приведены в скрипте. После успешной реализации данный метод управления был внедрен в алгоритм управления по скорости и по положению, что значительно увеличило диапазон скоростей, при которых система работает стабильно. Также значительно снизился протекаемый через плату управления ток, что позволило системе увеличить время стабильной работы без перегрева.

## Управление моментом в режиме foc_current

данный метод управления отличается от предыдущего тем, что составляющая Id тока является управляемой и это значение стремится к нулю, поэтому данный меод управления моментом является точным способом управления моментом BLDC двигателя. Описание данного метода управления приведено [здесь](https://docs.simplefoc.com/foc_current_torque_mode). Для реализации этого алгоритма управления к предыдущей системе управления необходимо добавить контур управления током Id с помощью ПИ-регулятора и LPF-фильтра по аналогии с контуром Iq. Пока данный метод управления находится в процессе реализации, после чео он будет применен в алгоритмах управления скоростью и положением для увеличения эффективности работы привода.

