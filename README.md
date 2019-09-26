# Feed Pump Controller
Feed pump controller for water treatment used Atmel ATtiny4313.
<br><br>
Контроллер дозирующего насоса для водоподготовки (обезжелезивания).<br><br>
![Controller](https://user-images.githubusercontent.com/6220128/65607139-b4af3780-dfb4-11e9-8c67-2f28ee1d1146.jpg)
<br>
На экране:<br>
Строки:<br>
1: текущее потребление в м3/ч -> потребленное после последней регенерации в м3;<br>
   "*" - будет регенерация, "->" - идет регенерация, "<-" - идет обратная промывка<br>
2: Regs - Количество регенераций; Days - дней после регенерации<br>
3. Day - потребленное за день; Yd - потребленное вчера<br>
4. День недели; Текущее время; Тревога: "!" - села батарейка, <br>
     "?" - низкий проток; "#" - малый расход на регенерацию;<br>
   Израсходовано в литрах:<br>
   "A:" - среднее за день (если за день 0, то не учитывается), <br>
   "R:" - за последнюю регенерацию,<br>
   "T:" - итого в кубометрах, <br>
   "D:" - за последний пролив воды, если за день было < 10л.<br>
<br>
Настройка, нажать кнопку SET (выход через 30 сек, кроме п.1,3)<br>
1. Вкл./Выкл. (NEXT - выбор выхода, SET - смена состояния)<br>
   P - Насос, R - OUT2 (во время регенерации), D - OUT3 (пролив)
2. Установка времени (W - номер дня недели 1..7)
3. Счетчик импульсов датчика за 4 минуты, NEXT - выход
4. Подстройка частоты (на выводе 14 (LCD 11) - 62500 Hz) (SET: -, NEXT: +)
5. Сброс счетчиков (нажать на SET более 15 секунд)
6. Установка числа импульсов на литр
7. Максимальный поток = (макс. в m3/ч) / 3.6 * (число импульсов на литр) / 10<br>
   насос включается пропорционально текущему расходу, но не выше макс.<br>
   Чем меньше значение - тем больше дозируется
8. Ресурс в литрах для запуска регенерации в определенный час
9. Минимальное время включения дозирующего насоса в мс (0.001 с)
10. Час регенерации
11. Минимальное кол-во литров во время пролива воды (меньше - авария)
12. Минимальное кол-во литров на регенерацию (меньше - авария)
13. Время пролива воды в сек, не больше 58!
14. Максимальный расход воды на пролив в литрах
<br><br>
Схема водоподготовки:<br>
![image](https://user-images.githubusercontent.com/6220128/65678007-41143580-e05b-11e9-9f80-7e81d38299be.png)
