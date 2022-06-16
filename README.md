# EasyKeyDublicatorRFID_OLED
Simple RFID (EM-Marie, Mifare Classic) and wired (Dallas, Cyfral, Metacom) key dublicator with arduino. Supports EM4305, T5577, Mifare Classic 1K, RW-1990.1 RW-1990.2 TM-01 TM2004 protocols. There are connections scheme in the project.

Изменения по сравнению с оригиналом:
- Добавлена поддержка модуля MFRC522, что позволяет копировать ключи Mifare Classic 1K 13.56Mhz;
- Добавлена поддержка кнопок. Теперь можно выбрать, что использовать: энкодер или кнопки. В версии с кнопками при долгом нажатии на кнопку BtnDown очищается EEPROM, на BtnOK ключ добавляется в EEPROM;
- Используется безбуферная библиотека работы с OLED для высвобождения ОЗУ под буфер для Mifare Classic 1K.
