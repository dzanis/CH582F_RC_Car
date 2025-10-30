
## 🧩 **Настройка HTTPS для Live Server с поддержкой Web Bluetooth (Windows)**

Этот способ нужен, чтобы Web Bluetooth API (`navigator.bluetooth`) **работал в Chrome на Android**,
так как он требует **доверенного HTTPS-соединения** (а не `http://`).

---

### 🧰 Что нужно заранее

1. **VS Code** (установлен Live Server)
2. **OpenSSL для Windows**
   Скачай и установи отсюда:
   👉 [https://slproweb.com/products/Win32OpenSSL.html](https://slproweb.com/products/Win32OpenSSL.html)
   Пример пути после установки:

   ```
   C:\Program Files\OpenSSL-Win64\bin\openssl.exe
   ```

---

### 🪄 1. Создаём батник для генерации сертификата

Создай файл `generate_cert.bat` в любой папке (например, на рабочем столе)
и вставь в него вот этот код:

```bat
@echo off
setlocal

echo Generating HTTPS certificate for Live Server...

set OPENSSL_PATH="C:\Program Files\OpenSSL-Win64\bin\openssl.exe"
set CERT_DIR=%USERPROFILE%\.localhost-ssl

if not exist %CERT_DIR% mkdir %CERT_DIR%
cd %CERT_DIR%

%OPENSSL_PATH% req -x509 -out localhost.crt -keyout localhost.key -newkey rsa:2048 -subj "/C=LV/ST=Riga/L=Riga/O=MyDevOrg/CN=cert" -addext "subjectAltName=DNS:localhost,IP:127.0.0.1,IP:192.168.0.181" -days 365

if exist localhost.crt (
    echo.
    echo ✅ Certificate successfully created in:
    echo %CERT_DIR%
) else (
    echo.
    echo ❌ ERROR: Certificate not created. Check OpenSSL path.
)

pause
```

> ⚠️ Если твой IP другой, замени `192.168.0.181` на свой локальный адрес.

---

### 🧩 2. Запускаем батник

1. Кликни правой кнопкой → **Запуск от имени администратора**
2. При появлении запроса:

   ```
   Enter PEM pass phrase:
   Verifying - Enter PEM pass phrase:
   ```

   — введи пароль `2255`
3. После завершения в консоли появится путь:

   ```
   C:\Users\<твоё_имя>\.localhost-ssl
   ```

---

### 🔐 3. Устанавливаем сертификат в Windows

1. Найди созданный файл `localhost.crt`
2. Дважды кликни → **Установить сертификат**
3. Выбери:

   * **Локальный компьютер** → Далее
   * **Поместить все сертификаты в следующее хранилище**
   * Выбери: **Доверенные корневые центры сертификации**
   * Далее → Готово
4. Подтверди предупреждение (да, доверяем)

---

### ⚙️ 4. Настраиваем Live Server на HTTPS

1. В VS Code открой `settings.json` (или через `Ctrl+,` → значок шестерёнки → *Open Settings (JSON)*)
2. Добавь в конец:

```json
{
    "liveServer.settings.https": {
        "enable": true,
        "cert": "C:\\Users\\<твоё_имя>\\.localhost-ssl\\localhost.crt",
        "key": "C:\\Users\\<твоё_имя>\\.localhost-ssl\\localhost.key",
        "passphrase": "2255"
    },
    "liveServer.settings.port": 5500
 }
```

3. Сохрани и перезапусти Live Server (`Alt+L Alt+O`)

---

### 🖥️ 5. Проверяем на ПК

Открой в браузере:

* `https://localhost:5500`
* или `https://127.0.0.1:5500`

✅ Если всё верно — появится зелёный замок (сертификат доверен).
Web Bluetooth **доступен** (`navigator.bluetooth` не undefined).

---

### 📱 6. Проверяем на телефоне (Android)

1. Телефон и ПК должны быть **в одной Wi-Fi сети**
2. Узнай IP компьютера (например `192.168.0.181`)
3. В Chrome на телефоне открой:

   ```
   https://192.168.0.181:5500
   ```
4. Появится предупреждение о безопасности (“Не защищено”)
   — нажми **“Продолжить” / “Details → Proceed anyway”**
5. После этого:

   * Страница загрузится
   * Web Bluetooth заработает 🎉

> 🔴 Значок красного замка останется — это **нормально**: сертификат самоподписанный,
> но BLE теперь разрешён и работает корректно.

---

### 🧪 7. Проверяем BLE

Открой DevTools → Console
и убедись, что:

```js
console.log(navigator.bluetooth)
```

возвращает объект, а не `undefined`.

Если да — всё настроено ✅
Теперь можно разрабатывать и тестировать BLE-передачу прямо с телефона!

---

### 💡 Резюме

| Этап                            | Цель                             | Статус        |
| ------------------------------- | -------------------------------- | ------------- |
| ✅ Установлен OpenSSL            | Для генерации сертификата        | ✔             |
| ✅ Создан cert/key               | В `%USERPROFILE%\.localhost-ssl` | ✔             |
| ✅ Добавлен в “Trusted Root”     | Чтобы Chrome на ПК доверял       | ✔             |
| ✅ Live Server по HTTPS          | Работает на `https://localhost`  | ✔             |
| ⚠️ Chrome Android предупреждает | Самоподписанный сертификат       | ✔ (нормально) |
| ✅ Web Bluetooth активен         | navigator.bluetooth доступен     | ✔             |

---

На основе https://dev.to/andrewelans/live-server-https-set-it-up-on-macwindows-29cb