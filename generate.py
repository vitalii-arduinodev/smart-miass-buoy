import os
import re
from openai import OpenAI

# Актуальные имена моделей в API DeepSeek:
# "deepseek-v4-pro"   - для сложных задач и генерации кода
# "deepseek-v4-flash" - для быстрых и дешевых ответов
MODEL_NAME = "deepseek-v4-flash"

# Ключ берём ТОЛЬКО из переменной окружения; хранить его в коде нельзя
# (прежде утекший ключ sk-c52e... нужно отозвать в кабинете DeepSeek)
api_key = os.getenv("DEEPSEEK_API_KEY")
if not api_key:
    raise SystemExit("[ОШИБКА] Переменная окружения DEEPSEEK_API_KEY не задана.")

client = OpenAI(
    api_key=api_key,
    base_url="https://api.deepseek.com"
)

system_instruction = (
    "Ты — Senior Embedded Engineer. Пиши готовый C++20 код для ESP32 (LilyGO A7670) под PlatformIO. "
    "Если требуются платформенные файлы, разделяй их строго заголовками формата '### имя_файла'. "
    "Запрещено использовать класс String в глобальной памяти. "
    "Выдавай ТОЛЬКО рабочий C++ код без пояснений."
)

prompt_text = open("prompt.txt", "r", encoding="utf-8").read()

print(f"Отправка запроса в {MODEL_NAME}...")

response = client.chat.completions.create(
    model=MODEL_NAME,
    messages=[
        {"role": "system", "content": system_instruction},
        {"role": "user", "content": prompt_text}
    ],
    # Отключаем thinking (размышления), чтобы токены шли СТРОГО на генерацию C++ кода!
    # Если размышления нужны, поставь "enabled" и увеличь max_tokens до 8192
    extra_body={"thinking": {"type": "disabled"}},
    max_tokens=60000
)

content = response.choices[0].message.content

if not content:
    print("[ОШИБКА] Ответ от модели пуст! Проверь баланс API или лимиты.")
    exit(1)

# Разбор ответа по секциям ### filename
sections = re.split(r'^###\s+(.+)$', content, flags=re.MULTILINE)

if len(sections) > 1:
    for i in range(1, len(sections), 2):
        file_path = sections[i].strip()
        file_code = sections[i+1].strip()
        
        # Очистка от тройных бэктиков ```cpp и ```
        file_code = re.sub(r'^```[a-zA-Z]*\n', '', file_code)
        file_code = re.sub(r'\n```$', '', file_code).strip()
        
        dir_name = os.path.dirname(file_path)
        if dir_name:
            os.makedirs(dir_name, exist_ok=True)
            
        with open(file_path, "w", encoding="utf-8") as f:
            f.write(file_code)
        print(f"[СОХРАНЕНО] -> {file_path}")
else:
    # Если заголовков ### нет, сохраняем в main.cpp
    cleaned = re.sub(r'^```[a-zA-Z]*\n', '', content.strip())
    cleaned = re.sub(r'\n```$', '', cleaned).strip()
    
    os.makedirs("src", exist_ok=True)
    with open("src/main.cpp", "w", encoding="utf-8") as f:
        f.write(cleaned)
    print("[СОХРАНЕНО] -> src/main.cpp")