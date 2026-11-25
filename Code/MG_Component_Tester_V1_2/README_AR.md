# MG_V1.0 — Optimized Component Tester

## مشكلة حجم الكود

نتيجة Arduino IDE السابقة: **31,044 bytes / 30,720 bytes**، وبذلك يوجد تجاوز **324 bytes**. تم إعداد نسخة محسّنة لتقليل حجم البرنامج من غير حذف خوارزميات القياس أو أسماء أعضاء الفريق أو مميزات شاشة OLED وRGB.

**مهم:** البيئة الحالية لا تحتوي على Arduino AVR Compiler، ولذلك هذه النسخة لم يتم تأكيد حجمها النهائي أو نجاحها على العتاد. **شغّل Verify** في Arduino IDE للحصول على الاستهلاك الفعلي. ناتج التعديل متوقع أن يكون أصغر بصورة ملحوظة بسبب التخلص من مسار `HardwareSerial` غير المستخدم.

## التعديلات الرئيسية

1. حصر تشغيل `Serial` داخل `#ifdef WITH_UART`؛ المسار الافتراضي يعتمد على OLED فقط، ولا يحتاج HardwareSerial أو تأخير إرسال كل حرف.
2. تصحيح `TXD_MSK` عند عدم استعمال UART إلى `0x00`، حتى لا تُضبط أطراف A3/A4/A5 كـ outputs بصورة غير مقصودة بسبب أوامر `DDRC`. حُفظت أطراف القياس A0/A1/A2.
3. تغيير تعريف OLED_RESET إلى `-1` لأن شاشة I2C ذات الأربع أطراف لا تحتوي على Reset موصل.
4. إضافة معالجة لحالة فشل تهيئة OLED بعرض LED أحمر ثابت وإيقاف المعالجة.
5. الحفاظ على واجهة `MG_V1.0`، وعرض 11 اسمًا، وحجز أول 3 صفوف Pixel من شاشة 128×64.
6. إضافة توثيق إنجليزي لملف البرنامج: معلومات المشروع والأجزاء والوظائف الأساسية. بقيت تعليقات القياس الأصلية المفيدة باللغة الإنجليزية، مع الاحتفاظ بنسبة محرك القياس لأصحابه.

## التوصيل الموافق للرسم

| الوظيفة | Arduino Nano |
| --- | --- |
| TP1/TP2/TP3 | A0/A1/A2 |
| Test button | A3 (مقاومة 10kΩ إلى +5V، الزر إلى GND) |
| OLED SDA/SCL | A4/A5 |
| TP1 680Ω / 470kΩ | D8 / D9 |
| TP2 680Ω / 470kΩ | D10 / D11 |
| TP3 680Ω / 470kΩ | D12 / D13 |
| RGB red/green/blue | D3/D4/D5 مع مقاومة منفصلة لكل لون |

**RGB:** افتراضيًا `MG_RGB_COMMON_ANODE 0` للـ Common Cathode. غيّرها إلى 1 لو الليد Common Anode. دلالة الأخضر: المكون تم التعرف عليه وقياسه؛ لا تعتبره شهادة مطابقة للمواصفات. الأحمر: غير معروف أو يوجد احتمال مشكلة، أو دخل جهد على أطراف الاختبار. الأزرق: القياس جارٍ.

**Display offset:** `MG_OLED_TOP_OFFSET 3` تعني 3 صفوف Pixel، ويمكن زيادتها إذا زاد عرض الجزء التالف.

## طريقة Verify وUpload

1. فك ملف ZIP ثم افتح `MG_Component_Tester_Optimized/MG_Component_Tester_Optimized.ino`.
2. Board: **Arduino Nano**؛ Processor: **ATmega328P**؛ وإذا فشل الرفع على بعض النسخ القديمة جرّب **ATmega328P (Old Bootloader)**.
3. Libraries: `Adafruit GFX` و`Adafruit SSD1306` و`Adafruit BusIO`. مكتبتا Wire وSPI تأتيان مع AVR Core.
4. اضغط **Verify** وافحص `Sketch uses ... bytes`. تأكد أنه لا يتجاوز **30,720 bytes**.
5. تأكد من توفير تغذية 5V مناسبة، ومن أن الاختبار يجري على **مكونات غير موصلة بمصدر طاقة**، مع فصل وتفريغ المكثفات بطريقة آمنة قبل إدخالها في التيستر.
6. اختبر مقاومة مرجعية معروفة، دايود، مكثف غير مشحون، وترانزستور؛ قارن النتائج بأداة قياس موثوقة قبل اعتماد الـPCB.

## رسالة invalid library

`loading library from C:\Users\engmo\AppData\Local\Arduino15\libraries\package-856373798: invalid library: no header files found` تحذير يخص مجلدًا غير صالح، **وليس سبب تجاوز حجم البرنامج**. يمكن بعد إغلاق Arduino IDE إعادة تسمية ذلك المجلد، إذا لم يكن يحوي ملفات مهمة، مثل `package-856373798.disabled`، ثم إعادة تشغيل Arduino IDE. لا تغيّر مكتبات Adafruit السليمة الظاهرة في اللوج.

## رسالة الخطأ إن استمرت

إذا استمر `text section exceeds available space in board`، انسخ آخر 15 سطرًا من الـ Verify الجديد بما فيها `Sketch uses ...` وحدد إعداد Board/Processor؛ يُمكن وقتها تحديد تحسين إضافي وفق الحجم المتبقي دون التضحية بالقياس.

## نسب المساهمة

- Mohamed Gamal: project firmware integration, OLED user interface, RGB status, customization, and engineering updates.
- Core transistor/component measurement engine: derived from Transistor Tester for Arduino v1.08a based on Karl-Heinz Kubbeler v1.08k; original authorship is acknowledged.
