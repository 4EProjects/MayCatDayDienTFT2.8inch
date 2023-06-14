
void touch_calibrate()
{
    uint16_t calData[5];
    uint8_t calDataOK = 0;

    // check file system exists
    if (!SPIFFS.begin())
    {
        Serial.println("Formating file system");
        SPIFFS.format();
        SPIFFS.begin();
    }

    // check if calibration file exists and size is correct
    if (SPIFFS.exists(CALIBRATION_FILE))
    {
        if (REPEAT_CAL)
        {
            // Delete if we want to re-calibrate
            SPIFFS.remove(CALIBRATION_FILE);
        }
        else
        {
            File f = SPIFFS.open(CALIBRATION_FILE, "r");
            if (f)
            {
                if (f.readBytes((char *)calData, 14) == 14)
                    calDataOK = 1;
                f.close();
            }
        }
    }

    if (calDataOK && !REPEAT_CAL)
    {
        // calibration data valid
        tft.setTouch(calData);
    }
    else
    {
        // data not valid so recalibrate
        tft.fillScreen(TFT_BLACK);
        tft.setCursor(20, 0);
        tft.setTextFont(2);
        tft.setTextSize(1);
        tft.setTextColor(TFT_WHITE, TFT_BLACK);

        tft.println("Touch corners as indicated");

        tft.setTextFont(1);
        tft.println();

        if (REPEAT_CAL)
        {
            tft.setTextColor(TFT_RED, TFT_BLACK);
            tft.println("Set REPEAT_CAL to false to stop this running again!");
        }

        tft.calibrateTouch(calData, TFT_MAGENTA, TFT_BLACK, 15);

        tft.setTextColor(TFT_GREEN, TFT_BLACK);
        tft.println("Calibration complete!");

        // store data
        File f = SPIFFS.open(CALIBRATION_FILE, "w");
        if (f)
        {
            f.write((const unsigned char *)calData, 14);
            f.close();
        }
    }
}

void pipp()
{
    digitalWrite(buzzerPin, HIGH);
    delay(50);
    digitalWrite(buzzerPin, LOW);
}

void EEPROM_SaveSetting()
{
    EEPROM_Setting.EEPROM_OpenAngle = OpenAngle;
    EEPROM_Setting.EEPROM_CloseAngle = CloseAngle;
    EEPROM_Setting.EEPROM_BladeAngle = BladeAngle;
    EEPROM_Setting.EEPROM_CutSpeed = CutSpeed;
    EEPROM_Setting.EEPROM_Diameter = Diameter;
    EEPROM_Setting.EEPROM_WireSpeed = WireSpeed;
    EEPROM_Setting.EEPROM_cuttingSelect = cuttingSelect;

    EEPROM.put(0, EEPROM_Setting);
    EEPROM.commit();
}
void EEPROM_LoadSetting()
{
    EEPROM.get(0, EEPROM_Setting);
    OpenAngle = EEPROM_Setting.EEPROM_OpenAngle;
    CloseAngle = EEPROM_Setting.EEPROM_CloseAngle;
    BladeAngle = EEPROM_Setting.EEPROM_BladeAngle;
    CutSpeed = EEPROM_Setting.EEPROM_CutSpeed;
    Diameter = EEPROM_Setting.EEPROM_Diameter;
    WireSpeed = EEPROM_Setting.EEPROM_WireSpeed;
    cuttingSelect = EEPROM_Setting.EEPROM_cuttingSelect;
}

void chayservo(uint8_t goc)
{
    unsigned long currentMillis = millis(); // Thời gian hiện tại

    if (currentMillis - previousMillis >= interval)
    {
        previousMillis = currentMillis; // Cập nhật thời gian trước đó

        currentAngle += increment;   // Tăng/giảm góc hiện tại
        myservo.write(currentAngle); // Ghi góc hiện tại cho servo

        if (currentAngle == goc || currentAngle == 0)
        {
            increment = -increment; // Đảo chiều tăng/giảm góc
        }
    }
}

// Chạy dây
void moveWire(float wirelength)
{
    uint32_t Steps = wirelength / (LengthPerStep * (float)Diameter); //
    // uint32_t Steps = 200;
    extruderStepper.setCurrentPosition(0);
    while (extruderStepper.currentPosition() != Steps)
    {
        extruderStepper.setSpeed(1000);
        extruderStepper.runSpeed();
    }
}

void moveWireIN()
{
    extruderStepper.setCurrentPosition(0);
    extruderStepper.moveTo(100);                // set vị trí đích đến, số vòng 600 = 3 vòng
    extruderStepper.setSpeed(1000);             // Cho motor chạy với tốc độ 200 bước/s
    while (extruderStepper.distanceToGo() != 0) // 400 bước = 2 vòng ..... currentPosition() trả về vị trí hiện tại của ĐC
    {
        extruderStepper.runSpeedToPosition();
    }
}
void moveWireOUT()
{
    extruderStepper.setCurrentPosition(100);
    extruderStepper.moveTo(0);                  // set vị trí đích đến, số vòng 600 = 3 vòng
    extruderStepper.setSpeed(1000);             // Cho motor chạy với tốc độ 200 bước/s
    while (extruderStepper.distanceToGo() != 0) // 400 bước = 2 vòng ..... currentPosition() trả về vị trí hiện tại của ĐC
    {
        extruderStepper.runSpeedToPosition();
    }
}
// Cắt bằng động cơ Step
void moveBlade(int Steps)
{
    linMotSteppers.setCurrentPosition(0);
    linMotSteppers.moveTo(Steps);
    uint16_t CutSpeedBuffer = map(CutSpeed, 0, 100, 0, SetMaxSpeed);
    linMotSteppers.setSpeed(CutSpeedBuffer);
    while (linMotSteppers.distanceToGo() != 0)
    {
        linMotSteppers.runSpeedToPosition();
    }
}
void BladeUp(uint8_t select)
{
    if (select == 0) // Servo
    {
        myservo.write(OpenAngle);
    }
    if (select == 1 && motorBladeState == 0) // Steper
    {
        motorBladeState = 1;
        moveBlade(200);
    }
}

void BladeDown(uint8_t select)
{
    if (select == 0) // Servo
    {
        myservo.write(CloseAngle);
    }
    if (select == 1 && motorBladeState == 1) // Steper
    {
        motorBladeState = 0;
        moveBlade(-200);
    }
}
void tuotDay()
{
    if (cuttingSelect == 0)
    {
        myservo.write(BladeAngle);
        delay(timeServo1deg * BladeAngle);
        myservo.write(OpenAngle);
        delay(timeServo1deg * BladeAngle);
    }
    else
    {
        // tuốt dây bằng động cơ Step
    }
}

void catDay()
{
    if (cuttingSelect == 0)
    {
        myservo.write(CloseAngle);
        delay(timeServo1deg * CloseAngle);
        myservo.write(OpenAngle);
        delay(timeServo1deg * CloseAngle);
    }
    else
    {
        // cắt dây bằng động cơ Step
    }
}

void drawKeypad()
{
    // Draw the keys
    for (uint8_t row = 0; row < 5; row++)
    {
        for (uint8_t col = 0; col < 3; col++)
        {
            uint8_t b = col + row * 3;

            if (b < 3)
                tft.setFreeFont(LABEL1_FONT);
            else
                tft.setFreeFont(LABEL2_FONT);

            key[b].initButton(&tft, KEY_X + col * (KEY_W + KEY_SPACING_X),
                              KEY_Y + row * (KEY_H + KEY_SPACING_Y), // x, y, w, h, outline, fill, text
                              KEY_W, KEY_H, TFT_WHITE, keyColor[b], TFT_WHITE,
                              keyLabel[b], KEY_TEXTSIZE);
            key[b].drawButton();
        }
    }
}

//------------------------------------------------------------------------------------------
void drawFillRect(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint32_t ColorFill, uint32_t ColorDraw)
{
    tft.fillRect(x, y, w, h, ColorFill);
    tft.drawRect(x, y, w, h, ColorDraw);
}

float TFT_keyboard(float currentValue)
{
    tft.fillRect(0, 0, 320, 240, TFT_DARKGREY);
    // Draw number display area and frame
    tft.fillRect(DISP_X, DISP_Y, DISP_W, DISP_H, TFT_BLACK);
    tft.drawRect(DISP_X, DISP_Y, DISP_W, DISP_H, TFT_WHITE);
    drawKeypad();

    int choke = 0;
    while (!choke)
    {
        uint16_t t_x = 0, t_y = 0; // To store the touch coordinates | Lưu tọa độ cảm ứngs

        // Pressed will be set true is there is a valid touch on the screen
        bool pressed = tft.getTouch(&t_x, &t_y);

        // Check if any key coordinate boxes contain the touch coordinates
        for (uint8_t b = 0; b < 15; b++)
        {
            if (pressed && key[b].contains(t_x, t_y))
            {
                pipp();
                key[b].press(true); // tell the button it is pressed
            }
            else
            {
                key[b].press(false); // tell the button it is NOT pressed
            }
        }

        // Kiểm tra xem nút bàn phím nào thay đổi không
        for (uint8_t b = 0; b < 15; b++)
        {

            if (b < 3)
                tft.setFreeFont(LABEL1_FONT);
            else
                tft.setFreeFont(LABEL2_FONT);

            if (key[b].justReleased())
                key[b].drawButton(); // draw normal

            if (key[b].justPressed())
            {
                key[b].drawButton(true); // draw invert

                // if a numberpad button, append the relevant # to the numberBuffer
                if (b >= 3)
                {
                    if (numberIndex < NUM_LEN)
                    {
                        numberBuffer[numberIndex] = keyLabel[b][0];
                        numberIndex++;
                        numberBuffer[numberIndex] = 0; // zero terminate
                    }
                }

                // Del button, xóa ký tự cuối cùng
                if (b == 1)
                {
                    numberBuffer[numberIndex] = 0;
                    if (numberIndex > 0)
                    {
                        numberIndex--;
                        numberBuffer[numberIndex] = 0;
                    }
                }
                // Send button, gửi chuỗi
                if (b == 2)
                {
                    Serial.println(numberBuffer);
                    // currentValue = numberBuffer.toFloat();
                    currentValue = atof(numberBuffer);
                    choke = 1;
                    numberIndex = 0;               // Reset index to 0
                    numberBuffer[numberIndex] = 0; // Place null in buffer
                }
                // New button, xóa chuỗi vừa nhấn đi
                if (b == 0)
                {
                    numberIndex = 0;               // Reset index to 0
                    numberBuffer[numberIndex] = 0; // Place null in buffer
                }

                // Update the number display field
                tft.setTextDatum(TL_DATUM);       // Use top left corner as text coord datum
                tft.setFreeFont(&FreeSans18pt7b); // Choose a nicefont that fits box
                tft.setTextColor(DISP_TCOLOR);    // Set the font colour

                // Draw the string, the value returned is the width in pixels
                int xwidth = tft.drawString(numberBuffer, DISP_X + 4, DISP_Y + 12);

                // Now cover up the rest of the line up by drawing a black rectangle.  No flicker this way
                // but it will not work with italic or oblique fonts due to character overlap.
                tft.fillRect(DISP_X + 4 + xwidth, DISP_Y + 1, DISP_W - xwidth - 5, DISP_H - 2, TFT_BLACK);

                delay(10); // UI debouncing
            }
        }
    }
    fillScreen1 = 0;
    fillScreenSetting = 0;
    return currentValue;
}

void TFT_Screen1()
{
    if (fillScreen1 == 0)
    {
        fillScreen1 = 1;
        tft.fillScreen(TFT_WHITE);
        tft.setCursor(20, 0);
        tft.setTextFont(2);
        tft.setTextSize(2);
        tft.setTextColor(TFT_BLACK, TFT_WHITE);
        tft.println("Wire stripping cutting");

        tft.fillRect(34, 48, 23, 14, TFT_RED); // Draw Wire
        tft.fillRect(60, 48, 200, 14, TFT_RED);
        tft.fillRect(263, 48, 23, 14, TFT_RED);

        tft.fillRect(11, 66, 63, 20, TFT_YELLOW); //
        tft.drawRect(11, 66, 63, 20, TFT_BLACK);
        tft.fillRect(127, 66, 63, 20, TFT_YELLOW); //
        tft.drawRect(127, 66, 63, 20, TFT_BLACK);
        tft.fillRect(247, 66, 63, 20, TFT_YELLOW); //
        tft.drawRect(247, 66, 63, 20, TFT_BLACK);

        tft.setTextFont(1);
        tft.setTextSize(2);
        tft.setTextColor(TFT_BLACK);
        tft.setCursor(29, 85);
        tft.print("mm");
        tft.setCursor(147, 85);
        tft.print("mm");
        tft.setCursor(268, 85);
        tft.print("mm");

        tft.setCursor(205, 144); // Quantity | Số lượng
        tft.print("Quantity");
        tft.fillRoundRect(198, 166, 109, 36, 9, TFT_GREEN);
        tft.drawRoundRect(198, 166, 110, 37, 9, TFT_BLACK);

        tft.fillRect(235, 205, 83, 33, TFT_BLUE); // Start/Stop button
        tft.drawRect(235, 205, 83, 33, TFT_WHITE);

        tft.fillRect(2, 205, 93, 33, TFT_BLUE); // Setting button
        tft.drawRect(2, 205, 93, 33, TFT_WHITE);

        tft.pushImage(60, 155, 80, 42, EXTRUDE);
        tft.pushImage(8, 117, 42, 80, BLADE);
    }

    tft.setTextFont(1);
    tft.setTextSize(2);
    tft.setTextColor(TFT_BLACK);
    tft.setCursor(13, 69);
    tft.print(StripLength1, 1);

    tft.setCursor(129, 69);
    tft.print(WireLength, 1);

    tft.setCursor(249, 69);
    tft.print(StripLength2, 1);

    tft.setCursor(249, 69);
    tft.print(StripLength2, 1);

    tft.setCursor(253, 175);
    tft.print("/");
    tft.print(Quantity, 0);

    if (StartState == 0)
    {
        tft.setCursor(245, 212);
        tft.setTextColor(TFT_WHITE, TFT_BLUE);
        tft.print("START");
    }

    if (StartState == 1)
    {
        tft.setCursor(245, 212);
        tft.setTextColor(TFT_YELLOW, TFT_BLUE);
        tft.print("PAUSE");
    }

    tft.setCursor(4, 212);
    tft.setTextColor(TFT_WHITE, TFT_BLUE);
    tft.print("SETTING");

    fillScreenSetting = 0;
}

void TFT_ScreenSetting()
{
    if (fillScreenSetting == 0)
    {
        fillScreenSetting = 1;
        tft.fillScreen(TFT_BLACK);

        tft.setTextFont(1);
        tft.setTextSize(2);
        tft.fillRect(12, 9, 143, 44, TFT_YELLOW);
        tft.setTextColor(TFT_BLACK);
        tft.setCursor(33, 13);
        tft.println("CUTTING");
        tft.print("    ANGLE");
        drawFillRect(12, 50, 143, 115, 0x17f1, TFT_WHITE);

        tft.fillRect(165, 9, 143, 44, TFT_YELLOW);
        tft.setCursor(188, 13);
        tft.print(" MOTOR");
        tft.setCursor(188, 33);
        tft.print("SETTING");
        drawFillRect(165, 50, 143, 115, 0x17f1, TFT_WHITE);

        tft.setTextSize(1);
        tft.setCursor(18, 58);
        tft.println("OPEN (deg)");
        tft.setCursor(18, 86);
        tft.println("CLOSE (deg)");
        tft.setCursor(18, 114);
        tft.println("BLADE (deg)");
        tft.setCursor(18, 142);
        tft.println("SPEED (%)");

        tft.setCursor(169, 58);
        tft.println("DIAMETER(mm)");
        tft.setCursor(169, 86);
        tft.println("SPEED (%)");

        drawFillRect(x_Open, y_Open, w_Open, h_Open, TFT_WHITE, TFT_BLACK);
        drawFillRect(x_Close, y_Close, w_Close, h_Close, TFT_WHITE, TFT_BLACK);
        drawFillRect(x_Blade, y_Blade, w_Blade, h_Blade, TFT_WHITE, TFT_BLACK);
        drawFillRect(x_cutSpeed, y_cutSpeed, w_cutSpeed, h_cutSpeed, TFT_WHITE, TFT_BLACK);

        drawFillRect(x_Diameter, y_Diameter, w_Diameter, h_Diameter, TFT_WHITE, TFT_BLACK);
        drawFillRect(x_wireSpeed, y_wireSpeed, w_wireSpeed, h_wireSpeed, TFT_WHITE, TFT_BLACK);
        drawFillRect(168, 132, 137, 30, TFT_YELLOW, TFT_BLACK);

        tft.fillRect(2, 205, 93, 33, TFT_BLUE); // Setting button
        tft.drawRect(2, 205, 93, 33, TFT_WHITE);
    }

    tft.setTextFont(1);
    tft.setTextSize(2);
    tft.setTextColor(TFT_BLACK, TFT_WHITE);
    tft.setCursor(x_Open + 5, y_Open + 5);
    tft.print(OpenAngle);
    tft.setCursor(x_Close + 5, y_Close + 5);
    tft.print(CloseAngle);
    tft.setCursor(x_Blade + 5, y_Blade + 5);
    tft.print(BladeAngle);
    tft.setCursor(x_cutSpeed + 5, y_cutSpeed + 5);
    tft.print(CutSpeed);

    tft.setCursor(x_Diameter + 5, y_Diameter + 5);
    tft.print(Diameter, 1);
    tft.setCursor(x_wireSpeed + 5, y_wireSpeed + 5);
    tft.print(WireSpeed);

    tft.setCursor(168 + 13, 132 + 7);
    tft.setTextColor(TFT_BLACK, TFT_YELLOW);
    if (cuttingSelect == 0)
        tft.print("  SERVO  ");
    else
        tft.print("STEPMOTOR");

    tft.setCursor(4, 212);
    tft.setTextColor(TFT_WHITE, TFT_BLUE);
    tft.print("SAVE >>");

    fillScreen1 = 0;
} // end TFT_ScreenSetting()

void TFT_Display()
{
    if (!SettingPage) // Home Page
    {
        TFT_Screen1();
        uint16_t t_x = 0, t_y = 0;               // coordenadas pulsacion
        bool pressed = tft.getTouch(&t_x, &t_y); // true al pulsa
        // // Comprueba si pulsas en zona de botón
        for (uint8_t b = 0; b < 9; b++)
        {
            if (pressed && key[b].contains(t_x, t_y))
            {
                key[b].press(true);

                // Serial.print(t_x);
                // Serial.print(",");
                // SerUial.println(t_y);
            }
            else
            {
                key[b].press(false);
            }
        }
        // Comprueba si pulsas en imágenes
        if (pressed)
        {
            pipp();
            // Serial.println("pulsado sobre imagen");
            Serial.print(t_x);
            Serial.print(",");
            Serial.println(t_y);
            if (t_x > 7 && t_x < 7 + 71 && t_y > 69 && t_y < 69 + 17)
            { // Strip Length 1 button
                Serial.println("Nut chon StripLength1");
                StripLength1 = TFT_keyboard(StripLength1);
            }
            if (t_x > 124 && t_x < 124 + 71 && t_y > 69 && t_y < 69 + 17)
            { // Wire Length button
                Serial.println("Nut chon WireLength");
                WireLength = TFT_keyboard(WireLength);
            }
            if (t_x > 243 && t_x < 243 + 71 && t_y > 69 && t_y < 69 + 17)
            { // Strip Length 2 button
                Serial.println("Nut chon StripLength2");
                StripLength2 = TFT_keyboard(StripLength2);
            }
            if (t_x > 198 && t_x < 198 + 110 && t_y > 166 && t_y < 166 + 37)
            { // Strip Length 2 button
                Serial.println("Nut chon Quantity");
                Quantity = TFT_keyboard(Quantity);
                // soluong = Quantity;
            }
            if (t_x > 13 && t_x < 13 + 30 && t_y > 120 && t_y < 120 + 30)
            { // Kéo cắt Up
                // Serial.println("Nut chon Quantity");
                BladeUp(cuttingSelect);
            }
            if (t_x > 13 && t_x < 13 + 30 && t_y > 164 && t_y < 164 + 30)
            { // Kéo cắt Down
                // Serial.println("Nut chon Quantity");
                BladeDown(cuttingSelect);
            }
            if (t_x > 107 && t_x < 107 + 30 && t_y > 160 && t_y < 160 + 30)
            { // Nút Kéo dây vào
                // Serial.println("Nut Quantity");
                moveWireIN();
            }
            if (t_x > 64 && t_x < 64 + 30 && t_y > 160 && t_y < 160 + 30)
            { // Nút Kéo dây ra
                // Serial.println("Nut Quantity");
                moveWireOUT();
            }
            if (t_x > 235 && t_x < 235 + 83 && t_y > 205 && t_y < 205 + 33)
            { // Start/Stop button
                Serial.println("Start/Stop button");
                StartState = !StartState;
                delay(100);
            }
            if (t_x > 2 && t_x < 2 + 93 && t_y > 205 && t_y < 205 + 33)
            { // Setting button
                Serial.println("Setting Page button");
                SettingPage = 1;
                delay(100);
            }
        }
        // // Accion si se pulsa boton
        // for (uint8_t b = 0; b < 2; b++) {

        //   if (key[b].justReleased()) {
        //     key[b].drawButton(); // redibuja al soltar

        //     switch (b) {
        //       case 0:
        //         status("system Enabled");
        //         TFT_keyboard();
        //         break;
        //       case 1:
        //         status("system Disabled");
        //         TFT_keyboard();
        //         break;
        //       default:
        //         delay(1);
        //         // statements
        //     }
        //   }
        //   if (key[b].justPressed()) {
        //     key[b].drawButton(true);  // cambia color del botón al pulsar
        //     delay(10); // UI debouncing
        //   }
        // }
    }
    if (SettingPage) // Setting Page
    {
        TFT_ScreenSetting();

        uint16_t t_x = 0, t_y = 0;               // coordenadas pulsacion
        bool pressed = tft.getTouch(&t_x, &t_y); // true al pulsar

        // // Comprueba si pulsas en zona de botón
        for (uint8_t b = 0; b < 2; b++)
        {
            if (pressed && key[b].contains(t_x, t_y))
            {

                key[b].press(true);

                // Serial.print(t_x);
                // Serial.print(",");
                // SerUial.println(t_y);
            }
            else
            {
                key[b].press(false);
            }
        }

        if (pressed)
        {
            pipp();
            Serial.print(t_x);
            Serial.print(",");
            Serial.println(t_y);

            if (t_x > x_Open && t_x < x_Open + w_Open && t_y > y_Open && t_y < y_Open + h_Open)
            { // Nút cài đặt góc cắt Open
                Serial.println("Chon goc Open");
                OpenAngle = TFT_keyboard(OpenAngle);
                delay(100);
            }
            if (t_x > x_Close && t_x < x_Close + w_Close && t_y > y_Close && t_y < y_Close + h_Close)
            { // Nút cài đặt góc cắt Close
                Serial.println("Chon goc Close");
                CloseAngle = TFT_keyboard(CloseAngle);
                delay(100);
            }
            if (t_x > x_Blade && t_x < x_Blade + w_Blade && t_y > y_Blade && t_y < y_Blade + h_Blade)
            { // Nút cài đặt góc cắt Blade
                Serial.println("Chon goc Blade");
                BladeAngle = TFT_keyboard(BladeAngle);
                delay(100);
            }
            if (t_x > x_cutSpeed && t_x < x_cutSpeed + w_cutSpeed && t_y > y_cutSpeed && t_y < y_cutSpeed + h_cutSpeed)
            { // Nút cài đặt tốc độ cắt
                Serial.println("Chon goc CuttingSpeed");
                CutSpeed = TFT_keyboard(CutSpeed);
                delay(100);
            }
            if (t_x > x_Diameter && t_x < x_Diameter + w_Diameter && t_y > y_Diameter && t_y < y_Diameter + h_Diameter)
            { // Nút cài đặt đường kính trục
                Serial.println("Chon Diameter");
                Diameter = TFT_keyboard(Diameter);
                delay(100);
            }
            if (t_x > x_wireSpeed && t_x < x_wireSpeed + w_wireSpeed && t_y > y_wireSpeed && t_y < y_wireSpeed + h_wireSpeed)
            { // Nút cài đặt tốc độ dây
                Serial.println("Chon goc WireSpeed");
                WireSpeed = TFT_keyboard(WireSpeed);
                delay(100);
            }

            if (t_x > 168 && t_x < 168 + 137 && t_y > 132 && t_y < 132 + 30)
            { // Cutting Select, Lựa loại động cơ cắt. 0 = Servo, 1 = StepMotor
                Serial.println("Cutting Select");
                cuttingSelect = !cuttingSelect;
                delay(100);
            }

            if (t_x > 2 && t_x < 2 + 93 && t_y > 205 && t_y < 205 + 33)
            { // Save Button
                Serial.println("Save Page button");
                SettingPage = 0;
                EEPROM_SaveSetting();
                delay(100);
            }
        }
    } //
}

void Handle()
{
    switch (handleStep)
    {
    case step1: // kéo đoạn 1 vào
    {
        moveWire(WireLength);
        handleStep = step2;
    }
    break;
    case step2: // đưa dao tuốt xuống
    {
        t1 = millis();
        myservo.write(BladeAngle);
        handleStep = step3;
    }
    break;
    case step3: // cắt tuốt đoạn 1
    {
        if (cuttingSelect == 0) // servo
        {
            if (millis() - t1 > 2000)
            {
                myservo.write(OpenAngle);
            }
            handleStep = step4;
        }
        else
        {
            // tuốt dây bằng động cơ Step
        }
    }
    break;
    case step4:
    {
        if (cuttingSelect == 0) // servo
        {
            if (millis() - t1 > 4000)
            {
                handleStep = step5;
            }
        }
        else
        {
            // tuốt dây bằng động cơ Step
        }
    }
    break;
    case step5:
    {
        handleStep = step6;
    }
    break;
    case step6:
    {
        handleStep = step7;
    }
    break;
    case step7:
    {
        soluong++;
    }
    break;
    }
}

void PrintMonitor()
{
    Serial.print("StripLength1: ");
    Serial.print(StripLength1);
    Serial.print(" | ");
    Serial.print("StripLength2: ");
    Serial.print(StripLength2);
    Serial.print(" | ");
    Serial.print("WireLength: ");
    Serial.print(WireLength);
    Serial.print(" | ");
    Serial.print("Quantity: ");
    Serial.print(Quantity);
    Serial.print(" | ");
    Serial.print("StepHandle: ");
    Serial.println(handleStep);
}