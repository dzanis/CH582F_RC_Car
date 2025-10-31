#include "CONFIG.h"
#include "rccar_service.h"

/* Значения характеристик */
static int16_t rccarBattValue     = 0;     // батарея
static uint8_t rccarControlValue [3];  //  Speed Dir Turn

// Текстовое имя для характеристики  (отображается в nRF Characteristic User Description)  
static uint8_t rccarBattUserDesc[] = "Batt";
static uint8_t rccarControlUserDesc[] = "Speed Dir Turn";

/* Проперти характеристик */
static uint8_t rccarBattProps       = GATT_PROP_READ | GATT_PROP_NOTIFY;
static uint8_t rccarControlProps   =  GATT_PROP_WRITE;

/* UUID (16-bit) */
#define RCCAR_SERVICE_UUID 0xFFF0
#define RCCAR_BATT_VALUE_UUID 0xFFF1
#define RCCAR_CONTROL_VALUE_UUID 0xFFF2

static const uint8_t rccarServiceUUID[ATT_BT_UUID_SIZE] = { LO_UINT16(RCCAR_SERVICE_UUID), HI_UINT16(RCCAR_SERVICE_UUID) };
static const uint8_t rccarBattUUID[ATT_BT_UUID_SIZE]   = { LO_UINT16(RCCAR_BATT_VALUE_UUID), HI_UINT16(RCCAR_BATT_VALUE_UUID) };
static const uint8_t rccarControlUUID[ATT_BT_UUID_SIZE]   = { LO_UINT16(RCCAR_CONTROL_VALUE_UUID), HI_UINT16(RCCAR_CONTROL_VALUE_UUID) };

// Service attribute
static const gattAttrType_t rccarService = {ATT_BT_UUID_SIZE, rccarServiceUUID};

/* Client Characteristic Configuration Descriptor (CCCD)(для Speed notify) */
static gattCharCfg_t rccarBattClientCharCfg[GATT_MAX_NUM_CONN];

/* Минимальная таблица атрибутов */
static gattAttribute_t rccarAttrTbl[] =
    {
        /* Primary Service */
        {{ATT_BT_UUID_SIZE, primaryServiceUUID}, GATT_PERMIT_READ, 0, (uint8_t *)&rccarService},

        /* --- Speed Characteristic --- */
        {{ATT_BT_UUID_SIZE, characterUUID}, GATT_PERMIT_READ, 0, &rccarBattProps},
        {{ATT_BT_UUID_SIZE, rccarBattUUID}, GATT_PERMIT_READ, 0, (uint8_t *)&rccarBattValue},
        {{ATT_BT_UUID_SIZE, clientCharCfgUUID}, GATT_PERMIT_READ | GATT_PERMIT_WRITE, 0, (uint8_t *)&rccarBattClientCharCfg},
        {{ATT_BT_UUID_SIZE, charUserDescUUID },GATT_PERMIT_READ, 0, (uint8_t *)rccarBattUserDesc},

        /* --- Wheel Circumference Characteristic --- */
        {{ATT_BT_UUID_SIZE, characterUUID}, GATT_PERMIT_READ, 0, &rccarControlProps},
        {{ATT_BT_UUID_SIZE, rccarControlUUID}, GATT_PERMIT_READ | GATT_PERMIT_WRITE, 0, (uint8_t *)&rccarControlValue},
        {{ATT_BT_UUID_SIZE, charUserDescUUID },GATT_PERMIT_READ, 0, (uint8_t *)rccarControlUserDesc}
    };

/* позиции характеристик в таблице */
#define BICYCLE_SPEED_VALUE_POS        2
#define BICYCLE_SPEED_CCCD_POS   (BICYCLE_SPEED_VALUE_POS + 1)
#define BICYCLE_WHEEL_CIRC_VALUE_POS   6

// GAP - Advertisement data (max size = 31 bytes, though this is
// best kept short to conserve power while advertising)
static uint8_t advertData[] = {
    // Flags; this sets the device to use limited discoverable
    // mode (advertises for 30 seconds at a time) instead of general
    // discoverable mode (advertises indefinitely)
    0x02, // length of this data
    GAP_ADTYPE_FLAGS,
    GAP_ADTYPE_FLAGS_GENERAL | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED,

    // service UUID, to notify central devices what services are included
    // in this peripheral
    0x03,                  // length of this data
    GAP_ADTYPE_16BIT_MORE, // some of the UUID's, but not all
    LO_UINT16(RCCAR_SERVICE_UUID),
    HI_UINT16(RCCAR_SERVICE_UUID)
};

typedef struct {
    uint16_t * valueHandle;           // <-- указатель на value handle характеристики
    gattCharCfg_t *clientCharCfg;    // <-- CCCD (Client Characteristic Configuration)
} CharacteristicNotify_t;

CharacteristicNotify_t myNotifyBatt ;

void InitNotify(CharacteristicNotify_t *notify, uint16_t *handle, gattCharCfg_t *clientCharCfg);
void findNotifyByUUID(const uint8_t *uuid, uint8_t uuid_len, CharacteristicNotify_t *notify);


/* Callbacks */
static bStatus_t rccar_ReadAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                  uint8_t *pValue, uint16_t *pLen, uint16_t offset,
                                  uint16_t maxLen, uint8_t method);

static bStatus_t rccar_WriteAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                     uint8_t *pValue, uint16_t len, uint16_t offset,
                                     uint8_t method);

static void rccar_NotifySpeedCB(linkDBItem_t *pLinkItem);                                     

/* Service callbacks */
static gattServiceCBs_t rccarCBs =
{
    rccar_ReadAttrCB,
    rccar_WriteAttrCB,
    NULL
};

/* ------------------------------------------------------------------------ */
/* TMOS  */
/* ------------------------------------------------------------------------ */
#define BATT_EVT   0x0001   // событие для нотификации batt
static uint8_t rccarTaskID = INVALID_TASK_ID;;       // ID задачи tmos
uint16_t rccar_ProcessEventCB(uint8_t task_id, uint16_t events);

// Метод вызывается если изменился статус соединения
static void rccar_HandleConnStatusCB(uint16_t connHandle, uint8_t changeType);


/* ------------------------------------------------------------------------ */
/* API функции */
/* ------------------------------------------------------------------------ */

void DRV8833_Init();
void DRV8833_Control(uint8_t speed, uint8_t dir, uint8_t turn);

bStatus_t RcCar_AddService(void)
{
    // ВАЖНО!!! Без рекламы в web ble не обнаружить этот сервисс
    GAPRole_SetParameter(GAPROLE_ADVERT_DATA, sizeof(advertData), advertData);
    // инициализация CCCD для speed notify
    GATTServApp_InitCharCfg(INVALID_CONNHANDLE, rccarBattClientCharCfg);

    // регистрация сервиса
    bStatus_t st = GATTServApp_RegisterService(rccarAttrTbl,
                                       GATT_NUM_ATTRS(rccarAttrTbl),
                                       GATT_MAX_ENCRYPT_KEY_SIZE,
                                       &rccarCBs);

      // Регистрация задачи TMOS (возвращает task id)
    rccarTaskID = TMOS_ProcessEventRegister(rccar_ProcessEventCB);   

     // регистрируем LinkDB callback что-бы знать изменения статуса соединения
    linkDB_Register(rccar_HandleConnStatusCB);

    //findNotifyByUUID(rccarBattUUID, ATT_BT_UUID_SIZE, &myNotifyBatt);
    InitNotify(&myNotifyBatt, &rccarAttrTbl[BICYCLE_SPEED_VALUE_POS].handle, rccarBattClientCharCfg);

    PRINT("RcCar_AddService: reg TMOS taskId=%d, gattRegSt=%d\r\n", rccarTaskID, st);  
    
    DRV8833_Init();

    return st;          
}

/**
 * @brief  Ручная инициализация структуры CharacteristicNotify_t.
 *
 * Используется, когда handle и CCCD уже известны заранее.
 * Просто записывает указатели в структуру для последующей отправки нотификаций.
 *
 * @param notify         Указатель на структуру CharacteristicNotify_t
 * @param handle         Указатель на handle характеристики
 * @param clientCharCfg  Указатель на клиентскую конфигурацию (CCCD)
 */
void InitNotify(CharacteristicNotify_t *notify, uint16_t *valueHandle, gattCharCfg_t *clientCharCfg)
{
    notify->valueHandle = valueHandle;
    notify->clientCharCfg = clientCharCfg;
}

/**
 * @brief  Находит характеристику по UUID и инициализирует структуру уведомления.
 *
 * Ищет в таблице GATT характеристику с указанным UUID.
 * Если найдена — сохраняет указатели на handle и CCCD (clientCharCfg)
 * в структуру CharacteristicNotify_t для последующей работы с нотификациями.
 *
 * @param uuid       UUID характеристики (в прямом порядке байт)
 * @param uuid_len   Длина UUID (2 или 16 байт)
 * @param notify     Указатель на структуру для записи найденных данных
 */
void findNotifyByUUID(const uint8_t *uuid, uint8_t uuid_len, CharacteristicNotify_t *notify)
{
    notify->valueHandle = NULL;
    notify->clientCharCfg = NULL;
    for (uint8_t i = 0; i < GATT_NUM_ATTRS(rccarAttrTbl) - 1; i++)
    {
        gattAttribute_t *attr = &rccarAttrTbl[i];
        if (attr->type.len == uuid_len &&
            tmos_memcmp(attr->type.uuid, uuid, uuid_len)) // tmos_memcmp Возвращает TRUE (1), если одинаковые
        {
            notify->valueHandle = &attr->handle;
            // проверяем следующий элемент (CCCD)
            gattAttribute_t *next = &rccarAttrTbl[i + 1];
            if (next->type.len == ATT_BT_UUID_SIZE &&
                BUILD_UINT16(next->type.uuid[0], next->type.uuid[1]) == GATT_CLIENT_CHAR_CFG_UUID)
            {
                notify->clientCharCfg = (gattCharCfg_t *)next->pValue;
            }
            else
            {
                PRINT("no CCCD found after valueHandle 0x%04X\r\n", attr->handle);
            }
            return;
        }
    }
    PRINT("no UUID found 0x%04X\r\n", BUILD_UINT16(uuid[0],uuid[1]));
}



void NotifyCharacteristic(uint16_t connHandle,CharacteristicNotify_t *notify, const void *data, uint8_t len)
{

    uint16_t cfg = GATTServApp_ReadCharCfg(connHandle, notify->clientCharCfg);
    if (!(cfg & GATT_CLIENT_CFG_NOTIFY))
    {
        return;
    }

    attHandleValueNoti_t noti;
    noti.handle = *notify->valueHandle;
    noti.len = len;
    noti.pValue = GATT_bm_alloc(connHandle, ATT_HANDLE_VALUE_NOTI, len, NULL, 0);

    if (noti.pValue)
    {
        tmos_memcpy(noti.pValue, data, len);
        if (GATT_Notification(connHandle, &noti, FALSE) != SUCCESS)
            GATT_bm_free((gattMsg_t *)&noti, ATT_HANDLE_VALUE_NOTI);
    }
}



/* ------------------------------------------------------------------------ */
/* Callbacks */
/* ------------------------------------------------------------------------ */

void rccar_NotifyCB(linkDBItem_t *pLinkItem)
{
    if(pLinkItem->stateFlags & LINK_CONNECTED)
    {
       NotifyCharacteristic(pLinkItem->connectionHandle,&myNotifyBatt, &rccarBattValue, sizeof(rccarBattValue));
    }

}

/* Callback-функция для отправки уведомления (Notification)
    Вызывается методом linkDB_PerformFunc(rccar_NotifySpeedCB);
    которая последовательно передаёт указатель на 
    структуру каждого соединения linkDBItem_t
*/
void rccar_NotifySpeedCB(linkDBItem_t *pLinkItem)
{
    if(pLinkItem->stateFlags & LINK_CONNECTED)
    {
        uint16_t value = GATTServApp_ReadCharCfg(pLinkItem->connectionHandle,
                                                 rccarBattClientCharCfg);
        if(value & GATT_CLIENT_CFG_NOTIFY)
        {
            attHandleValueNoti_t noti;
             // выделяем память для notify через BLE-heap
            noti.pValue = GATT_bm_alloc(pLinkItem->connectionHandle, ATT_HANDLE_VALUE_NOTI, 2, NULL, 0);
            if(noti.pValue != NULL)// если нет памяти 
            {
               // формируем структуру notify
                noti.handle = rccarAttrTbl[BICYCLE_SPEED_VALUE_POS].handle;
                noti.len    = 2;
                // записываем значение скорости
                noti.pValue[0] = LO_UINT16(rccarBattValue);
                noti.pValue[1] = HI_UINT16(rccarBattValue);
                // отправляем
                if(GATT_Notification(pLinkItem->connectionHandle, &noti, FALSE) != SUCCESS)
                {
                    // освобождаем память
                    GATT_bm_free((gattMsg_t *)&noti, ATT_HANDLE_VALUE_NOTI);
                }
            }
        }
    }

}


/* ------------------------------------------------------------------------
 * Callback обработки чтения характеристик (Read Attribute Callback)
 *
 * Этот калбэк вызывается стеком BLE, когда клиент (например, nRF Connect
 * или смартфон) запрашивает чтение значения характеристики сервиса.
 *
 * В данном случае читается одна характеристика —  (rccarBattValue)
 
 * Если в будущем появятся характеристики разной длины или других типов,
 * следует различать их по UUID uint16_t uuid = BUILD_UINT16(pAttr->type.uuid[0], pAttr->type.uuid[1]);
 * и возвращать значение соответствующей длины.
 * ------------------------------------------------------------------------ */

static bStatus_t rccar_ReadAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                    uint8_t *pValue, uint16_t *pLen, uint16_t offset,
                                    uint16_t maxLen, uint8_t method)
{
    *pLen = 2;
    tmos_memcpy(pValue, pAttr->pValue, 2);
    return SUCCESS;
}

/* ------------------------------------------------------------------------
 * Callback обработки записи характеристик (Write Attribute Callback)
 *
 * Этот калбэк вызывается стеком BLE при поступлении запроса записи (Write Request)
 * от клиента (например, смартфона) к атрибутам нашего сервиса.
 *
 * Обрабатываются два типа UUID:
 *
 * 1. RCCAR_CONTROL_VALUE_UUID — клиент записывает данные для контроля двигателями.
 *    Значение сохраняется в rccarControlValue.
 *
 * 2. GATT_CLIENT_CHAR_CFG_UUID (0x2902) — клиент записывает конфигурацию уведомлений (CCCD)
 *    для характеристики batt. Если клиент включил уведомления (бит Notify),
 *    запускается периодическая отправка rccar_NotifySpeedCB() через TMOS таймер.
 *    Если уведомления выключены — таймер останавливается.
 *
 * Любые другие UUID возвращают ошибку ATT_ERR_ATTR_NOT_FOUND.
 * ------------------------------------------------------------------------ */
static bStatus_t rccar_WriteAttrCB(uint16_t connHandle, gattAttribute_t *pAttr,
                                     uint8_t *pValue, uint16_t len, uint16_t offset,
                                     uint8_t method)
{

    bStatus_t status = SUCCESS;
    uint16_t uuid = BUILD_UINT16(pAttr->type.uuid[0], pAttr->type.uuid[1]);

    switch(uuid)
    {
        case RCCAR_CONTROL_VALUE_UUID:
        if (len == 3)
            {
                DRV8833_Control(pValue[0], pValue[1], pValue[2]);            
                return SUCCESS;
            }
        break;
        case GATT_CLIENT_CHAR_CFG_UUID: // при изменении CCCD
            status = GATTServApp_ProcessCCCWriteReq(connHandle, pAttr, pValue, len,
                                                    offset, GATT_CLIENT_CFG_NOTIFY);
            if (status == SUCCESS)
            {
                // CCCD всегда идёт сразу после характеристики (Value),
                // поэтому handle дескриптора CCCD = Value handle  + 1
                if (pAttr->handle == *myNotifyBatt.valueHandle + 1)
                {
                    uint16_t charCfg = BUILD_UINT16(pValue[0], pValue[1]);
                    if (charCfg & GATT_CLIENT_CFG_NOTIFY) // client ENABLED notifications
                    {
                        PRINT("Batt: client ENABLED notifications\r\n");
                        tmos_start_task(rccarTaskID, BATT_EVT, 1000);
                    }
                    else
                    {
                        PRINT("Batt: client DISABLED notifications\r\n");
                        tmos_stop_task(rccarTaskID, BATT_EVT);
                    }
                }
            }
            break;

        default:
            status = ATT_ERR_ATTR_NOT_FOUND;
            break;
    }

    return (status);
    
}


/* TMOS event processor
   Он вызывается из TMOS_SystemProcess().
*/
uint16_t rccar_ProcessEventCB(uint8_t task_id, uint16_t events)
{
    // Когда придёт BATT_EVT — шлём notify и перезапускаем таймер.
    if (events & BATT_EVT) 
    {  
        rccarBattValue++;
        // тут вызываем нотификацию  для отправки характеристики 
        //linkDB_PerformFunc(rccar_NotifySpeedCB);
        linkDB_PerformFunc(rccar_NotifyCB);


        // перезапускаем таймер на 1s
        tmos_start_task(task_id, BATT_EVT, 1000);

        // пометим событие как обработанное 
        return (events ^ BATT_EVT);
    }

    return 0;
}


/* Метод вызывается если изменился статус соединения  */
static void rccar_HandleConnStatusCB(uint16_t connHandle, uint8_t changeType)
{
    // исключаем loopback (тот же define есть в примерах) 
    if (connHandle != LOOPBACK_CONNHANDLE)
    {
        //запись удалена или stateflags изменились и link уже не up 
        if ((changeType == LINKDB_STATUS_UPDATE_REMOVED) ||
            ((changeType == LINKDB_STATUS_UPDATE_STATEFLAGS) && (!linkDB_Up(connHandle))))
        {
            // Сбрасываем Client Char Config только для этого connectionHandle
            GATTServApp_InitCharCfg(connHandle, rccarBattClientCharCfg);

            // Останавливаем таймер нотификации
            if (rccarTaskID != INVALID_TASK_ID)
            {
                PRINT("Batt: DISABLE notifications\r\n");
                tmos_stop_task(rccarTaskID, BATT_EVT);
            }
            PRINT("Batt: connection %d removed or down\r\n", connHandle);
        }
    }
}

void DRV8833_Init()
{
    // Настройка GPIO 
    // PA12 - PWM4   PA13 - PWM5
    GPIOA_ModeCfg(GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15, GPIO_ModeOut_PP_5mA);

    // Настройка ШИМ
    PWMX_CLKCfg(6);              // такт ШИМ = Fsys / 6 = 32 MHz / 6 = 5.33 MHz
    PWMX_CycleCfg(PWMX_Cycle_255); // период = 255 тактов  Fpwm = 5.33 MHz / 255 = ~20 kHz

}

void DRV8833_Control(uint8_t speed, uint8_t dir, uint8_t turn)
{

    if (dir == 1) // вперёд
    {
        PWMX_ACTOUT(CH_PWM4, speed, High_Level, ENABLE);
        PWMX_ACTOUT(CH_PWM5, 0, High_Level, ENABLE);
    }
    else // назад
    {
        PWMX_ACTOUT(CH_PWM4, 0, High_Level, ENABLE);
        PWMX_ACTOUT(CH_PWM5, speed, High_Level, ENABLE);
    }

    if(turn == 1) // вправо
    {
        GPIOA_SetBits(GPIO_Pin_14); 
        GPIOA_ResetBits(GPIO_Pin_15); 
    }
    else if(turn == 2) // влево
    {
        GPIOA_SetBits(GPIO_Pin_15); 
        GPIOA_ResetBits(GPIO_Pin_14); 
    }
    else // прямо
    {
        GPIOA_ResetBits(GPIO_Pin_14 | GPIO_Pin_15);
    }

}