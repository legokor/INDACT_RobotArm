#ifndef WIFICONTROLLER_TEXTFIELD_H_
#define WIFICONTROLLER_TEXTFIELD_H_

#include <stdbool.h>

typedef struct WifiController_TextFieldListElement
{
    char *text;
    struct WifiController_TextFieldListElement *next;
} WifiController_TextFieldListElement_t;

typedef struct WifiController_TextFieldList
{
    WifiController_TextFieldListElement_t *head;
    WifiController_TextFieldListElement_t *tail;
    int size;
} WifiController_TextFieldList_t;

typedef WifiController_TextFieldList_t WifiController_TextField_t;

void WifiController_TextField_Init(WifiController_TextField_t *this);
void WifiController_TextField_Delete(WifiController_TextField_t *this);

bool WifiController_TextField_PushBack(WifiController_TextField_t *this, const char *text);

int WifiController_TextField_GetSize(const WifiController_TextField_t *this);
const char* WifiController_TextField_At(const WifiController_TextField_t *this, int index);

#endif /* WIFICONTROLLER_TEXTFIELD_H_ */
