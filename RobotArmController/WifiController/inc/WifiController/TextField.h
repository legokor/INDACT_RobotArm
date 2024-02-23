#ifndef WIFICONTROLLER_TEXTFIELD_H_
#define WIFICONTROLLER_TEXTFIELD_H_

#include <stdbool.h>
#include <stddef.h>

typedef struct WifiController_TextFieldListElement
{
    char *text;
    struct WifiController_TextFieldListElement *next;
} WifiController_TextFieldListElement_t;

typedef struct WifiController_TextFieldList
{
    WifiController_TextFieldListElement_t *head;
    WifiController_TextFieldListElement_t *tail;
    size_t size;
} WifiController_TextFieldList_t;

void WifiController_TextField_Init(WifiController_TextFieldList_t *tfl);
void WifiController_TextField_Delete(WifiController_TextFieldList_t *tfl);

bool WifiController_TextField_PushBack(WifiController_TextFieldList_t *tfl, const char *text);

int WifiController_TextField_GetSize(const WifiController_TextFieldList_t *tfl);
const char* WifiController_TextField_At(const WifiController_TextFieldList_t *tfl, int index);

#endif /* WIFICONTROLLER_TEXTFIELD_H_ */
